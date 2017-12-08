(ns mavlink.core-test
  (:require [clojure.test :refer :all]
            [clojure.java.io :as io]
            [clojure.core.async :as async]
            [mavlink.checksum :refer :all]
            [mavlink.core :refer :all]
            [mavlink.type :refer :all]
            [mavlink.test_utilities :refer :all])
  (:import [com.mavlink CRC]
           [java.io PipedInputStream PipedOutputStream]
           [java.nio ByteBuffer ByteOrder]))

(use 'clojure.pprint)

; The string version produces message specific magic bytes
; the other version returns the full checksum
(defn compute-crc-checksum
  ([^String s]
  (let [the-bytes (.getBytes s)
        crc (new CRC)]
    (doseq [b the-bytes]
      (.update-checksum crc (bit-and (long b) 0xff)))
    (bit-xor (.getMSB ^CRC crc) (.getLSB ^CRC crc))))
  ([the-bytes start-idx last-idx crc-seed]
   (let [crc (new CRC)]
     (doseq [idx (range start-idx last-idx)]
       (.update-checksum crc
                         (if (.isArray (class the-bytes))
                           (aget ^bytes the-bytes idx)
                           (.get ^java.nio.ByteBuffer the-bytes (int idx)))))
     (when crc-seed
       (.update-checksum crc crc-seed))
     (.crcValue crc))))

(def decode-input-pipe (mk-pipe))
(def decode-output-channel (async/chan 300))
(def encode-input-channel  (async/chan 300))
(def encode-output-channel (async/chan 300))

(defn encode-oneway
  "Given a message map, encode it. Bute don't poll for the result because
   an error is expected."
  [message]
  (async/>!! encode-input-channel message))

(defn encode-roundtrip
  "Given a message map, encode it then get the decoded and round robin it back to the encode
   and compare the result."
  [message]
  (async/>!! encode-input-channel message)
  (when-let [message-bytes (async/<!! encode-output-channel)]
    ; (println "The encoded bytes:")
    ; (doseq [b message-bytes] (print (str (bit-and 0xff b) " "))) (println)
    (.write ^PipedOutputStream (:pipe-out decode-input-pipe) message-bytes 0 (count message-bytes))
    (.flush ^PipedOutputStream (:pipe-out decode-input-pipe))
    (async/<!! decode-output-channel)))


(def mavlink (parse {:xml-sources [{:xml-file "ardupilotmega.xml"
                                      :xml-source (-> "test/resources/ardupilotmega.xml" io/input-stream)}
                                     {:xml-file "common.xml"
                                      :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                     {:xml-file "uAvionix.xml"
                                      :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]}))

(def channel  (open-channel mavlink {:protocol :mavlink1
                                     :system-id 99
                                     :component-id 88
                                     :link-id 77
                                     :decode-input-stream (:pipe-in decode-input-pipe)
                                     :decode-output-channel decode-output-channel
                                     :encode-input-channel encode-input-channel
                                     :encode-output-link encode-output-channel
                                     :report-error #(pprint %)
                                     :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                                     :signing-options {:accept-message-handler
                                                              #(do
                                                                (println "clj-mavlink/accept-message:\n" %1)
                                                                true)
                                                       }}))

(defn get-test-message 
  "Given a message's specification map, generate a test message-map for it."
  [{:keys [msg-key fields] :as message-spec}]
  {:pre [msg-key
         (not (empty? fields))]}
  (merge {:message-id msg-key}
         (apply merge (map #(let [{:keys [name-key type-key enum-type length bitmask]} %
                                  enum-group (enum-type (:enums-by-group mavlink))]
                              {name-key (if (and bitmask enum-type)
                                          (get-test-bitmask enum-group)
                                          (let [value (get-test-value type-key  5 length)]
                                            (if enum-type
                                              (get enum-group value value)
                                              value)))})
                           fields))))

(deftest clj-mavlink-utilities
  (testing "checksums"
    (let [s "this is a string test"
          byte-string "fe 19 e0 1 1 16 0 0 0 0 93 2 63 0 49 4e 49 54 49 41 4c 5f 4d 4f 44 45 0 0 0 0 2 bb 4c"
          len-byte-string (count (clojure.string/split s #" "))
          some-bytes (mkbytes byte-string)
          buffer (ByteBuffer/wrap some-bytes)
          crc-seed (byte 55)
          mk-crc-seed (fn [checksum]
                          (bit-xor (bit-and checksum 0xFF)
                                   (bit-and (bit-shift-right checksum 8) 0xff)))]
      (.order buffer ByteOrder/LITTLE_ENDIAN)
      ; note that compute-crc-checksum makes the magic byte for strings
      ; this is to make it as much like the Java MAVlink packet as possible.
      (is (== (mk-crc-seed (compute-checksum (.getBytes s)))
              (compute-crc-checksum s))
          "checksum string")
      (is (== (compute-crc-checksum some-bytes 0 len-byte-string nil)
              (compute-crc-checksum buffer 0 len-byte-string nil))
          "checksum byte array with no magic byte")
      (is (== (compute-crc-checksum some-bytes 0 len-byte-string crc-seed)
              (compute-crc-checksum buffer 0 len-byte-string crc-seed))
          "checksum byte array with magic byte")
      (is (== (compute-checksum some-bytes 1 15 nil)
              (compute-crc-checksum some-bytes 1 15 nil)
              (compute-checksum buffer 1 15 nil)
              (compute-crc-checksum buffer 1 15 nil))
          "checksum byte array 15 bytes no magic byte")
      (is (== (compute-checksum some-bytes 1 15 crc-seed)
              (compute-crc-checksum some-bytes 1 15 crc-seed)
              (compute-checksum buffer 1 15 crc-seed)
          (compute-crc-checksum buffer 1 15 crc-seed))
          "checksum byte array 15 bytes with magic byte")))

  (testing "typed-read-write"
    (let [buffer (ByteBuffer/allocate 200)
          test-values (atom {})]
      (.order buffer ByteOrder/LITTLE_ENDIAN)
      (doseq [type-key [:uint64_t :int64_t :double :uint32_t :int32_t
                        :float :uint16_t :int16_t :uint8_t
                        :uint8_t_mavlink_version :int8_t :char]
              :let [write-fn (type-key write-payload)
                    test-value (get-test-value type-key 5)]]
        (is write-fn
            (str "write function not defined for " type-key))
        (swap! test-values assoc type-key test-value)
        (write-fn buffer test-value))
      (.position buffer 0)
      (doseq [type-key [:uint64_t :int64_t :double :uint32_t :int32_t
                        :float :uint16_t :int16_t :uint8_t
                        :uint8_t_mavlink_version :int8_t :char]
              :let [read-fn (type-key read-payload)]]
        (is read-fn
            (str "read function not defined for " type-key))
        (is (= (type-key @test-values)
                (read-fn buffer))
            (str "roundtrip data write-read for " type-key " failed."))))))

(deftest simple-parser-test
  (testing "Testing Simple parsing of enums."
    (let [mavlink-simple (parse
                        {:xml-sources [{:xml-file "test-parse.xml"
                                        :xml-source (-> "test/resources/test-parse.xml" io/input-stream)}]
                         :descriptions true})]
      (is (thrown-with-msg? Exception #"Enum values conflict" 
                  (parse
                    {:xml-sources [{:xml-file "common.xml"
                                    :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                   {:xml-file "test-parse.xml"
                                    :xml-source (-> "test/resources/test-parse.xml" io/input-stream)}]
                     :descriptions true})))
      ;(is (thrown-with-msg? Exception #"Unable to translate enum"
      ;             (encode channel-simple {:message-id :heartbeat :type :dummy-enum})))
      (is (= (:enum-to-value mavlink-simple)
             {:mav-autopilot-generic 0,
              :mav-autopilot-reserved 1,
              :mav-autopilot-slugs 2,
              :mav-cmd-ack-ok 0,
              :mav-cmd-ack-err-fail 1,
              :mav-cmd-ack-err-access-denied 2,
              :mav-cmd-ack-err-not-supported 3,
              :mav-cmd-ack-err-coordinate-frame-not-supported 4,
              :mav-cmd-ack-err-coordinates-out-of-range 5,
              :mav-cmd-ack-err-x-lat-out-of-range 6,
              :mav-cmd-ack-err-y-lon-out-of-range 7,
              :mav-cmd-ack-err-z-alt-out-of-range 8,
              :mav-type-generic 0,
              :mav-type-fixed-wing 1,
              :mav-state-uninit 0,
              :mav-state-boot 1,
              :mav-state-calibrating 2,
              :mav-state-standby 3,
              :mav-state-active 4,
              :mav-state-critical 5,
              :mav-state-emergency 6,
              :mav-state-poweroff 7
              :mav-test-five 5,
              :mav-test-six 6,
              :mav-test-ten 10
              :mav-test-eleven 11})
          "Enum-to-value test failed.")
      (is (= (:enums-by-group mavlink-simple)
             {:mav-test {5 :mav-test-five,
                         6 :mav-test-six,
                         10 :mav-test-ten
                         11 :mav-test-eleven}
              :mav-autopilot {0 :mav-autopilot-generic,
                              1 :mav-autopilot-reserved,
                              2 :mav-autopilot-slugs},
              :mav-cmd-ack {0 :mav-cmd-ack-ok,
                            1 :mav-cmd-ack-err-fail,
                            2 :mav-cmd-ack-err-access-denied,
                            3 :mav-cmd-ack-err-not-supported,
                            4 :mav-cmd-ack-err-coordinate-frame-not-supported,
                            5 :mav-cmd-ack-err-coordinates-out-of-range,
                            6 :mav-cmd-ack-err-x-lat-out-of-range,
                            7 :mav-cmd-ack-err-y-lon-out-of-range,
                            8 :mav-cmd-ack-err-z-alt-out-of-range},
              :mav-type {0 :mav-type-generic,
                         1 :mav-type-fixed-wing},
              :mav-state {0 :mav-state-uninit,
                          1 :mav-state-boot,
                          2 :mav-state-calibrating,
                          3 :mav-state-standby,
                          4 :mav-state-active,
                          5 :mav-state-critical,
                          6 :mav-state-emergency,
                          7 :mav-state-poweroff}})
          "Enum-by-group test failed")
      (is (= (get (:mav-autopilot (:enums-by-group mavlink-simple)) 1) :mav-autopilot-reserved)
          "Fetching of enum by its value from enums-by-group failed.")
      (is (= (:fld-name (get (:fields (:heartbeat (:messages-by-keyword mavlink-simple))) 3))
               "base_mode")
          "Fetching type of base-mode from heartbeat messages-by-keyword failed.")
      (is (= (:msg-id (:gps-status (:messages-by-keyword mavlink-simple))) 25)
          "Fetching id of message from messages-by-keyword failed")
      (is (= (:msg-id (get (:messages-by-id mavlink-simple) 25)) 25)
          "Fetching id of message from messages-by-id failed")
      (is (not (nil? (:mav-autopilot (:descriptions mavlink-simple))))
          "Failed to find description")
      )))

(deftest mavlink-core
  (testing "Testing multi file include."
    (is (not (nil? (:uavionix-adsb-out-cfg (:messages-by-keyword mavlink))))
        "Include from uAvionix.xml failed.")
    (is (not (nil? (:heartbeat (:messages-by-keyword mavlink))))
        "Include from common.xml fialed.")
    (is (not (nil? (:sensor-offsets (:messages-by-keyword mavlink))))
        "Include from ardupilotmega.xml failed."))
  (testing "For valid message checksums."
    (is (== (-> mavlink :messages-by-keyword :heartbeat :crc-seed) 50)
        "Hearbeat magic byte checksum.")
    (is (== (-> mavlink :messages-by-keyword :sys-status :crc-seed) 124)
        "Sys Status magic byte checksum.")
    (is (== (-> mavlink :messages-by-keyword :change-operator-control :crc-seed) 217)
        "Change Operator Control magic byte checksum.")
    (is (== (-> mavlink :messages-by-keyword :param-set :crc-seed) 168)
        "Param Set magic byte checksum.")
    (is (== (-> mavlink :messages-by-keyword :ping :crc-seed) 237)
        "output Raw magic byte checksum.")
    (is (== (-> mavlink :messages-by-keyword :servo-output-raw :crc-seed) 222)
        "output Raw magic byte checksum.")))
