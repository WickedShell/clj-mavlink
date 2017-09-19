(ns mavlink.core-test
  (:require [clojure.test :refer :all]
            [clojure.java.io :as io]
            [clojure.core.async :as async]
            [mavlink.checksum :refer :all]
            [mavlink.core :refer :all]
            [mavlink.type :refer :all])
  (:import [com.mavlink CRC]
           [java.io PipedInputStream PipedOutputStream]
           [java.nio ByteBuffer ByteOrder]))

(use 'clojure.pprint)

(defn mkbytes
  [^String s]
  (into-array Byte/TYPE (mapv #(.byteValue (Long/valueOf % 16))
                              (clojure.string/split s #" "))))

(defn get-test-value
  "Given a type, return a test value specific for that type.
   Note that if it is an unknown type, a simple 0 is returned.
   If the length is given, then generate a full array of test values."
  ([type-key i]
    (condp = type-key
      :char     (char (+ (byte \A) (mod i 26)))
      :int8_t   (bit-and 0xff (+ 5 i))
      :uint8_t  (bit-and 0xff (+ 5 i))
      :uint8_t_mavlink_version  3
      :int16_t  (bit-and 0xffff (+ 17235 (* i 52)))
      :uint16_t (bit-and 0xffff (+ 17235 (* i 52)))
      :int32_t  (bit-and 0xffffffff (+ 963497464 (* i 52)))
      :uint32_t (bit-and 0xffffffff (+ 963497464 (* i 52)))
      :float    (float (+ 17.0 (* i 7)))
      :double   (float (+ 177777.0 (* i 7)))
      :int64_t   9223372036854775807
      :uint64_t  (bigint (new java.math.BigInteger "9223372036854775807"))
      0))
  ([type-key i length]
   (if length
     (if (= type-key :char)
       (reduce str (map #(get-test-value type-key %) (range length)))
       (reduce conj [] (map #(get-test-value type-key %) (range length))))
     (get-test-value type-key i))))

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

(defn mk-pipe
  []
  (let [pipe-in (PipedInputStream.)
        pipe-out (PipedOutputStream. pipe-in)]
    {:pipe-in pipe-in
     :pipe-out pipe-out}))

(def decode-input-pipe (mk-pipe))
(def decode-output-channel (async/chan 300))
(def encode-input-channel  (async/chan 300))
(def encode-output-channel (async/chan 300))

(def secret-keyset [(bytes (byte-array (map (comp byte int) "000000abcdefghijklmnopqrstuvwxyz")))
                    (bytes (byte-array (map (comp byte int) "abcdefghijklmnopqrstuvwxyz123456")))])
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
    (async/<!! decode-output-channel)))


(def mavlink (parse {:xml-sources [{:xml-file "test-include.xml"
                                    :xml-source (-> "test/resources/test-include.xml" io/input-stream)}
                                   {:xml-file "common.xml"
                                    :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                   {:xml-file "uAvionix.xml"
                                    :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}
                                  ]
                     :descriptions true}))

(def mavlink-2 (parse {:xml-sources [{:xml-file "ardupilotmega.xml"
                                      :xml-source (-> "test/resources/ardupilotmega.xml" io/input-stream)}
                                     {:xml-file "common.xml"
                                      :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                     {:xml-file "uAvionix.xml"
                                      :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]}))

(def channel  (open-channel mavlink-2 {:protocol :mavlink1
                                     :system-id 99
                                     :component-id 88
                                     :link-id 77
                                     :decode-input-stream (:pipe-in decode-input-pipe)
                                     :decode-output-channel decode-output-channel
                                     :encode-input-channel encode-input-channel
                                     :encode-output-link encode-output-channel
                                     :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                                     :signing-options {:secret-key (get secret-keyset 0)
                                                       :secret-keyset secret-keyset
                                                       :accept-message-handler
                                                              #(do
                                                                (println "clj-mavlink/accept-message:\n" %1)
                                                                true)
                                                       }}))

(defn get-test-message 
  "Given a message's specification map, generate a test message-map for it.
   NOTE uses mavlink-2!!!!!"
  [{:keys [msg-key fields] :as message-spec}]
  {:pre [msg-key
         (not (empty? fields))]}
  (merge {:message-id msg-key}
         (apply merge (map #(let [{:keys [name-key type-key enum-type length]} %
                                  value (get-test-value type-key  5 length)]
                              {name-key (if enum-type
                                          (get (enum-type (:enums-by-group mavlink-2))
                                               value value)
                                          value)})
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
      (is (== (mk-crc-seed (compute-checksum s))
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

(deftest mavlink
  (testing "Testing multi file include."
    (is (not (nil? (:uavionix-adsb-out-cfg (:messages-by-keyword mavlink-2))))
        "Include from uAvionix.xml failed.")
    (is (not (nil? (:heartbeat (:messages-by-keyword mavlink-2))))
        "Include from common.xml fialed.")
    (is (not (nil? (:sensor-offsets (:messages-by-keyword mavlink-2))))
        "Include from ardupilotmega.xml failed."))
  (testing "For valid message checksums."
    (is (== (-> mavlink-2 :messages-by-keyword :heartbeat :crc-seed) 50)
        "Hearbeat magic byte checksum.")
    (is (== (-> mavlink-2 :messages-by-keyword :sys-status :crc-seed) 124)
        "Sys Status magic byte checksum.")
    (is (== (-> mavlink-2 :messages-by-keyword :change-operator-control :crc-seed) 217)
        "Change Operator Control magic byte checksum.")
    (is (== (-> mavlink-2 :messages-by-keyword :param-set :crc-seed) 168)
        "Param Set magic byte checksum.")
    (is (== (-> mavlink-2 :messages-by-keyword :ping :crc-seed) 237)
        "output Raw magic byte checksum.")
    (is (== (-> mavlink-2 :messages-by-keyword :servo-output-raw :crc-seed) 222)
        "output Raw magic byte checksum."))
  (testing "Message round trips."
    (doseq [id (range 255)]
      (when-let [msg-info (get (:messages-by-id mavlink-2) id)]
        (let [message (get-test-message msg-info)
              decoded-message (encode-roundtrip message)]
          (println (str "-- Testing message " (:message-id message))) ; " :: " message))
          (doseq [field (keys message)
                  :let [result (if (number? (field message))
                                 (when (number? (field decoded-message))
                                   (== (field message) (field decoded-message)))
                                 (= (field message) (field decoded-message)))]]
            (is result
              (str "message " (:message-id message) " field " field
                   " failed: " (field message) " -> " (field decoded-message))))))))
  (testing "automatic protocol change from MAVlink 1 to MAVlink 2"
    (let [statistics (:statistics channel)]
      (encode-oneway {:message-id :device-op-read})
      (Thread/sleep 1) ; give the encode a thread an opportunity to run
      (is (== 1 (:bad-protocol @statistics))
          "MAVlink 2 only message should fail to encode due to bad protocol")
      (let [decoded-message (encode-roundtrip {:message-id :heartbeat :mavlink-protocol :mavlink2})]
        (is decoded-message
            "Failed to send MAVlink 2 heartbeat"))
      (let [decoded-message (encode-roundtrip {:message-id :device-op-read})]
        (is decoded-message
          "Failed to send MAVlink 2 only test message"))
      (is (== 1 (:bad-protocol @(:statistics channel)))
          "Second attempt to send MAVlink 2 only message should pass.")
      )
    )
  (testing "Roundtrip of all messages."
    (println (str "There are " (count (vals (:messages-by-id mavlink-2))) " messages."))
    (doseq [msg-info (vals (:messages-by-id mavlink-2))]
        (let [message (get-test-message msg-info)
              decoded-message (encode-roundtrip message)]
          (println (str "-- Testing message " (:message-id message) " :: " message))
          ; (pprint decoded-message)
          (doseq [field (keys message)
                  :let [result (if (number? (field message))
                                 (when (number? (field decoded-message))
                                   (== (field message) (field decoded-message)))
                                 (= (field message) (field decoded-message)))]]
            (is result
              (str "message " (:message-id message) " field " field
                   " failed: " (field message) " -> " (field decoded-message)))))))

    (let [statistics (:statistics channel)]
      (println "\n\nMavlink Statistics")
      (pprint @statistics))
  )

; retest all messages.
