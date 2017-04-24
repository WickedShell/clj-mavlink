(ns mavlink.core-test
  (:require [clojure.test :refer :all]
            [clojure.java.io :as io]
            [mavlink.checksum :refer :all]
            [mavlink.core :refer :all]
            [mavlink.type :refer :all]
            [mavlink.mavlink-xml :refer [lookup-enum]])
  (:import [com.mavlink CRC]
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
      :char     (char (+ (byte \A) i))
      :int8_t   (bit-and 0xff (+ 5 i))
      :uint8_t  (bit-and 0xff (+ 5 i))
      :uint8_t_mavlink_version  2
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
                           (aget ^"[B"the-bytes idx)
                           (.get the-bytes idx))))
     (when crc-seed
       (.update-checksum crc crc-seed))
     (.crcValue crc))))

(def mavlink (parse {:xml-sources [{:xml-file "test-include.xml"
                                    :xml-source (-> "test/resources/test-include.xml" io/input-stream)}
                                   {:xml-file "common.xml"
                                    :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                   {:xml-file "uAvionix.xml"
                                    :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}
                                  ]
                     :descriptions true}))

(def channel (open-channel mavlink {:protocol :mavlink1} 99 88))

(defn get-test-message 
  "Given a message's specification map, generate a test message-map for it."
  [{:keys [msg-key fields] :as message-spec}]
  {:pre [msg-key
         (not (empty? fields))]}
  (merge {:message-id msg-key}
         (apply merge (map #(let [{:keys [name-key type-key enum-type length]} %
                                  value (get-test-value type-key  5 length)]
                              {name-key (if enum-type
                                          (lookup-enum (enum-type (:enums-by-group mavlink))
                                                                           value)
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

(deftest include-parse
  (testing "Testing multi file include."
    (is (== (:system-id channel) 99)
        "Include test system id set failed.")
    (is (not (nil? (:uavionix-adsb-out-cfg (:messages-by-keyword (:mavlink channel)))))
        "Include from uAvionix.xml failed.")
    (is (not (nil? (:heartbeat (:messages-by-keyword (:mavlink channel)))))
        "Include from common.xml fialed.")
    (is (not (nil? (:sensor-offsets (:messages-by-keyword (:mavlink channel)))))
        "Include from ardupilotmega.xml failed."))
  (testing "For valid message checksums."
    (is (== (-> (:mavlink channel) :messages-by-keyword :heartbeat :crc-seed) 50)
        "Hearbeat magic byte checksum.")
    (is (== (-> (:mavlink channel) :messages-by-keyword :sys-status :crc-seed) 124)
        "Sys Status magic byte checksum.")
    (is (== (-> (:mavlink channel) :messages-by-keyword :change-operator-control :crc-seed) 217)
        "Change Operator Control magic byte checksum.")
    (is (== (-> (:mavlink channel) :messages-by-keyword :param-set :crc-seed) 168)
        "Param Set magic byte checksum.")
    (is (== (-> (:mavlink channel) :messages-by-keyword :ping :crc-seed) 237)
        "output Raw magic byte checksum.")
    (is (== (-> (:mavlink channel) :messages-by-keyword :servo-output-raw :crc-seed) 222)
        "output Raw magic byte checksum."))
  (testing "Message round trips."
    (doseq [id (range 255)]
      (when-let [msg-info (get (:messages-by-id (:mavlink channel)) id)]
        (let [message (get-test-message msg-info)
              encoded (encode channel message)
              decoded (dissoc (first (decode-bytes channel encoded))
                              :sequence-id :system-id :component-id)]
          (is (= message decoded)
              "Roundtrip failed.")
          (not= message decoded))))))

(deftest simple-parser-test
  (testing "Testing Simple parsing of enums."
    (let [mavlink-simple (parse
                        {:xml-sources [{:xml-file "test-parse.xml"
                                        :xml-source (-> "test/resources/test-parse.xml" io/input-stream)}]
                         :descriptions true})
          channel-simple (open-channel mavlink-simple {:protocol :mavlink1} 99 88)]
      ;(pprint channel-simple)
      (is (thrown-with-msg? Exception #"conflicts" 
                  (parse
                    {:xml-sources [{:xml-file "common.xml"
                                    :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                   {:xml-file "test-parse.xml"
                                    :xml-source (-> "test/resources/test-parse.xml" io/input-stream)}]
                     :descriptions true})))
      (is (thrown-with-msg? Exception #"Unknown message fields"
                   (encode channel-simple {:message-id :heartbeat :dummy-field 0})))
      (is (thrown-with-msg? Exception #"Undefined enum"
                   (encode channel-simple {:message-id :heartbeat :type :dummy-enum})))
      (is (= (:enum-to-value (:mavlink channel-simple))
             {:mav-autopilot-generic 0
              :mav-autopilot-reserved 1
              :mav-autopilot-slugs 2
              :mav-type-generic 0
              :mav-type-fixed-wing 1
             })
          "Enum-to-value test failed.")
      (is (= (:enums-by-group (:mavlink channel-simple))
            {:mav-autopilot
               {0 :mav-autopilot-generic
                1 :mav-autopilot-reserved
                2 :mav-autopilot-slugs}
             :mav-type
               {0 :mav-type-generic
                1 :mav-type-fixed-wing}
             })
          "Enum-by-group test failed")
      (is (= (get (:mav-autopilot (:enums-by-group (:mavlink channel-simple))) 1) :mav-autopilot-reserved)
          "Fetching of enum by its value from enums-by-group failed.")
      (is (= (:fld-name (get (:fields (:heartbeat (:messages-by-keyword (:mavlink channel-simple)))) 3))
               "base_mode")
          "Fetching type of base-mode from heartbeat messages-by-keyword failed.")
      (is (= (:msg-id (:gps-status (:messages-by-keyword (:mavlink channel-simple)))) 25)
          "Fetching id of message from messages-by-keyword failed")
      (is (= (:msg-id (get (:messages-by-id (:mavlink channel-simple)) 25)) 25)
          "Fetching id of message from messages-by-id failed")
      (is (not (nil? (:mav-autopilot (:descriptions (:mavlink channel-simple)))))
          "Failed to find description")
      )))

(deftest ardupilot.xml
  (testing "Testing Ardupilot Messages."
    (let [mavlink-complex (parse
                        {:xml-sources [{:xml-file "ardupilotmega.xml"
                                        :xml-source (-> "test/resources/ardupilotmega.xml" io/input-stream)}
                                       {:xml-file "common.xml"
                                        :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                       {:xml-file "uAvionix.xml"
                                        :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                         })
          channel-complex (open-channel mavlink-complex {:protocol :mavlink1} 99 88)
          simple-message (do
                           (is (nil? (decode-byte-mavlink1 channel-complex (.byteValue (Long/valueOf "fe" 16)))))
                           (is (nil? (decode-byte-mavlink1 channel-complex (byte 3))))
                           (is (nil? (decode-byte-mavlink1 channel-complex (.byteValue (Long/valueOf "e3" 16)))))
                           (is (nil? (decode-byte-mavlink1 channel-complex (byte 1))))
                           (is (nil? (decode-byte-mavlink1 channel-complex (byte 1))))
                           (is (nil? (decode-byte-mavlink1 channel-complex (.byteValue (Long/valueOf "a5" 16)))))
                           (is (nil? (decode-byte-mavlink1 channel-complex (.byteValue (Long/valueOf "d0" 16)))))
                           (is (nil? (decode-byte-mavlink1 channel-complex (byte 18))))
                           (is (nil? (decode-byte-mavlink1 channel-complex (byte 0))))
                           (is (nil? (decode-byte-mavlink1 channel-complex (.byteValue (Long/valueOf "3b" 16)))))
                           (decode-byte-mavlink1 channel-complex (.byteValue (Long/valueOf "d2" 16))))
                           ;(is (nil? (decode-byte channel-complex (.byteValue (Long/valueOf "fe" 16)))))
                           ;(is (nil? (decode-byte channel-complex (byte 3))))
                           ;(is (nil? (decode-byte channel-complex (.byteValue (Long/valueOf "e3" 16)))))
                           ;(is (nil? (decode-byte channel-complex (byte 1))))
                           ;(is (nil? (decode-byte channel-complex (byte 1))))
                           ;(is (nil? (decode-byte channel-complex (.byteValue (Long/valueOf "a5" 16)))))
                           ;(is (nil? (decode-byte channel-complex (.byteValue (Long/valueOf "d0" 16)))))
                           ;(is (nil? (decode-byte channel-complex (byte 18))))
                           ;(is (nil? (decode-byte channel-complex (byte 0))))
                           ;(is (nil? (decode-byte channel-complex (.byteValue (Long/valueOf "3b" 16)))))
                           ;(decode-byte channel-complex (.byteValue (Long/valueOf "d2" 16))))
          decoded (first (decode-bytes channel-complex (encode channel-complex {:message-id :heartbeat :type :mav-type-helicopter
                                                          :autopilot :mav-autopilot-ardupilotmega
                                                          :base_mode :mav-mode-auto-armed
                                                          :system_status :mav-state-poweroff})))
          ; the following tests two header decoding errors, the first is that the message id is wrong,
          ; then there is another spurious start byte if the following message decodes, which it should then
          ; we're good.
          bad-message (is (nil? (decode-bytes channel-complex (mkbytes "fe 49 e0 1 1 03 0 0 0 fe 93 2 63 0 49 4e 49 54 49 41 4c 5f 4d 4f 44 45 0 0 0 0 2 bb 4c")))
                          "Bad message id, entire message should be eaten.")
          multi-message (decode-bytes channel-complex (mkbytes "fe 19 e0 1 1 16 0 0 0 0 93 2 63 0 49 4e 49 54 49 41 4c 5f 4d 4f 44 45 0 0 0 0 2 bb 4c fe 19 e1 1 1 16 0 c0 5a 45 93 2 64 0 4c 49 4d 5f 52 4f 4c 4c 5f 43 44 0 0 0 0 0 4 79 73 fe 1c e2 1 1 a3 43 50 ba b9 3d 7e 23 bb 35 2d e2 3a 0 0 0 0 0 0 0 0 e0 51 ce 3a 70 6e 2 3b 9c 21"))
          three-part-message (do
                               (is (nil? (decode-bytes channel-complex (mkbytes "fe 1c e2 1 1 a3 43 50 ba b9"))))
                               (is (nil? (decode-bytes channel-complex (mkbytes "3d 7e 23 bb 35 2d e2 3a 0 0 0"))))
                               (decode-bytes channel-complex (mkbytes "0 0 0 0 0 e0 51 ce 3a 70 6e 2 3b 9c 21 fe 9 7 ff be")))
          partial-message (decode-bytes channel-complex (mkbytes "0 0 0 0 0 6 8 0 0 0 c7 18"))
          ]
    (comment
      (println "Multi message :")
      (pprint multi-message)
      (println "Simple message :")
      (pprint simple-message)
      (println "Three part message :")
      (pprint three-part-message)
      (println "Partial message :")
      (pprint partial-message)
    )
      (is (= (decode-bytes channel-complex (mkbytes "20 10 30 40 50 fe 19 e0 1 1 16 0 0 0 0 93 2 63 0 49 4e 49 54 49 41 4c 5f 4d 4f 44 45 0 0 0 0 2 bb 4c"))
            [{:message-id :param-value
              :sequence-id 224
              :system-id 1
              :component-id 1
              :param_value 0.0
              :param_count 659
              :param_index 99
              :param_id "INITIAL_MODE"
              :param_type :mav-param-type-int8}])
          "Leading garbage bytes.")
      (is (nil? (:mav-autopilot (:descriptions channel-complex)))
          "Should not be a description")
      (is (= (:type decoded) :mav-type-helicopter)
          "Heartbeat type failed")
      (is (= (:autopilot decoded) :mav-autopilot-ardupilotmega)
          "Heartbeat autopilot failed")
      (is (= (:base_mode decoded) 220)
          "Heartbeat base_mode failed")
      (is (= (:system_status decoded) :mav-state-poweroff)
          "Heartbeat system_status failed")
      (is (= (count multi-message) 3)
          "Decode multiple messages failed")
      (is (= "INITIAL_MODE" (:param_id (get multi-message 0)))
          "Decode param_id from multi-message [0] failed")
      (is (= 659 (:param_count (get multi-message 1)))
          "Decode param_count from multi-message [1] failed")
      (is (== 0.0 (:accel_weight (get multi-message 2)))
          "Decode accel_weight from multi-message [2] failed")
      (is (= {:message-id :hwstatus :sequence-id 227 :system-id 1 :component-id 1 :Vcc 4816 :I2Cerr 0}
             simple-message)
          "Decode simple-message (decoded a byte at a time failed")
      (is (= (str (:omegaIz (first three-part-message))) (str 0.0017255904)))
      (is (= (:message-id (first three-part-message)) :ahrs))
      (is (= (str (:omegaIx (first three-part-message))) (str -3.5536484E-4)))
      (is (= (str (:error_yaw (first three-part-message))) (str 0.0019902252)))
      (is (= (:sequence-id (first three-part-message)) 226))
      (is (= (str (:accel_weight (first three-part-message))) (str 0.0)))
      (is (= (str (:error_rp (first three-part-message))) (str 0.0015740953)))
      (is (= (str (:renorm_val (first three-part-message))) (str 0.0)))
      (is (= (:component-id (first three-part-message)) 1))
      (is (= (str (:omegaIy (first three-part-message))) (str -0.002494707)))
      (is (= (:system-id (first three-part-message)) 1))
      (is (thrown-with-msg? Exception #"Unknown message fields"
          (encode channel-complex {:message-id :gps-status :satellite_prn_BAD [1 2 3 4 5]})))
      (is (thrown-with-msg? Exception #"Undefined enum"
          (encode channel-complex {:message-id :heartbeat :type :not-an-enum})))
      (is (= (:satellite_prn (first (decode-bytes channel-complex (encode channel-complex {:message-id :gps-status :satellite_prn [1 2 3 4 5]}))))
             [1 2 3 4 5 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]))
      (is (= {:message-id :heartbeat
              :autopilot :mav-autopilot-invalid
              :base_mode 0
              :custom_mode 0
              :type :mav-type-gcs
              :sequence-id 7
              :system_status :mav-state-uninit
              :mavlink_version 0
              :component-id 190
              :system-id 255}
             (first partial-message))
          "Decode of message in two parts failed.")
        )))


