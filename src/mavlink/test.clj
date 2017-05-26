(ns mavlink.test
  (:require [clojure.java.io :as io]
            [mavlink.core :refer :all])
  (:import  [java.nio ByteBuffer ByteOrder]
            [com.MAVLink MAVLinkPacket Parser]
            [com.MAVLink.common msg_heartbeat]
            [com.MAVLink.enums MAV_AUTOPILOT MAV_STATE MAV_TYPE]))

(use 'clojure.pprint)

(def mavlink-map
          (parse {:xml-sources [{:xml-file "ardupilotmega.xml"
                                 :xml-source (-> "test/resources/ardupilotmega.xml" io/input-stream)}
                                {:xml-file "common.xml"
                                 :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                {:xml-file "uAvionix.xml"
                                 :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}
                               ]
                    }))

(def channel (open-channel mavlink-map {:protocol :mavlink2
                                        :system-id 99
                                        :component-id 88
                                        :link-id 77}))

(defn mk-bytes
  [^String s]
  (into-array Byte/TYPE (mapv #(.byteValue (Long/valueOf % 16)) (clojure.string/split s #" "))))


(defonce ^Parser mavParser
  (let [parser (new Parser true)]
    (set! (.ignoreRadioPackets (.stats parser)) true)
    parser))

(defn encode-heartbeat-packet
  []
    (let [msg (new msg_heartbeat)]
      (set! (.custom_mode msg) 0) ; Autopilot specific bit flags (int)
      (set! (.type msg) MAV_TYPE/MAV_TYPE_GCS)
      (set! (.autopilot msg) MAV_AUTOPILOT/MAV_AUTOPILOT_INVALID)
      (set! (.base_mode msg) 0)   ; bit field, clearing all
      (set! (.system_status msg) MAV_STATE/MAV_STATE_UNINIT)
      (.encodePacket (.pack msg))))

(defn java-decode-byte
  [a-byte]
  (let [x (bit-and a-byte 0xff)
        ^MAVLinkPacket message (.mavlink_parse_char mavParser x)]
    (when message
      (.unpack message))))

(defn java-decode-bytes
  ([^bytes some-bytes]
   {:pre [(= (Class/forName "[B") (class some-bytes))]}
   (java-decode-bytes some-bytes (alength some-bytes)))
  ([^bytes some-bytes num-bytes]
   {:pre [(= (Class/forName "[B") (class some-bytes))]}
   (loop [idx 0
          messages []]
     (if (>= idx num-bytes)
       (when-not (empty? messages)
         messages)
       (if-let [message (java-decode-byte (aget some-bytes idx))]
         (recur (inc idx)
                (conj messages message))
         (recur (inc idx)
                messages))))))

