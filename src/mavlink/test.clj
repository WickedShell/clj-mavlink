(ns mavlink.test
  (:require [clojure.java.io :as io]
            [clojure.core.async :as async]
            [mavlink.core :refer :all])
  (:import  [java.nio ByteBuffer ByteOrder]
            [java.io PipedInputStream PipedOutputStream]
            [com.MAVLink MAVLinkPacket Parser]
            [com.MAVLink.common msg_heartbeat]
            [com.MAVLink.enums MAV_AUTOPILOT MAV_STATE MAV_TYPE]))

(def mavlink-map
          (parse {:xml-sources [{:xml-file "ardupilotmega.xml"
                                 :xml-source (-> "test/resources/ardupilotmega.xml" io/input-stream)}
                                {:xml-file "common.xml"
                                 :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                {:xml-file "uAvionix.xml"
                                 :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}
                               ]
                    }))

(defn mk-bytes
  [^String s]
  (into-array Byte/TYPE (mapv #(.byteValue (Long/valueOf % 16)) (clojure.string/split s #" "))))

(defn mk-pipe
  []
  (let [pipe-in (PipedInputStream.)
        pipe-out (PipedOutputStream. pipe-in)]
    {:pipe-in pipe-in
     :pipe-out pipe-out}))

(defonce decode-input-pipe (mk-pipe))
(defonce decode-output-channel (async/chan 300))
(defonce encode-input-channel  (async/chan 300))
(defonce encode-output-channel (async/chan 300))

(defn decode-bytearray
  [^bytes the-array]
  (.write ^PipedOutputStream (:pipe-out decode-input-pipe) the-array 0 (count the-array)))

(defn decode-bytes-from-string
  [String s]
  (decode-bytearray (mk-bytes s)))

(defn encode-message
  [message]
  (async/go (async/>! encode-input-channel message))
  nil)

(defn get-channel
  "Returns result of opening the channel, a map with :close-fn and :statistics bindings."
  [mavlink]
  ; start decode output thread
  (async/thread
    (loop [message (async/<!! decode-output-channel)]
      (when message
        (println "\nDecoded message:\n" message)
        (recur (async/<!! decode-output-channel)))))

  ; start encode output thread
  (async/thread
    (loop [message-bytes (async/<!! encode-output-channel)]
      (decode-bytearray message-bytes)
      (when message-bytes
        (recur (async/<!! encode-output-channel)))))

  (open-channel mavlink {:protocol :mavlink1
                         :system-id 99
                         :component-id 88
                         :link-id 77
                         :decode-input-stream (:pipe-in decode-input-pipe)
                         :decode-output-channel decode-output-channel
                         :encode-input-channel encode-input-channel
                         :encode-output-link encode-output-channel
                         :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                         :accept-message-handler #(do
                                                    (println "clj-mavlink/accept-message:\n" %1)
                                                    true)
                         :signing-options nil}))
 
(def channel (get-channel mavlink-map))

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

