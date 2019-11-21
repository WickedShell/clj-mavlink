(ns mavlink.mavlink2-signing-tlog
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

(def last-error (atom nil))

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
    (.flush ^PipedOutputStream (:pipe-out decode-input-pipe))
    (async/<!! decode-output-channel)))

(def mavlink (parse {:xml-sources [{:xml-file "ardupilotmega.xml"
                                      :xml-source (-> "test/resources/ardupilotmega.xml" io/input-stream)}
                                     {:xml-file "common.xml"
                                      :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                     {:xml-file "uAvionix.xml"
                                      :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                     :retain-fields? true}))

(def tlog-stream (io/output-stream "MAVLINK2.tlog"))

(def channel  (open-channel mavlink {:protocol :mavlink1
                                     :system-id 99
                                     :component-id 88
                                     :link-id 77
                                     :decode-input-stream (:pipe-in decode-input-pipe)
                                     :decode-output-channel decode-output-channel
                                     :encode-input-channel encode-input-channel
                                     :encode-output-link encode-output-channel
                                     :report-error #(reset! last-error %1)
                                     :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                                     :signing-options {:secret-key (get secret-keyset 0)
                                                       :secret-keyset secret-keyset
                                                       :accept-message-handler
                                                              #(do
                                                                (println "clj-mavlink/accept-message:\n" %1)
                                                                true)
                                                       }
                                     :tlog-stream tlog-stream}))

(defn get-test-message 
  "Given a message's specification map, generate a test message-map for it."
  [{:keys [msg-key fields] :as message-spec}]
  {:pre [msg-key
         (not (empty? fields))]}
  (merge {:message'id msg-key}
         (apply merge (map #(let [{:keys [name-key type-key enum-type length bitmask]} %
                                  enum-group (when enum-type
                                               (enum-type (:enums-by-group mavlink)))]
                              {name-key (if (and bitmask enum-type)
                                          (get-test-bitmask enum-group)
                                          (let [value (get-test-value type-key  5 length)]
                                            (if enum-type
                                              (get enum-group value value)
                                              value)))})
                           fields))))

(deftest mavlink-2-0-signing
  (testing "Message round trips MAVlink 1.0."
    (doseq [id (range 255)]
      (when-let [msg-info (get (:messages-by-id mavlink) id)]
        (let [message (get-test-message msg-info)
              decoded-message (encode-roundtrip message)]
          ; (println (str "-- Testing message " (:message'id message))) ; " :: " message))
          (is (compare-messages mavlink message decoded-message)
            (str "Roundtrip failed.\n Sent msg: " message
                 "\nReceived message: "decoded-message))))))
  (testing "automatic protocol change from MAVlink 1 to MAVlink 2"
    (let [statistics (:statistics channel)]
      (encode-oneway {:message'id :device-op-read})
      (Thread/sleep 1) ; give the encode a thread an opportunity to run
      (is (== 1 (:bad-protocol @statistics))
          "MAVlink 2 only message should fail to encode due to bad protocol")
      (let [decoded-message (encode-roundtrip {:message'id :heartbeat :mavlink'protocol :mavlink2})]
        (is decoded-message
            "Failed to send MAVlink 2 heartbeat"))
      (let [decoded-message (encode-roundtrip {:message'id :device-op-read})]
        (is decoded-message
          "Failed to send MAVlink 2 only test message"))
      (is (== 1 (:bad-protocol @(:statistics channel)))
          "Second attempt to send MAVlink 2 only message should pass.")
      )
    )
  (testing "Roundtrip of all messages."
    (println (str "There are " (count (vals (:messages-by-id mavlink))) " messages types."))
    (reset! last-error nil)
    (doseq [msg-info (vals (:messages-by-id mavlink))]
        (let [message (get-test-message msg-info)
              decoded-message (encode-roundtrip message)]
          ; (println (str "-- Testing message " (:message'id message) " :: " message))
          (is (compare-messages mavlink message decoded-message)
            (str "Roundtrip failed.\n Sent msg: " message
                 "\nReceived message: "decoded-message)))))

    (is (nil? @last-error)
        "None of the round trips should cause a failure.")

  (.close tlog-stream)
  (println "MAVLINK2.tlog closed!!!!!")
  )

