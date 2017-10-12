(ns mavlink.mavlink1
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

(def decode-input-pipe (mk-pipe))
(def decode-output-channel (async/chan 300))
(def encode-input-channel  (async/chan 300))
(def encode-output-channel (async/chan 300))

(defn encode-oneway
  "Given a message map, encode it. But don't poll for the result because
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
         (apply merge (map #(let [{:keys [name-key type-key enum-type length]} %
                                  value (get-test-value type-key  5 length)]
                              {name-key (if enum-type
                                          (get (enum-type (:enums-by-group mavlink))
                                               value value)
                                          value)})
                           fields))))

(deftest mavlink-1-0
  (testing "Message round trips."
    (doseq [id (range 255)]
      (when-let [msg-info (get (:messages-by-id mavlink) id)]
        (let [message (get-test-message msg-info)
              decoded-message (encode-roundtrip message)]
          ;(println (str "-- Testing message " (:message-id message))) ; " :: " message))
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
      ))
  )

(deftest message-encode-errors
  (testing "Message encoding erros."
    (let [statistics (:statistics channel)]
      (let [message {:message-id :bad-message-id}
            decoded-message (encode-oneway message)]
        (is (== (:encode-failed @statistics) 1)
            "Encode should fail due to bad message id"))
      (let [message {:message-id :heartbeat :non-existent-field "NOPE"}
            decoded-message (encode-roundtrip message)]
        (is (nil? (:non-existent-field decoded-message))
            "Encode should fail bad field."))
      (let [message {:message-id :heartbeat :type :bad-enum}
            decoded-message (encode-oneway message)]
        (is (= message decoded-message)
            "Encode should fail due to bad message id"))

      ; print the final statistics
      (println "\n\nMavlink Statistics")
      (pprint @statistics))
    ))
