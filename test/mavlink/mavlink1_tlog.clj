(ns mavlink.mavlink1-tlog
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
(def mavlink1-tlog "MAVLINK1.tlog")
(def mavlink1-input-tlog "MAVLINK1-input.tlog")

(def msg-total (atom 0)) ; total number of messages round tripped, used to verify reading tlog input stream

; These are used by the round trip testing.
(def decode-input-pipe (mk-pipe))
(def decode-output-channel (async/chan 300))
(def encode-input-channel  (async/chan 300))
(def encode-output-channel (async/chan 300))

(defn encode-oneway
  "Given a message map, encode it. But don't poll for the result because
   an error is expected."
  [message]
  ; (println "ENCODING oneway :") (pprint message)
  (async/>!! encode-input-channel message))

(defn encode-roundtrip
  "Given a message map, encode it then get the decoded and round robin it back to the encode
   and compare the result."
  [message]
  (async/>!! encode-input-channel message)
  (when-let [message-bytes (async/<!! encode-output-channel)]
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

(deftest mavlink-1-0-tlog
  (testing "Message round trips."
    (let [tlog-stream (io/output-stream mavlink1-tlog)
          channel  (open-channel mavlink {:protocol :mavlink1
                                          :system-id 99
                                          :component-id 88
                                          :link-id 77
                                          :report-error #(reset! last-error %1)
                                          :decode-input-stream (:pipe-in decode-input-pipe)
                                          :decode-output-channel decode-output-channel
                                          :encode-input-channel encode-input-channel
                                          :encode-output-link encode-output-channel
                                          :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                                          :signing-options {:accept-message-handler
                                                                   #(do
                                                                     (println "clj-mavlink/accept-message:\n" %1)
                                                                     true)
                                                            }
                                          :tlog-stream tlog-stream})]

        (reset! last-error nil)
        (doseq [id (range 255)]
          (when-let [msg-info (get (:messages-by-id mavlink) id)]
            (let [message (get-test-message msg-info)
                  decoded-message (encode-roundtrip message)]
              (swap! msg-total inc)
              (is (compare-messages mavlink message decoded-message)
                (str "Roundtrip failed.\n Sent msg: " message
                     "\nReceived message: "decoded-message)))))
        (is (nil? @last-error)
            "None of the round trips should cause a failure.")

        ; close the channel and the tlog-stream
        ((:close-channel-fn channel))
        (.close tlog-stream))
    
      ; now read the messages back out of the tlog
      (with-open [decode-tlog-stream (io/input-stream mavlink1-tlog)]
        (let [initial-tlog-size (.length (io/file mavlink1-tlog)) 
              decode-output-channel2 (async/chan 300)
              encode-input-channel2  (async/chan 300)
              encode-output-channel2 (async/chan 300)
              tlog-stream (io/output-stream mavlink1-input-tlog)
              channel  (open-channel mavlink {:protocol :mavlink1
                                              :system-id 99
                                              :component-id 88
                                              :link-id 77
                                              :report-error #(reset! last-error %1)
                                              :input-is-tlog? true
                                              :decode-input-stream decode-tlog-stream
                                              :decode-output-channel decode-output-channel2
                                              :encode-input-channel encode-input-channel2
                                              :encode-output-link encode-output-channel2
                                              :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                                              :signing-options {:accept-message-handler
                                                                       #(do
                                                                         (println "clj-mavlink/accept-message:\n" %1)
                                                                         true)
                                                                }
                                              :tlog-stream tlog-stream})]
          (let [msg-count (loop [num-msgs 0]
                            (let [[message _] (async/alts!! [decode-output-channel2 (async/timeout 1000)])]
                              (if message
                                (recur (inc num-msgs))
                                num-msgs)))]
            ; number of processed messages is 2 * number of messages (one encode and one decode) + 1 for the SigningTuples
            (is (= (inc (* 2 @msg-total)) msg-count)
                (str "Failed to read all of messages from tlog. Total written to tlog :" @msg-total
                         "Total read from tlog: " msg-count))
            (is (nil? @last-error)
                "None of the tlog messages should cause a failure."))

          ((:close-channel-fn channel))
          (.close tlog-stream)

          (is (= initial-tlog-size (.length (io/file mavlink1-input-tlog)))
              "mavlink processing of tlog did not produce identical sized tlog")

          (with-open [tlog-stream (io/input-stream mavlink1-tlog)
                      tlog-input-stream (io/input-stream mavlink1-input-tlog)]
            (loop [n 0]
              (let [c1 (.read tlog-stream)
                    c2 (.read tlog-input-stream)]
                (is (= c1 c2)
                  (format "Byte %d; %c from input tlog does not equal %c from output tlog" n c1 c2))
                (when (and (< (inc n) initial-tlog-size)
                           (= c1 c2))
                  (recur (inc n))))))

          ))))
