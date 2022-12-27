(ns mavlink.mavlink1-tlog
  (:require [clojure.test :refer :all]
            [clojure.java.io :as io]
            [clojure.core.async :as async]
            [mavlink.core :refer :all]
            [mavlink.type :refer :all]
            [mavlink.test_utilities :refer :all]))

(def last-error (atom nil))
(def mavlink1-tlog "MAVLINK1.tlog")
(def mavlink1-input-tlog "MAVLINK1-input.tlog")

(def msg-total (atom 0)) ; total number of messages round tripped, used to verify reading tlog input stream


; The tlog test creates a tlog and then uses that tlog as an input stream to generate another tlog.
; Because the opening and closing of tlogs happens within one test, this test file does not use the wrap
; around function to do the test, (the same utilitiey functions are used to open and close the test mavlink connections.
;
(deftest mavlink-1-0-tlog
  (testing "Message tlog streams."
    ; Open  a mavlink1 connection and roundtrip all the messages saving the tlog-stream to mavlink1-tlog.
    ;
    ; Verify none of the roundtripsshould cause an error.
    ;
    ; Close the mavlink connection and mavlink1-tlog.
    ;
    (let [tlog-stream (io/output-stream mavlink1-tlog)
          mavlink-connection (open-mavlink-connection
                               {:xml-sources [{:xml-file "ardupilotmega.xml"
                                               :xml-source (-> "test/resources/ardupilotmega.xml" io/input-stream)}
                                              {:xml-file "common.xml"
                                               :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                              {:xml-file "uAvionix.xml"
                                               :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                                :retain-fields? true}
                               {:protocol :mavlink1
                                :system-id 99
                                :component-id 88
                                :link-id 77
                                :report-error #(do (println ">>>>>>>>>>>>>>> ERROR " %1) (reset! last-error %1))
                                :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                                :signing-options {:accept-message-handler
                                                   #(do
                                                     (println "clj-mavlink/accept-message:\n" %1)
                                                     true)}
                                :tlog-stream tlog-stream})
          {:keys [mavlink compare-messages encode-roundtrip get-test-message]} mavlink-connection]
      (reset! last-error nil)
      (doseq [id (range 255)]
        (when-let [msg-info (get (:messages-by-id mavlink) id)]
          (let [message (get-test-message msg-info)
                decoded-message (encode-roundtrip message)]
            (swap! msg-total inc)
            (is (compare-messages message decoded-message)))))
      (is (nil? @last-error)
          "None of the round trips should cause a failure.")

      (close-mavlink-connection mavlink-connection)
      (.close tlog-stream))
    
    ; now open a mavlink connection using mavlink1-tlog (generated above) as an input stream.
    ; And save the output tlog stream.  Then decode all the messages and count how many messages
    ; were decoded (from mavlink1-tlog).
    ;
    ; Verify the number of messages is identical to the number of messages roundtripped above.
    ; and that no errors occurred.
    ;
    ; Close the mavlink connection and tlog stream.
    ;
    (with-open [decode-tlog-stream (io/input-stream mavlink1-tlog)]
      (let [tlog-stream (io/output-stream mavlink1-input-tlog)
            decode-output-channel(async/chan 300)
            mavlink-connection (open-mavlink-connection
                                 {:xml-sources [{:xml-file "ardupilotmega.xml"
                                                 :xml-source (-> "test/resources/ardupilotmega.xml" io/input-stream)}
                                                {:xml-file "common.xml"
                                                 :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                                {:xml-file "uAvionix.xml"
                                                 :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                                  :retain-fields? true}
                                 {:protocol :mavlink1
                                  :system-id 99
                                  :component-id 88
                                  :link-id 77
                                  :report-error #(reset! last-error %1)
                                  :input-is-tlog? true
                                  :decode-input-stream decode-tlog-stream
                                  :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                                  :signing-options {:accept-message-handler
                                                       #(do
                                                         (println "clj-mavlink/accept-message:\n" %1)
                                                         true)}
                                  :decode-output-channel decode-output-channel
                                  :tlog-stream tlog-stream})
            msg-count (loop [num-msgs 0]
                        ; decode and count all the messages
                        (let [[message _] (async/alts!! [decode-output-channel (async/timeout 1000)])]
                          (if message
                            (recur (inc num-msgs))
                            num-msgs)))]
        ; number of processed messages is 2 * number of messages (one encode and one decode) + 1 for the SigningTuples
        (is (= (inc (* 2 @msg-total)) msg-count)
            (str "Failed to read all of messages from tlog. Total written to tlog :" @msg-total
                    "Total read from tlog: " msg-count))
        (is (nil? @last-error)
            "None of the tlog messages should cause a failure.") 

        (close-mavlink-connection mavlink-connection)
        (.close tlog-stream)

        (is (= (.length (io/file mavlink1-tlog)) (.length (io/file mavlink1-input-tlog)))
            "mavlink processing of tlog did not produce identical sized tlog")))

    ; Verify via a byte by byte comparison to see if the input tlog is identical to the output tlog.
    ;
    ; mavlink1-tlog, the input tlog, is the tlog created by roundtripping messages.
    ; mavlink1-input-tlog, the output tlog, is the tlog using mavlink1-tlog as input
    ;
    (with-open [tlog-stream (io/input-stream mavlink1-tlog)
                tlog-input-stream (io/input-stream mavlink1-input-tlog)]
      (let [mavlink1-tlog-size (.length (io/file mavlink1-tlog))]
        (loop [n 0]
          (let [c1 (.read tlog-stream)
                c2 (.read tlog-input-stream)]
            (is (= c1 c2)
              (format "Byte %d; %c from input tlog does not equal %c from output tlog" n c1 c2))
            (when (and (< (inc n) mavlink1-tlog-size)
                       (= c1 c2))
              (recur (inc n)))))))))
