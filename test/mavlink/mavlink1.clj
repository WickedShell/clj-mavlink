(ns mavlink.mavlink1
  (:require [clojure.test :refer :all]
            [clojure.java.io :as io]
            [mavlink.core :refer :all]
            [mavlink.type :refer :all]
            [mavlink.test_utilities :refer :all]))

(use 'clojure.pprint)

(def last-error (atom nil))
(def print-report-error (atom false))

(def ^:dynamic *mavlink-connection* nil)

(defn mavlink-setup [work]
  (let [mavlink-connection (open-mavlink-connection
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
                                         :report-error #(do (when @print-report-error 
                                                              (println "Unexpected Error occurred " %1))
                                                            (reset! last-error %1))
                                         :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                                         :signing-options {:accept-message-handler
                                                                  #(do
                                                                    (println "clj-mavlink/accept-message:\n" %1)
                                                                    true)
                                                           }})]
    (try
      (binding [*mavlink-connection* mavlink-connection]
        (work))
      (finally
        (close-mavlink-connection mavlink-connection)))))

(use-fixtures :once mavlink-setup)

(deftest payload-strings
  (let [{:keys [mavlink compare-messages encode-roundtrip get-test-message]} *mavlink-connection*]
    ; empty array;
    ; array with first byte is a null;
    ; array with no null bytes;
    ; array with byte values -128-127
    ; array with unicode
    ; array with no null byte
    (reset! print-report-error true)
    (testing "String encoding"
      (let [msg-info (get (:messages-by-keyword mavlink) :param-value)
            message (get-test-message msg-info)
            empty-param-name (assoc message :param-id (new String ^"[B" (into-array Byte/TYPE [])))
            null-param-name (assoc message :param-id  (new String ^"[B" (into-array Byte/TYPE [\u0000])))
            short-param-name (assoc message :param-id  (new String ^"[B" (into-array Byte/TYPE [78 \u0000])))
            pound1-bytes [80 79 85 78 68 20 49 20 -62 -93 -62]
            pound1-param-name (assoc message :param-id  ^"[B" (into-array Byte/TYPE pound1-bytes))
            pound2-bytes [80 79 85 78 68 20 50 20 -62 -93 -62 \u0000 80 79 85 78 68]
            pound2-param-name (assoc message :param-id ^"[B"(into-array Byte/TYPE pound2-bytes))
            full-param-name (assoc message :param-id (new String ^"[B"(into-array Byte/TYPE [80 79 85 78 68 20 50 20 80 79 85 78 68 20 50 20])))
            toolarge-param-name (assoc message :param-id (new String ^"[B"(into-array Byte/TYPE [80 79 85 78 68 20 50 20 80 79 85 78 68 20 50 20 80])))
            ]

        (let [decoded-message (encode-roundtrip message)]
          (is (compare-messages message decoded-message)))

        (let [decoded-message (encode-roundtrip empty-param-name)]
          (is (compare-messages empty-param-name decoded-message)))

        (let [decoded-message (encode-roundtrip null-param-name)] ; note that "\0" goes down and "" comes back
          (is (= (:param-id decoded-message) "")))

;        (let [decoded-message (encode-roundtrip short-param-name)]
;          (is (= (:param-id decoded-message) "N")))

        (let [decoded-message (encode-roundtrip pound1-param-name)]
          (is (= (:param-id decoded-message) (first (.split (new String ^"[B" (into-array Byte/TYPE pound1-bytes) "UTF-8") "\u0000")))))

        (let [decoded-message (encode-roundtrip pound2-param-name)]
          (is (= (:param-id decoded-message) (first (.split (new String ^"[B" (into-array Byte/TYPE pound2-bytes) "UTF-8") "\u0000")))))

        (let [decoded-message (encode-roundtrip full-param-name)]
          (is (compare-messages full-param-name decoded-message)))

        (let [decoded-message (encode-roundtrip toolarge-param-name)]
          (is (compare-messages full-param-name decoded-message)))))))

(deftest mavlink-1-0
  (let [{:keys [mavlink compare-messages encode-roundtrip encode-oneway get-test-message]} *mavlink-connection*]
    (reset! print-report-error false)
    (testing "Message round trips."
      (reset! last-error nil)
      (doseq [id (range 255)]
        (when-let [msg-info (get (:messages-by-id mavlink) id)]
          (let [message (get-test-message msg-info)
                decoded-message (encode-roundtrip message)]
            (is (= (:protocol' decoded-message) :mavlink1)
                (str "Message protocol error. Sent msg: " message
                     "\nReceived message: " decoded-message))
            (is (compare-messages message decoded-message)))))
      (is (nil? @last-error)
          "None of the round trips should cause a failure."))

    (testing "automatic protocol change from MAVlink 1 to MAVlink 2"
      (let [statistics (:statistics *mavlink-connection*)]
        (encode-oneway {:message'id :device-op-read})
        (Thread/sleep 1) ; give the encode a thread an opportunity to run
        (is  (and (== 1 (:bad-protocol @statistics))
                 (= :bad-protocol (:cause (ex-data @last-error))))
            "MAVlink 2 only message should fail to encode due to bad protocol")
        (let [decoded-message (encode-roundtrip {:message'id :heartbeat :protocol' :mavlink2})]
          (is decoded-message
              "Failed to send MAVlink 2 heartbeat"))
        (let [decoded-message (encode-roundtrip {:message'id :device-op-read})]
          (is decoded-message
            "Failed to send MAVlink 2 only test message"))
        (is (== 1 (:bad-protocol @(:statistics *mavlink-connection*)))
            "Second attempt to send MAVlink 2 only message should pass.")
        ))))

(deftest message-encode-errors
  (let [{:keys [mavlink compare-messages encode-roundtrip encode-oneway]} *mavlink-connection*]
    (reset! print-report-error false)
    (testing "Message encoding errors."
      (let [statistics (:statistics *mavlink-connection*)]
        (let [message {:message'id :bad-message-id}
              decoded-message (encode-oneway message)]
          (Thread/sleep 500) ; give the encode a chance to run
          (is (= :invalid-message-id (:cause (ex-data @last-error)))
              "Encode should fail due to bad message id"))
        (let [message {:message'id :heartbeat :type :bad-enum}]
          (encode-oneway message)
          (Thread/sleep 500) ; give the encode a chance to run
          (is (= :invalid-enum (:cause (ex-data @last-error)))
              "Encode should fail due to bad message id"))
        (println "RUTH YOU NEED TO ADD tests for invalid bitmasks and valid bitmasks")
  ; FIXME cannot run this test because this does not currently cause a failure.
  ;      (let [message {:message'id :heartbeat :non-existent-field "NOPE"}
  ;            decoded-message (encode-roundtrip message)]
  ;        (is (= :encode-fn-failed (:cause (ex-data @last-error)))
  ;            "Encode should fail bad field."))
        ))))

