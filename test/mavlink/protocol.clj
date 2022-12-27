(ns mavlink.protocol
  (:require [clojure.test :refer :all]
            [clojure.java.io :as io]
            [mavlink.core :refer :all]
            [mavlink.type :refer :all]
            [mavlink.test_utilities :refer :all]))

(def last-error (atom nil))

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
                              :report-error #(reset! last-error %1)
                              :exception-handler #(println "clj-mavlink/test exception:\n" %1)
                              :signing-options {:accept-message-handler
                                                (fn [_] true)}})]
    (try
      (binding [*mavlink-connection* mavlink-connection]
        (work))
      (finally
        (close-mavlink-connection mavlink-connection)))))

(use-fixtures :once mavlink-setup)


(deftest mavlink-1
  (let [{:keys [mavlink compare-messages encode-roundtrip encode-oneway get-test-message]} *mavlink-connection*]
    (testing "Message mavlink1 round trips."
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

    (testing "clj-mavlink message swap protocol to mavlink2"
      (reset! last-error nil)
      ; Swap to mavlink2 and resend all the messages.
      (encode-oneway {:message'id :clj-mavlink
                      :protocol :mavlink2})
      (doseq [id (range 255)]
        (when-let [msg-info (get (:messages-by-id mavlink) id)]
          (let [message (get-test-message msg-info)
                decoded-message (encode-roundtrip message)]
            (is (= (:protocol' decoded-message) :mavlink2)
                (str "Message protocol error. Sent msg: " message
                     "\nReceived message: " decoded-message))
            (is (compare-messages message decoded-message)))))
      (is (nil? @last-error)
          "None of the round trips should cause a failure."))

    (testing "automatic protocol change from MAVlink 1 to MAVlink 2"
      (let [statistics (:statistics *mavlink-connection*)]
        (let [decoded-message (encode-roundtrip {:message'id :heartbeat :protocol' :mavlink2})]
          (is decoded-message
              "Failed to send MAVlink 2 heartbeat"))
        (let [decoded-message (encode-roundtrip {:message'id :device-op-read})]
          (is decoded-message
            "Failed to send MAVlink 2 only test message"))
        (let [decoded-message (encode-roundtrip {:message'id :heartbeat :protocol' :mavlink1})]
          (is decoded-message
              "Failed to accept MAVlink 1 heartbeat"))))

    (testing "Illegal swap back to mavlink1"
      ; Swap to mavlink2 and resend all the messages.
      (reset! last-error nil)
      (encode-oneway {:message'id :clj-mavlink
                      :protocol :mavlink1})
      (Thread/sleep 500) ; give the encode a chance to run
      (is (= :bad-protocol (:cause (ex-data @last-error)))
          "Encode should fail because cannot go from mavlink2 to mavlink1")
      (reset! last-error nil)
      (encode-oneway {:message'id :clj-mavlink
                      :protocol :unknown-mavlink})
      (Thread/sleep 500) ; give the encode a chance to run
      (is (= :bad-protocol (:cause (ex-data @last-error)))
          "Encode should fail because unknown protocol"))))
