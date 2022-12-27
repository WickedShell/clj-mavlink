(ns mavlink.mavlink2-signing
  (:require [clojure.test :refer :all]
            [clojure.java.io :as io]
            [mavlink.checksum :refer :all]
            [mavlink.core :refer :all]
            [mavlink.type :refer :all]
            [mavlink.test_utilities :refer :all]))

(use 'clojure.pprint)

(def last-error (atom nil))

(def secret-keyset [(bytes (byte-array (map (comp byte int) "000000abcdefghijklmnopqrstuvwxyz")))
                    (bytes (byte-array (map (comp byte int) "abcdefghijklmnopqrstuvwxyz123456")))])

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
                              :signing-options {:secret-key (get secret-keyset 0)
                                                :secret-keyset secret-keyset
                                                :accept-message-handler
                                                       #(do
                                                         (println "clj-mavlink/accept-message:\n" %1)
                                                         true)}})]
    (try
      (binding [*mavlink-connection* mavlink-connection]
        (work))
      (finally
        (close-mavlink-connection mavlink-connection)))))

(use-fixtures :once mavlink-setup)

(deftest mavlink-2-0-signing
  (let [{:keys [mavlink compare-messages encode-roundtrip encode-oneway get-test-message]} *mavlink-connection*]
    (testing "Message round trips MAVlink 1.0."
      (doseq [id (range 255)]
        (when-let [msg-info (get (:messages-by-id mavlink) id)]
          (let [message (get-test-message msg-info)
                decoded-message (encode-roundtrip message)]
            (is (= (:protocol' decoded-message) :mavlink1)
                (str "Message protocol error. Sent msg: " message
                     "\nReceived message: " decoded-message))
            (is (compare-messages message decoded-message))))))

    (testing "automatic protocol change from MAVlink 1 to MAVlink 2"
      (let [statistics (:statistics *mavlink-connection*)]
        (encode-oneway {:message'id :device-op-read})
        (Thread/sleep 1) ; give the encode a thread an opportunity to run
        (is (== 1 (:bad-protocol @statistics))
            "MAVlink 2 only message should fail to encode due to bad protocol")
        (let [decoded-message (encode-roundtrip {:message'id :heartbeat :protocol' :mavlink2})]
          (is decoded-message
              "Failed to send MAVlink 2 heartbeat"))
        (let [decoded-message (encode-roundtrip {:message'id :device-op-read})]
          (is decoded-message
            "Failed to send MAVlink 2 only test message"))
        (is (== 1 (:bad-protocol @(:statistics *mavlink-connection*)))
            "Second attempt to send MAVlink 2 only message should pass.")))

    (testing "Roundtrip of all messages."
      (reset! last-error nil)
      (doseq [msg-info (vals (:messages-by-id mavlink))]
          (let [message (get-test-message msg-info)
                decoded-message (encode-roundtrip message)]
            (is (compare-messages message decoded-message)))))

      (is (nil? @last-error)
          "None of the round trips should cause a failure.")))

