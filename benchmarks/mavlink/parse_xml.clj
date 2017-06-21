(ns mavlink.parse-xml
  (:require [clojure.java.io :as io]
            [mavlink.core :as mavlink])
  (:use perforate.core))

(defgoal parse-xml "Load MAVLink XML's and create a parser")

(defcase parse-xml :one-file
  []
  (mavlink/parse {:xml-sources [{:xml-file "common.xml"
                                 :xml-source (-> "test/resources/common.xml" io/input-stream)}]
                  :descriptions true}))

(defcase parse-xml :one-file-no-description
  []
  (mavlink/parse {:xml-sources [{:xml-file "common.xml"
                                 :xml-source (-> "test/resources/common.xml" io/input-stream)}]
                  :descriptions false}))

(defcase parse-xml :multiple-files
  []
  (mavlink/parse {:xml-sources [{:xml-file "test-include.xml"
                                 :xml-source (-> "test/resources/test-include.xml" io/input-stream)}
                                {:xml-file "common.xml"
                                 :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                {:xml-file "uAvionix.xml"
                                 :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}
                                ]
                  :descriptions true}))

(defgoal decode-message "Decode an encoded message"
  :setup (fn [] (let [channel (mavlink/open-channel
                                (mavlink/parse {:xml-sources [{:xml-file "test-include.xml"
                                                               :xml-source (-> "test/resources/test-include.xml" io/input-stream)}
                                                              {:xml-file "common.xml"
                                                               :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                                              {:xml-file "uAvionix.xml"
                                                               :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]})
                                {:protocol :mavlink2
                                 :system-id 25
                                 :component-id 25})
                      message (mavlink/encode channel {:message-id :gps-raw-int
                                                       :time_usec 2602727
                                                       :fix_type 4
                                                       :lat 15617036
                                                       :lon 26125162
                                                       :alt 100
                                                       :eph 25
                                                       :epv 70
                                                       :vel 2000
                                                       :cog 26})]
                  [channel message])))

(defcase decode-message :decode
  [channel message]
  (mavlink/decode-bytes channel message))

(defgoal encode-message "Encode a message"
  :setup (fn [] (let [channel (mavlink/open-channel
                                (mavlink/parse {:xml-sources [{:xml-file "test-include.xml"
                                                               :xml-source (-> "test/resources/test-include.xml" io/input-stream)}
                                                              {:xml-file "common.xml"
                                                               :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                                              {:xml-file "uAvionix.xml"
                                                               :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]})
                                {:protocol :mavlink2
                                 :system-id 25
                                 :component-id 25})]
                  [channel])))

(defcase encode-message :encode-simple
  [channel]
  (mavlink/encode channel {:message-id :gps-raw-int
                           :time_usec 2602727
                           :fix_type 4
                           :lat 15617036
                           :lon 26125162
                           :alt 100
                           :eph 25
                           :epv 70
                           :vel 2000
                           :cog 26}))
