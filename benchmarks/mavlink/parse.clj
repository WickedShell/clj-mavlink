(ns mavlink.parse
  (:require [clojure.java.io :as io]
            [mavlink.core :as mavlink])
  (:use perforate.core))

(defgoal parse-goal "Load MAVLink XML's and create a parser")

(defcase parse-goal :one-file
  []
  (mavlink/parse {:xml-sources [{:xml-file "common.xml"
                                 :xml-source (-> "test/resources/common.xml" io/input-stream)}]
                  :descriptions true}))

(defcase parse-goal :one-file-no-description
  []
  (mavlink/parse {:xml-sources [{:xml-file "common.xml"
                                 :xml-source (-> "test/resources/common.xml" io/input-stream)}]
                  :descriptions true}))

(defcase parse-goal :multiple-files
  []
  (mavlink/parse {:xml-sources [{:xml-file "test-include.xml"
                                 :xml-source (-> "test/resources/test-include.xml" io/input-stream)}
                                {:xml-file "common.xml"
                                 :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                {:xml-file "uAvionix.xml"
                                 :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}
                                ]
                  :descriptions true}))
