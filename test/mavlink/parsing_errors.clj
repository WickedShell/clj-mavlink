(ns mavlink.parsing-errors
  (:require [clojure.test :refer :all]
            [clojure.java.io :as io]
            [mavlink.core :refer :all]))

; This file tests parsing errors.
;

(deftest parsing
  (testing "Testing bad.xml"
    (is (thrown-with-msg? Exception #"Missing a required include file" 
      (let [mavlink
            (parse {:mavlink-1-0 "not used"
                    :xml-sources [{:xml-file "bad.xml"
                                   :xml-source (-> "test/resources/bad.xml" io/input-stream)}
                                  {:xml-file "common.xml"
                                   :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                  {:xml-file "uAvionix.xml"
                                   :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                    :system-id 99
                    :component-id 88
                    :descriptions true})]))))
  (testing "Testing bad2.xml"
    (is (thrown-with-msg? Exception #"Message ID's conflict"
      (let [mavlink
            (parse {:mavlink-1-0 "not used"
                    :xml-sources [{:xml-file "bad2.xml"
                                   :xml-source (-> "test/resources/bad2.xml" io/input-stream)}
                                  {:xml-file "common.xml"
                                   :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                  {:xml-file "uAvionix.xml"
                                   :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                    :system-id 99
                    :component-id 88
                    :descriptions true})]))))
  (testing "Testing bad3.xml"
    (is (thrown-with-msg? Exception #"Message names conflict"
      (let [mavlink
            (parse {:mavlink-1-0 "not used"
                    :xml-sources [{:xml-file "bad3.xml"
                                   :xml-source (-> "test/resources/bad3.xml" io/input-stream)}
                                  {:xml-file "common.xml"
                                   :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                  {:xml-file "uAvionix.xml"
                                   :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                    :system-id 99
                    :component-id 88
                    :descriptions true})])))
    (is (thrown-with-msg? Exception #"Unable to determine the XML file name"
      (let [mavlink
            (parse {:mavlink-1-0 "not used"
                    :xml-sources [{:xml-source (-> "test/resources/bad3.xml" io/input-stream)}
                                  {:xml-file "common.xml"
                                   :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                  {:xml-file "uAvionix.xml"
                                   :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                    :system-id 99
                    :component-id 88
                    :descriptions true})]))))
  (testing "Testing bad4.xml"
    (is (thrown-with-msg? Exception #"Unable to parse a message id"
      (let [mavlink
            (parse {:mavlink-1-0 "not used"
                    :xml-sources [{:xml-file "bad4.xml"
                                   :xml-source (-> "test/resources/bad4.xml" io/input-stream)}
                                  {:xml-file "common.xml"
                                   :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                  {:xml-file "uAvionix.xml"
                                   :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                    :system-id 99
                    :component-id 88
                    :descriptions true})]))))
  (testing "Testing bad5.xml"
    (is (thrown-with-msg? Exception #"Unknown [a-z]+ type"
      (let [mavlink
            (parse {:mavlink-1-0 "not used"
                    :xml-sources [{:xml-file "bad5.xml"
                                   :xml-source (-> "test/resources/bad5.xml" io/input-stream)}
                                  {:xml-file "common.xml"
                                   :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                  {:xml-file "uAvionix.xml"
                                   :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                    :system-id 99
                    :component-id 88
                    :descriptions true})]))))
  (testing "Testing bad6.xml"
    (is true ; (thrown-with-msg? Exception #"Unknown [a-z]+ type"
      (let [mavlink
            (parse {:mavlink-1-0 "not used"
                    :xml-sources [{:xml-file "bad6.xml"
                                   :xml-source (-> "test/resources/bad6.xml" io/input-stream)}
                                  {:xml-file "common.xml"
                                   :xml-source (-> "test/resources/common.xml" io/input-stream)}
                                  {:xml-file "uAvionix.xml"
                                   :xml-source (-> "test/resources/uAvionix.xml" io/input-stream)}]
                    :system-id 99
                    :component-id 88
                    :descriptions true})]))))
  ;)
