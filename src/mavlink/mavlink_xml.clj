(ns mavlink.mavlink-xml
  (:require [clojure.data.xml :as xml]
            [clojure.xml :as clj-xml]
            [clojure.zip :as zip]
            [clojure.data.zip.xml :as zip-xml]
            [clojure.string :as string]
            [mavlink.checksum :refer :all]
            [mavlink.type :refer :all])
  (:import [clojure.data.xml Element])
  (:gen-class))

(use 'clojure.pprint)

(def ^:const MAX-MESSAGE-SIZE 300)  ; the maximum message size

(defn report-error
  "Report errors, for now just print them. Any number of arguments that
   can be passed to str maybe given. Return nil so this can be used to report the
   error as well as return the error condition nil."
  [data & s]
  (throw (ex-info (str "ERROR: " (string/join " " s))
                  data)))

(defn- keywordize
  "Take a string, replace all _ with - and put it in lower case,
   then convert to a keyword and return it.
   Not intended for use on type-names"
  [^String s]
  (if s
    (keyword (string/lower-case (string/replace s \_ \-)))
    (report-error {:cause :null-pointer}
                  "keywordize: null pointer")))

(defn- get-value
  "Take a string, if it is nil, return it, otherwise convert the string to a long
   and return it. Return nil if it fails to convert. The idenitfier is
   just used to build the error message"
  [^String identifier ^String s]
  (when s
    (try
      (Long/valueOf s)
      (catch Exception e
        (report-error {:cause :string-not-number}
                      identifier ">" s "<" "failed to convert to long.")))))

(defn- get-type
  "Take a type string from the XML convert to the base type.
   That means strip off the array length indicator, if any
   and return the result as a keyword"
  [^String s]
  (keyword (string/replace s #"\[\d+\]" "")))

(defn- get-array-length
  "Take a type string from the XML and return array length.
   If the type is not an array, then return nil."
  [^String s]
  (when-let [array-spec (re-find (re-matcher #"\[\d+\]" s))]
    (when-let [length (re-find (re-matcher #"\d+" array-spec))]
      (get-value (str s " array type length") length))))

(defn lookup-enum
 "Given an enum group, lookup enum for the value int his group, return either
  the enum or the value if the enum could not be found."
  [enums v]
  (get enums v v))

(defn get-extension-fields
  "Returns just the extension fields of a messages; or nil if there are none."
  [m]
  (when-let [node (zip-xml/xml1-> m :extensions)]
    (when-let [ext-fields (zip/rights node)]
      (vec
        (for [f ext-fields]
          (let [fld-name (-> f :attrs :name)
                type-name (-> f :attrs :type)
                enum-name (-> f :attrs :enum)]
            {:fld-name fld-name
             :name-key (keyword fld-name)
             :type-key (get-type type-name)
             :enum-type (when enum-name
                          (keywordize enum-name))
             :length (get-array-length type-name)
             }))))))

(defn sort-fields
  "Take a vector of fields and return them in a vector sorted based on
   the order to place them in message."
  [fields]
  (vec (sort #(let [priority1 (get-type-priority (:type-key %1))
                    priority2 (get-type-priority (:type-key %2))]
                  (< priority1 priority2))
             fields)))

(defn get-mavlink
  "Return a mavlink map for one xml source."
  [{:keys [system-id component-id descriptions mavlink-version] :as options} ^String file-name zipper]
 (let [enum-to-value
        (with-local-vars [last-value 0]
          (apply merge (zip-xml/xml-> zipper :mavlink :enums :enum :entry
                                      (fn[e]
                                        (let [enum-name  (zip-xml/attr e :name)
                                              value (get-value (str "In " file-name " " enum-name " value")
                                                       (zip-xml/attr e :value))]
                                          { (keywordize enum-name)
                                            (if value
                                              (var-set last-value value)
                                              (var-set last-value (inc @last-value)))})))))
       enums-by-group
        (apply merge (zip-xml/xml-> zipper :mavlink :enums :enum
                      (fn[e] 
                        { (keywordize (zip-xml/attr e :name))
                              (apply merge
                                     (zip-xml/xml-> e :entry
                                       #(let [enum-key (keywordize (zip-xml/attr % :name))]
                                          {(enum-key enum-to-value) enum-key})))
                             } )))
       messages-by-keyword
          (apply
            merge
            (zip-xml/xml->
              zipper :mavlink :messages :message
              (fn[m]
                (let [msg-name (-> m first :attrs :name)
                      msg-id (get-value (str "In " file-name " " msg-name " id")
                                            (-> m first :attrs :id))
                      all-fields (vec
                                   (zip-xml/xml-> m :field
                                                  (fn[f]
                                                    (let [fld-name (zip-xml/attr f :name)
                                                          type-name (zip-xml/attr f :type)
                                                          enum-name (zip-xml/attr f :enum)
                                                          type-key (get-type type-name)]
                                                      {:name-key (keyword fld-name)
                                                       :fld-name fld-name
                                                       :type-key type-key
                                                       :enum-type (when enum-name
                                                                    (keywordize enum-name))
                                                       :length (get-array-length type-name)}))))
                      ext-fields (get-extension-fields m)
                      sorted-fields (sort-fields
                                      (if (empty? ext-fields)
                                        all-fields
                                        (filterv (fn[m]
                                                   (empty?
                                                    (filter #(= (:name-key m)
                                                                (:name-key %)) ext-fields)))
                                                 all-fields)))
                      payload-size (apply + (map #(let [{:keys [type-key length]} %]
                                                    (* (type-key type-size) (or length 1)))
                                                 sorted-fields))
                      msg-magic-byte ; note that field names and their type names are used here
                        (let [msg-seed (str msg-name " "
                                            (apply str
                                                   (map #(let [{:keys [fld-name type-key length]} %]
                                                           (str (if (= type-key :uint8_t_mavlink_version)
                                                                  "uint8_t"
                                                                  (name type-key))  " "
                                                                fld-name " "
                                                                (when length
                                                                  (char length))))
                                                        sorted-fields)))
                              checksum (compute-checksum msg-seed)]
                          (bit-xor (bit-and checksum 0xFF)
                                   (bit-and (bit-shift-right checksum 8) 0xff)))
                      default-msg (apply
                                    merge
                                    (map #(let [{:keys [name-key type-key length]} %]
                                            (if length
                                              { name-key (get-default-array type-key length) }
                                              { name-key (get-default-value type-key mavlink-version) }))
                                         sorted-fields))
                      encode-fns (mapv
                                   #(let [{:keys [name-key type-key length]} %
                                          write-fn (type-key write-payload)]
                                      (if write-fn
                                        (if length
                                          (fn [payload message]
                                            (let [value (name-key message)
                                                  num-missing (- length (count value))]
                                              (doseq [fval value]
                                                (write-fn payload fval))
                                              (dotimes [i num-missing]
                                                (write-fn payload (get-default-value type-key mavlink-version)))))
                                          (fn [payload message]
                                            (write-fn payload (name-key message))))
                                       (report-error {:cause :no-write-fn}
                                                     (str "No function to write " type-key))))
                                   sorted-fields)

                      decode-fns 
                        (mapv #(let [{:keys [name-key type-key length enum-type]} %
                                     read-fn (type-key read-payload)
                                     enum-group (get enums-by-group enum-type) ]
                                 (if-not read-fn
                                   (report-error {:cause :no-read-fn}
                                                 "Unknown type" type-key
                                                 "for field" name-key
                                                 "in message" msg-name)
                                   (if length
                                     (fn [buffer]
                                       (let [new-array (reduce
                                                         (fn [vr i] (conj vr (let [v (read-fn buffer)]
                                                                               (if enum-type
                                                                                 (lookup-enum enum-group v)
                                                                                 v))))
                                                         []
                                                         (range length))]
                                         { name-key (if (= type-key :char)
                                                      (.trim (new String ^"[B" (into-array Byte/TYPE new-array)))
                                                      new-array) }))
                                     (fn [buffer]
                                       { name-key (let [v (read-fn buffer)]
                                                    (if enum-type
                                                      (lookup-enum enum-group v)
                                                      v)) } ))))
                              sorted-fields)]
                  { (keywordize msg-name)
                    {:msg-id msg-id
                     :default-msg default-msg
                     :last-value (ref default-msg)
                     :msg-key (keywordize msg-name)
                     :payload-size payload-size
                     :fields sorted-fields
                     :encode-fns encode-fns
                     :decode-fns decode-fns
                     :magic-byte msg-magic-byte
                     :extension-fields (sort-fields ext-fields)
                    }
                  }))))
       all-descriptions
         (if descriptions
           (apply
             merge
             (concat
               (zip-xml/xml-> zipper :mavlink :enums :enum
                            (fn[eg]
                              { (keywordize (zip-xml/attr eg :name))
                                (zip-xml/xml1-> eg :description zip-xml/text)
                              }))
               (zip-xml/xml-> zipper :mavlink :enums :enum :entry
                            (fn[e]
                              { (keywordize (zip-xml/attr e :name))
                                (zip-xml/xml1-> e :description zip-xml/text)
                              }))
               (zip-xml/xml-> zipper :mavlink :messages :message
                            (fn[m]
                              { (keywordize (zip-xml/attr m :name))
                                (zip-xml/xml1-> m :description zip-xml/text)
                              }))))
           nil)]
   {:descriptions all-descriptions
    :enum-to-value enum-to-value
    :enums-by-group enums-by-group
    :messages-by-keyword messages-by-keyword
    :messages-by-id (apply merge (map #(hash-map (:msg-id %) %)
                                    (vals messages-by-keyword)))}))

(defn add-mavlink
  "Given two mavlink maps, merge the contents of the second with the first.
   new-file is the name of the file used for errors."
  [{:keys [descriptions enum-to-value enums-by-group
           messages-by-keyword messages-by-id] :as mavlink}
   new-part new-file]
  (let [conflicts (filterv #(% enum-to-value) (keys (:enum-to-value new-part)))]
    (when-not (empty? conflicts)
      (report-error {:cause :enum-conflicts}
                    "Adding" new-file
                    "there are conflicts with the following enums"
                    conflicts
                    "The new values will override the existing values.")))
  (let [conflicts (filterv #(get messages-by-id %) (keys (:messages-by-id new-part)))]
    (when-not (empty? conflicts)
      (report-error {:cause :message-id-conflicts}
                    "Adding" new-file
                    "there are conflicts with the following message ids:"
                    conflicts
                    "The new values will override the existing values.")))
  (let [conflicts (filterv #(% messages-by-keyword) (keys (:messages-by-keyword new-part)))]
    (when-not (empty? conflicts)
      (report-error {:cause :message-name-conflicts}
                    "Adding" new-file
                    "there are conflicts with the following message names:"
                    conflicts
                    "The new values will override the existing values.")))
  {:descriptions (merge descriptions (:descriptions new-part))
   :enum-to-value (merge enum-to-value (:enum-to-value new-part))
   :enums-by-group (merge enums-by-group (:enums-by-group new-part))
   :messages-by-keyword (merge messages-by-keyword
                               (:messages-by-keyword new-part))
   :messages-by-id (merge messages-by-id (:messages-by-id new-part))})

(defn get-xml-zippers
  "Open all the xml sources, verify that each source' includes are in the list; return
   new xml-sources vector with a zipper binding for each source."
  [xml-sources]
  (letfn [(verify-includes [zipper]
            (let [includes (zip-xml/xml-> zipper :mavlink :include zip-xml/text)]
              (doseq [file includes]
                (let [included (some #(= file (:xml-file %)) xml-sources)]
                  (when-not included
                    (report-error {:cause :missing-xml-include}
                                  file "is included but not listed in :xml-sources" xml-sources))))))]
    (let [sources-with-zippers
            (mapv #(let [{:keys [xml-file xml-source]} %
                         zipper (-> (:xml-source %) xml/parse zip/xml-zip)
                         file-name (or (zip-xml/xml1-> zipper :mavlink (zip-xml/attr :file))
                                       xml-file)]
                     (when-not file-name
                       (report-error {:cause :missing-xml-file-id}
                                     "no file= attribute in XML and no :xml-file in" xml-source))
                     (assoc % :xml-zipper zipper
                              :xml-file file-name))
                  xml-sources)]
      (doseq [{:keys [xml-zipper]} sources-with-zippers]
        (verify-includes xml-zipper))
      sources-with-zippers)))

