(ns mavlink.mavlink-xml
  (:require [clojure.data.xml :as xml]
            [clojure.zip :as zip]
            [clojure.data.zip.xml :as zip-xml]
            [clojure.string :as string]
            [mavlink.checksum :refer :all]
            [mavlink.type :refer :all]))

(defmacro keywordize
  "Take a string, replace all _ with - and put it in lower case,
   then convert to a keyword and return it.
   Not intended for use on type-names"
  [s]
  `(keyword (string/lower-case (string/replace ^String ~s \_ \-))))

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
      (try
        (Long/valueOf ^String length)
        (catch Exception e
          (throw (ex-info "Unable to parse array length"
                          {:cause :string-not-number
                           :type s
                           :input length})))))))

(defn get-fields
  "Returns just the fields of a message; or nil if there are none.
   While using extentions tag as a separator, this function looks to
   the left of the tag, if there is no extensions tag, then all the fields are
   returned."
  [m]
  (if-let [node (zip-xml/xml1-> m :extensions)]
    (remove nil? 
            (mapv #(when (= (:tag %) :field)
                     (let [{:keys [enum name type]} (:attrs % first)]
                       {:fld-name name
                        :name-key (keyword name)
                        :type-key (get-type type)
                        :enum-type (when enum
                                     (keywordize enum))
                        :length (get-array-length type) }))
                  (zip/lefts node)))
    (vec
      (zip-xml/xml-> m :field
        (fn [f]
          (let [{:keys [enum name type]} (-> f first :attrs)]
            {:fld-name name
             :name-key (keyword name)
             :type-key (get-type type)
             :enum-type (when enum
                          (keywordize enum))
             :length (get-array-length type) }))))))

(defn get-extension-fields
  "Returns just the extension fields of a message; or nil if there are none."
  [m]
  (when-let [node (zip-xml/xml1-> m :extensions)]
    (when-let [ext-fields (zip/rights node)]
      (vec
        (for [f ext-fields]
          (let [{:keys [enum name type]} (:attrs f)]
            {:fld-name name
             :name-key (keyword name)
             :type-key (get-type type)
             :enum-type (when enum
                          (keywordize enum))
             :length (get-array-length type)
             }))))))

(defn sort-fields
  "Take a vector of fields and return them in a vector sorted based on
   the order to place them in message."
  [fields]
  (when fields
    (vec (sort #(let [priority1 (get-type-priority (:type-key %1))
                      priority2 (get-type-priority (:type-key %2))]
                    (< priority1 priority2))
               fields))))

(defmacro translate-keyword
  "Translates a keyword to the appropriate enum value if needed"
  [mavlink value]
  `(if (keyword? ~value)
     (if-let [enum-val# (~value (:enum-to-value ~mavlink))]
       enum-val#
       (throw (ex-info "Unable to translate enum before encoding"
                       {:cause :undefined-enum
                        :enum ~value})))
     ~value))

(defn gen-encode-fn
  "Generate the encode function for a field. Because they have the same structure
   this function works on both regular fields and extension fields."
  [{:keys [name-key type-key length] :as field}]
  (let [write-fn (type-key write-payload)]
    (if write-fn
      (if length
        (fn encode-it-array [mavlink payload message]
          (let [value (name-key message)
                num-missing (- length (count value))]
            (doseq [fval value]
              (write-fn payload (translate-keyword mavlink fval)))
            (dotimes [i num-missing]
              (write-fn payload 0))))
        (fn encode-it [mavlink payload message]
          (write-fn payload (translate-keyword mavlink (get message name-key 0)))))
     (throw (ex-info (str "No function to write " type-key)
                     {:cause :no-write-fn})))))

(defn gen-decode-fn
  "Generate the decode function for a field. Because they have the same structure
   this function works on both regular fields and extension fields."
  [{:keys [name-key type-key length enum-type] :as field} enums-by-group]
  (let [read-fn (type-key read-payload)
        enum-group (get enums-by-group enum-type) ]
    (if-not read-fn
      (throw (ex-info "Unknown type for read"
                      {:cause :no-read-fn
                       :type-key type-key
                       :name-key name-key}))
      (if length
        (fn decode-it-array [buffer message]
          (let [new-array (reduce
                            (fn [vr i] (conj vr (let [v (read-fn buffer)]
                                                    (get enum-group v v))))
                            []
                            (range length))]
            (assoc! message  name-key (if (= type-key :char)
                                        (.trim (new String ^"[B" (into-array Byte/TYPE new-array)))
                                        new-array))))
        (fn decode-it [buffer message]
          (assoc! message name-key (let [v (read-fn buffer)]
                                     (get enum-group v v))))))))

(defn get-mavlink
  "Return a mavlink map for one xml source."
  [{:keys [^String file-name zipper] :as source} {:keys [descriptions] :as options}]
  (let [enum-to-value (reduce merge
                              (zip-xml/xml-> zipper
                                             :mavlink
                                             :enums
                                             :enum
                                             (fn[enum-group]
                                               ; returns a map of the enum entries for this group
                                               (let [entries (zip-xml/xml-> enum-group :entry)]
                                                 (loop [last-value -1
                                                        entry (first entries)
                                                        rest-entries (rest entries)
                                                        values-map (transient {})]
                                                   (if (nil? entry)
                                                     (persistent! values-map)
                                                     (let [enum-name (zip-xml/attr entry :name)
                                                           value-str (zip-xml/attr entry :value)
                                                           enum-value (if value-str
                                                                        (try
                                                                          (Long/valueOf ^String value-str)
                                                                          (catch Exception e
                                                                            (throw (ex-info "Unable to parse an enum value"
                                                                                            {:cause :string-not-number
                                                                                             :input value-str
                                                                                             :enum-name enum-name}))))
                                                                        (inc last-value))]
                                                       (recur (long enum-value)
                                                              (first rest-entries)
                                                              (rest rest-entries)
                                                              (assoc! values-map
                                                                      (keywordize enum-name)
                                                                      enum-value)))))))))
       enums-by-group
        (apply merge (zip-xml/xml-> zipper :mavlink :enums :enum
                      (fn [e] 
                        {(keywordize (zip-xml/attr e :name))
                         (apply merge
                                (zip-xml/xml-> e :entry
                                               #(let [enum-key (keywordize (zip-xml/attr % :name))]
                                                  {(enum-key enum-to-value) enum-key})))})))
       messages-by-keyword
          (apply
            merge
            (zip-xml/xml->
              zipper :mavlink :messages :message
              (fn[m]
                (let [msg-name (-> m first :attrs :name)
                      msg-id-str (-> m first :attrs :id)
                      msg-id (try
                               (Long/valueOf ^String msg-id-str)
                               (catch Exception e
                                 (throw (ex-info "Unable to parse a message id"
                                                 {:cause :string-not-number
                                                  :input msg-id-str
                                                  :message-name msg-name}))))
                      fields (sort-fields (get-fields m))
                      ext-fields (sort-fields (get-extension-fields m))
                      payload-size (apply + (map #(let [{:keys [type-key length]} %]
                                                    (* (type-key type-size) (or length 1)))
                                                 fields))
                      payload-size-ext (apply + (map #(let [{:keys [type-key length]} %]
                                                    (* (type-key type-size) (or length 1)))
                                                 ext-fields))
                      crc-seed ; note that field names and their type names are used here
                        (let [msg-seed (str msg-name " "
                                            (apply str
                                                   (map #(let [{:keys [fld-name type-key length]} %]
                                                           (str (if (= type-key :uint8_t_mavlink_version)
                                                                  "uint8_t"
                                                                  (name type-key)) " "
                                                                fld-name " "
                                                                (when length
                                                                  (char length))))
                                                        fields)))
                              checksum (compute-checksum msg-seed)]
                          (bit-xor (bit-and checksum 0xFF)
                                   (bit-and (bit-shift-right checksum 8) 0xff)))
                      encode-fns (mapv gen-encode-fn fields)
                      decode-fns (mapv #(gen-decode-fn % enums-by-group) fields)
                      ext-encode-fns (mapv gen-encode-fn ext-fields)
                      ext-decode-fns (mapv #(gen-decode-fn % enums-by-group) ext-fields)
                      ]
                  {(keywordize msg-name)
                   {:msg-id msg-id
                    :msg-key (keywordize msg-name)
                    :payload-size payload-size
                    :extension-payload-size (+ payload-size payload-size-ext)
                    :fields fields
                    :encode-fns encode-fns
                    :decode-fns decode-fns
                    :extension-fields ext-fields
                    :extension-encode-fns ext-encode-fns
                    :extension-decode-fns ext-decode-fns
                    :crc-seed crc-seed
                    }
                  }))))
       all-descriptions
         (when descriptions
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
                              })))))]
   {:descriptions all-descriptions
    :enum-to-value enum-to-value
    :enums-by-group enums-by-group
    :messages-by-keyword messages-by-keyword
    :messages-by-id (apply merge (map #(hash-map (:msg-id %) %)
                                    (vals messages-by-keyword)))
    :source file-name}))

(defn add-mavlink
  "Given two mavlink maps, merge the contents of the second with the first."
  [{:keys [descriptions enum-to-value enums-by-group
           messages-by-keyword messages-by-id source]} new-part]
  (let [conflicts (filterv #(% enum-to-value) (keys (:enum-to-value new-part)))]
    (when-not (empty? conflicts)
      (throw (ex-info "Enum values conflict"
                      {:cause :enum-conflicts
                       :conflicts conflicts
                       :source source}))))
  (let [conflicts (filterv #(get messages-by-id %) (keys (:messages-by-id new-part)))]
    (when-not (empty? conflicts)
      (throw (ex-info "Message ID's conflict"
                      {:cause :message-id-conflicts
                       :conflicts conflicts
                       :source source}))))
  (let [conflicts (filterv #(% messages-by-keyword) (keys (:messages-by-keyword new-part)))]
    (when-not (empty? conflicts)
      (throw (ex-info "Message names conflict"
                      {:cause :message-name-conflicts
                       :conflicts conflicts
                       :source source}))))
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
                    (throw (ex-info "Missing a required include file"
                                     {:cause :missing-xml-include
                                      :file file
                                      :xml-sources xml-sources})))))))]
    (let [sources-with-zippers
            (mapv #(let [{:keys [xml-file xml-source]} %
                         zipper (-> xml-source xml/parse zip/xml-zip)
                         file-name (or (zip-xml/xml1-> zipper :mavlink (zip-xml/attr :file))
                                       xml-file)]
                     (when-not file-name
                       (throw (ex-info "Unable to determine the XML file name"
                                       {:cause :missing-xml-file-id
                                        :xml-source xml-source})))
                     (assoc % :zipper zipper
                              :file-name file-name))
                  xml-sources)]
      (doseq [{:keys [zipper]} sources-with-zippers]
        (verify-includes zipper))
      sources-with-zippers)))

