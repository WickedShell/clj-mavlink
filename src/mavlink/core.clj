(ns mavlink.core
  (:require [mavlink.checksum :refer :all]
            [mavlink.type :refer [byte-to-long]]
            [mavlink.mavlink-xml :refer :all])
  (:import [java.nio ByteBuffer ByteOrder]))

(defonce MAVLINK1-START-BYTE (.byteValue (new Long 254)))

(defn parse
  "Given a map with the specifications for a mavlink instance, return a
   map of the mavlink instance.
  
   The map should contain the following bindings:

   :mavlink-1-0    - not used yet
   :system-id      - an integer type holding a byte value of the system id
   :component-id   - an integer type holding a byte value of the component id
   :xml-sources    - either a vector of 1 or more maps holding the XML sources:
      [{:xml-file     - holds the filename of the XML file
        :xml-source - An XML source suitable for clojure.data.xml/parse
        }]
                     or a vector of 1 or more XML sources

  For example: {:mavlink-1-0 test-parse.xml
                :xml-sources [{:xml-file test-parse.xml
                               :xml-source (-> test/resources/test-parse.xml io/input-stream)}]
                :system-id 99
                :component-id 88
                :descriptions true}

  Possible :cause failures from ExceptionInfo exceptions:
    :null-pointer           - obviously a null pointer
    :string-not-number      - string conversion to a number failed usually due to non numeric characters
    :unknown-type           - unknown type specifier in an XML file
    :no-write-fn            - the type is missing a write function
    :no-read-fn             - the type is missing a read function
    :enum-conflicts         - there is a name conflict in the enumerated types
    :message-id-conflicts   - there is a conflict with the message id values in an XML file
    :message-name-conflicts - there is a conflict witht he message names in an XML file
    :missing-xml-include    - an XML file is included, but no source was identified for this file
    :missing-xml-file-id    - an XML source is missing an XML file indicator
    :encode-bad-keys        - An encode message has bad message field keys
    :undefined-enum         - A message value uses an unidentified enumerated value
    :unknown-message        - unknown message id while decoding
    :bad-checksum           - obviously a bad checksum
  "
  [{:keys [mavlink-1-0 system-id component-id descriptions
                xml-sources] :as info}]
  {:pre [mavlink-1-0
         (instance? Long system-id)
         (instance? Long component-id)
         (<= 0 system-id 255)
         (<= 0 component-id 255)
         (pos? (count xml-sources))
         ]}
   (when-let [parsed (get-xml-zippers xml-sources)]
     (let [options {:system-id system-id
                    :component-id component-id
                    :descriptions descriptions
                    :mavlink-version 2}]
       (loop [source (first parsed)
              rest-parsed (rest parsed)
              mavlink {}]
         (if (nil? source)
           (assoc mavlink :input-buffer
                            (let [byte-buffer (ByteBuffer/allocate MAX-MESSAGE-SIZE)]
                              (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
                              byte-buffer)
                          :system-id system-id
                          :component-id component-id
                          :sequence-id-atom (atom 0))
           (let [mavlink-part (get-mavlink options
                                           (:xml-file source) (:xml-zipper source))]
             (recur (first rest-parsed)
                    (rest rest-parsed)
                    (if (empty? mavlink)
                      mavlink-part
                      (add-mavlink mavlink mavlink-part (:xml-file source))))))))))

(defn encode
  "Encodes a message map returning a byte array, suitable for sending over the wire.
   The system-id, component-id and sequence-id may all be specified in the message map;
   if specified in the message map, the calues will override the default values. If the
   the sequence-id is specified, in addition to overriding the default sequence-id, the
   atom used to generate the default sequence-id is set to this value."
  ^bytes [{:keys [system-id component-id messages-by-keyword 
                  enum-to-value sequence-id-atom] :as mavlink}
          {:keys [message-id] :as message-map}]
  {:pre [(keyword? message-id)
         (message-id messages-by-keyword)
         (<= 0 (:msg-id (message-id messages-by-keyword)) 255)   ; mavlink 1.0 only
         (instance? Long system-id)
         (instance? Long component-id)
         (map? messages-by-keyword)
         (instance? clojure.lang.Atom sequence-id-atom)
         ]}
  (let [{:keys [encode-fns ^long payload-size magic-byte default-msg ^long msg-id msg-key]} (message-id messages-by-keyword)
        ^long sys-id (or (:system-id message-map) system-id)
        ^long comp-id (or (:component-id message-map) component-id)
        ^long seq-id (if-let [seq-id- (:sequence-id message-map)]
                       (reset! sequence-id-atom seq-id-) ; FIXME: range check this
                       (swap! sequence-id-atom #(mod (inc %) 256)))
        merged-message (merge default-msg
                              (dissoc message-map :message-id :system-id :component-id :sequence-id))
        bad-keys (let [bad-keys (filterv (fn[k] (nil? (k default-msg))) (keys merged-message))]
                   (when-not (empty? bad-keys)
                     (throw (ex-info "Unknown message fields"
                                     {:cause :encode-bad-keys
                                      :fields bad-keys}))))
        message (apply merge (map #(let [[current-key current-value] %]
                                     (if (keyword? current-value)
                                       (if-let [enum-val (current-value enum-to-value)]
                                         { current-key enum-val }
                                         (throw (ex-info "Undefined enum"
                                                         {:cause :undefined-enum
                                                          :enum current-value})))
                                       { current-key current-value }))
                                     merged-message))
        payload (let [byte-buffer (ByteBuffer/allocate payload-size)]
                  (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
                  byte-buffer)
        packed (byte-array (+ 6 payload-size 2))]
    (aset-byte packed 0 MAVLINK1-START-BYTE)
    (aset-byte packed 1 (.byteValue (new Long payload-size)))
    (aset-byte packed 2 (.byteValue (new Long seq-id)))
    (aset-byte packed 3 (.byteValue (new Long sys-id)))
    (aset-byte packed 4 (.byteValue (new Long comp-id)))
    (aset-byte packed 5 (.byteValue (new Long msg-id)))
    (doseq [encode-fn encode-fns]
      (encode-fn payload message))
    ; for Mavlink 2.0 ...
    ; trim the message and fix the length
    ; (while (zero? (.get payload (dec (.position payload))))
    ;   (.position payload (dec (.position payload)))
    ;   (.put payload 1 (byte (dec (.get payload 1)))))
    (java.lang.System/arraycopy (.array payload) 0 packed 6 (.position payload))
    (let [checksum (compute-checksum packed 1 (+ 6 payload-size) magic-byte)]
      (aset-byte packed (+ 6 payload-size) (.byteValue (new Long (bit-and checksum 0xff))))
      (aset-byte packed (+ 7 payload-size) (.byteValue (new Long (bit-and (bit-shift-right checksum 8) 0xff)))))
    packed))

(defn decode-byte
  "Add a byte to the mavlink input buffer, and if we have a message check the
   checksum. If the checksum matches decode the message, clear the buffer and
   return the message.  If the checksum mis-matches, clear the buffer and
   report the error."
  [{:keys [messages-by-id ^ByteBuffer input-buffer] :as mavlink} a-byte]
  {:pre [input-buffer
         messages-by-id
         (instance? Byte a-byte)]}
  (if (zero? (.position input-buffer))
    (when (= a-byte MAVLINK1-START-BYTE)
      (.put input-buffer ^byte a-byte)
      nil)
    (do
      (.put input-buffer ^byte a-byte)
      ; when I have at minimum the header and checksum, continue
      (when (>= (.position input-buffer) 8)
        (let [payload-size (byte-to-long (.get input-buffer 1))]
          ; when I have the header, payload and checksum, continue
          (when (>= (.position input-buffer) (+ 8 payload-size))
            (let [msg-id (byte-to-long (.get input-buffer 5))
                  {:keys [decode-fns msg-key last-value magic-byte]} (get messages-by-id msg-id)
                  checksum (let [lsb (.get input-buffer ^int (+ 6 payload-size))
                                 msb (.get input-buffer ^int (+ 7 payload-size))]
                             (bit-or (bit-and lsb 0xff)
                                     (bit-and (bit-shift-left msb 8) 0xff00)))
                  checksum2 (compute-checksum input-buffer 1 (+ 6 payload-size) magic-byte)
                  ]
              (.position input-buffer 6)
              (if (== checksum checksum2)
                (let [message (apply merge {:message-id msg-key
                                            :sequence-id (byte-to-long (.get input-buffer 2))
                                            :system-id (byte-to-long (.get input-buffer 3))
                                            :component-id (byte-to-long (.get input-buffer 4))}
                                           (map #(% input-buffer) decode-fns))]
                  (.clear input-buffer)
                  (dosync
                    (ref-set last-value message)))
                (do
                  (.clear input-buffer)
                  (if (or (nil? msg-key) (nil? magic-byte))
                    (throw (ex-info "Unknown message"
                                    {:cause :unknown-message
                                     :message-id msg-id}))
                    (throw (ex-info "Checksum mismatch"
                                    {:cause :bad-checksum
                                     :message-checksum checksum
                                     :calculated-checksum checksum2
                                     :message-id msg-id
                                     :payload-size payload-size
                                     :message-key msg-key}))))))))))))

(defn decode-bytes
  "Decodes a MAVLink message from a byte array.
   This function can be called with either a byte array or a single byte.
   Nil is returned if no messages could be decoded. If a byte completes a message
   then the message-map is returned. If a byte array completes one or more messages,
   then those messages are returned in a vector, otherwise nil is treturned."
  ([mavlink ^bytes some-bytes]
   {:pre [(= (Class/forName "[B") (class some-bytes))]}
   (decode-bytes mavlink some-bytes (alength some-bytes)))
  ([mavlink ^bytes some-bytes num-bytes]
   {:pre [(= (Class/forName "[B") (class some-bytes))]}
   (loop [idx 0
          messages []]
     (if (>= idx num-bytes)
       (when-not (empty? messages)
         messages)
       (if-let [message (decode-byte mavlink (aget some-bytes idx))]
         (recur (inc idx)
                (conj messages message))
         (recur (inc idx)
                messages))))))

(defn get-ref
  "Return the Ref for a message. If the message-key is a keyword it is looked
   up from the messages-by-keyword. Otherwise it is assumed to be a number and
   is used to find the ref in the messages-by-id."
  [{:keys [messages-by-keyword messages-by-id] :as mavlink} message-key]
  {:pre [messages-by-keyword
         messages-by-id
         ]}
  (if (keyword? message-key)
    (:last-value (message-key messages-by-keyword))
    (:last-value (get messages-by-id message-key))))

(defn get-last-message
  "Return the last value map of the message received.
   message-key should be a keyword or the id value of a message."
  [{:keys [messages-by-keyword messages-by-id] :as mavlink} message-key]
  {:pre [messages-by-keyword
         messages-by-id
         ]}
  (if (keyword? message-key)
    @(:last-value (message-key messages-by-keyword))
    @(:last-value (get messages-by-id message-key))))

(defn get-description
  "Return the description, only useful if descriptions were saved.
   Otherwise nil is returned."
  [{:keys [descriptions] :as mavlink} msg-key]
  (when descriptions
    (msg-key descriptions)))
