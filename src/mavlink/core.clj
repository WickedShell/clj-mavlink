(ns mavlink.core
  (:require [mavlink.checksum :refer :all]
            [mavlink.type :refer [byte-to-long]]
            [mavlink.mavlink-xml :refer :all])
  (:import [java.nio ByteBuffer ByteOrder]))

(def ^:const MAVLINK1-START-VALUE 254)
(defonce MAVLINK1-START-BYTE (.byteValue (new Long MAVLINK1-START-VALUE)))
(def ^:const MAVLINK1-HDR-SIZE 6)

(def ^:const MAVLINK2-START-VALUE 253)
(defonce MAVLINK2-START-BYTE (.byteValue (new Long MAVLINK2-START-VALUE)))
(def ^:const MAVLINK2-HDR-SIZE 10)


(defn parse
  "Given a map with the specifications for a mavlink instance, return a
   map of the mavlink instance.
  
   The map should contain the following bindings:

   :descriptions - true or false indicating whether to save descriptions
   :xml-sources    - either a vector of 1 or more maps holding the XML sources:
      [{:xml-file     - holds the filename of the XML file
        :xml-source - An XML source suitable for clojure.data.xml/parse
        }]
                     or a vector of 1 or more XML sources

  For example: {:xml-sources [{:xml-file test-parse.xml
                               :xml-source (-> test/resources/test-parse.xml io/input-stream)}]
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
    :bad-checksum           - obviously a bad checksum
  "
  [{:keys [descriptions xml-sources] :as info}]
  {:pre [(pos? (count xml-sources))
         ]}
   (when-let [parsed (get-xml-zippers xml-sources)]
     (let [options {:descriptions descriptions}]
       (loop [source (first parsed)
              rest-parsed (rest parsed)
              mavlink {}]
         (if (nil? source)
           mavlink
           (let [mavlink-part (get-mavlink options
                                           (:xml-file source) (:xml-zipper source))]
             (recur (first rest-parsed)
                    (rest rest-parsed)
                    (if (empty? mavlink)
                      mavlink-part
                      (add-mavlink mavlink mavlink-part (:xml-file source))))))))))

(defn open-channel
  "Given a mavlink (the result of parse), and the transmission specification for
   the protocol (including the incompat and compat flags (if they are not given, use 0)),
   the system id, and the component id, return a channel ready to use to encode and
   decode messages."
  [mavlink {:keys [protocol incompat-flags compat-flags] :as tx-protocol} 
   system-id component-id]
  {:pre [(instance? Long system-id)
         (instance? Long component-id)
         (map? tx-protocol)
         (or (= protocol :mavlink1) (= protocol :mavlink2))
         (map? (:messages-by-keyword mavlink))
         (map? (:messages-by-id mavlink))
         ]}
  (declare start-state)
  {:mavlink mavlink
   :tx-protocol {:protocol protocol
                 :incompat-flags (or incompat-flags 0)
                 :compat-flags (or compat-flags 0)}
   :system-id system-id
   :component-id component-id
   :sequence-id-atom (atom 0)
   :last-values (apply merge (map #(let [{:keys [msg-key default-msg]} %]
                                     {msg-key (ref default-msg)})
                                  (vals (:messages-by-keyword mavlink))))
   :decode-sm (atom start-state)
   :decode-message-id (atom 0)
   :statistics (atom {:bytes-received 0
                      :messages-decoded 0
                      :messages-encoded 0
                      :skipped-tx-sequences 0
                      :bad-checksums 0})
   :input-buffer
     (let [byte-buffer (ByteBuffer/allocate MAX-MESSAGE-SIZE)]
       (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
       byte-buffer)})

(defn encode-mavlink1
  "Encodes a mavlink1 message map returning a byte array, suitable for sending over the wire.
   The system-id, component-id and sequence-id may all be specified in the message map;
   if specified in the message map, the calues will override the default values. If the
   the sequence-id is specified, in addition to overriding the default sequence-id, the
   atom used to generate the default sequence-id is set to this value."
  ^bytes [{:keys [mavlink statistics system-id component-id sequence-id-atom] :as channel}
          {:keys [message-id] :as message-map}]
  {:pre [(keyword? message-id)
         (message-id (:messages-by-keyword mavlink))
         (<= 0 (:msg-id (message-id (:messages-by-keyword mavlink))) 255)   ; mavlink 1.0 only
         (instance? Long system-id)
         (instance? Long component-id)
         (map? (:messages-by-keyword mavlink))
         (instance? clojure.lang.Atom sequence-id-atom)
         (instance? clojure.lang.Atom statistics)
         ]}
  (let [{:keys [messages-by-keyword enum-to-value]} mavlink
        {:keys [encode-fns ^long payload-size crc-seed
                default-msg ^long msg-id msg-key]} (message-id messages-by-keyword)
        ^long sys-id (or (:system-id message-map) system-id)
        ^long comp-id (or (:component-id message-map) component-id)
        ^long seq-id (if-let [seq-id- (:sequence-id message-map)]
                       (reset! sequence-id-atom (mod seq-id- 256))
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
    ; now copy the array from the payload to the packed array.
    (java.lang.System/arraycopy (.array payload) 0 packed 6 (.position payload))
    ; finally calculate and put the checksum in, lsb first.
    (let [checksum (compute-checksum packed 1 (+ 6 payload-size) crc-seed)]
      (aset-byte packed (+ 6 payload-size) (.byteValue (new Long (bit-and checksum 0xff))))
      (aset-byte packed (+ 7 payload-size) (.byteValue (new Long (bit-and (bit-shift-right checksum 8) 0xff)))))
    packed))

(defn encode-mavlink2
  "Encodes a message map returning a byte array, suitable for sending over the wire.
   The system-id, component-id and sequence-id may all be specified in the message map;
   if specified in the message map, the calues will override the default values. If the
   the sequence-id is specified, in addition to overriding the default sequence-id, the
   atom used to generate the default sequence-id is set to this value."
  ^bytes [{:keys [mavlink statistics system-id component-id sequence-id-atom
                  incompat-flags compat-flags] :as channel}
          {:keys [message-id] :as message-map}]
  {:pre [(message-id (:messages-by-keyword mavlink))
         (<= 0 (:msg-id (message-id (:messages-by-keyword mavlink))) 16777215)
         (instance? Long system-id)
         (instance? Long component-id)
         (map? (:messages-by-keyword mavlink))
         (instance? clojure.lang.Atom sequence-id-atom)
         (instance? clojure.lang.Atom statistics)
         ]}
  (let [{:keys [messages-by-keyword enum-to-value]} mavlink
        {:keys [encode-fns encode-fns-ext ^long payload-size crc-seed
                default-msg ^long msg-id msg-key]} (message-id messages-by-keyword)
        ^long sys-id (or (:system-id message-map) system-id)
        ^long comp-id (or (:component-id message-map) component-id)
        ^long seq-id (if-let [seq-id- (:sequence-id message-map)]
                       (reset! sequence-id-atom (mod seq-id- 256))
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
        packed (byte-array (+ MAVLINK2-HDR-SIZE payload-size 2))]
    (aset-byte packed 0 MAVLINK1-START-BYTE)
    (aset-byte packed 1 (.byteValue (new Long payload-size)))
    (aset-byte packed 2 (.byteValue (new Long ^long incompat-flags)))
    (aset-byte packed 3 (.byteValue (new Long ^long compat-flags)))
    (aset-byte packed 4 (.byteValue (new Long seq-id)))
    (aset-byte packed 5 (.byteValue (new Long sys-id)))
    (aset-byte packed 6 (.byteValue (new Long comp-id)))
    (aset-byte packed 7 (.byteValue (new Long (bit-and msg-id 0xff))))
    (aset-byte packed 8 (.byteValue (new Long (bit-and (bit-shift-right msg-id 8) 0xff))))
    (aset-byte packed 9 (.byteValue (new Long (bit-and (bit-shift-right msg-id 16) 0xff))))

    ; encode the payload
    (doseq [encode-fn (concat encode-fns encode-fns-ext)]
      (encode-fn payload message))
  
    ; trim the message and fix the payload size
    (while (zero? (.get payload (dec (.position payload))))
      (.position payload (dec (.position payload))))
    (let [trimmed-payload-size (.position payload)]
      (aset-byte packed (byte-to-long (.byteValue (new Long trimmed-payload-size))))

      ; now copy the array from the payload to the packed array.
      (java.lang.System/arraycopy (.array payload) 0 packed 6 (.position payload))

      ; finally calculate and put the checksum in, lsb first.
      (let [checksum (compute-checksum packed 1 (+ MAVLINK2-HDR-SIZE trimmed-payload-size) crc-seed)]
        (aset-byte packed (+ MAVLINK2-HDR-SIZE trimmed-payload-size)
                   (.byteValue (new Long (bit-and checksum 0xff))))
        (aset-byte packed (+ 1 MAVLINK2-HDR-SIZE trimmed-payload-size)
                   (.byteValue (new Long (bit-and (bit-shift-right checksum 8) 0xff)))))
      packed)))

(defn encode
  "Encodes a message map returning a byte array, suitable for sending over the wire.
   The system-id, component-id and sequence-id may all be specified in the message map;
   if specified in the message map, the calues will override the default values. If the
   the sequence-id is specified, in addition to overriding the default sequence-id, the
   atom used to generate the default sequence-id is set to this value."
  ^bytes [{:keys [mavlink statistics tx-protocol] :as channel}
          {:keys [message-id] :as message-map}]
  {:pre [(keyword? message-id)
         (message-id (:messages-by-keyword mavlink))
         (map? (:messages-by-keyword mavlink))
         (map? tx-protocol)
         (instance? clojure.lang.Atom statistics)
         ]}
  (let [{:keys [protocol]} tx-protocol
        packed (if (= protocol :mavlink1)
                 (encode-mavlink1 channel message-map)
                 (if (= protocol :mavlink2)
                   (encode-mavlink2 channel message-map)
                   (throw (ex-info "Bad protocol"
                                   {:cause :bad-protocol
                                    :protocol protocol}))))]
    (when packed
      (swap! statistics #(assoc % :messages-encoded (+ (:messages-encoded %) (count packed)))))
    packed))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Start decode state machine state functions.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(declare start-state)
(defn verify-payload
  "Verifies that the payload size is less than the payload specified for the message.
   If the message-info is nil, then nil is returned, otherwise true or false is
   returned if the payload size is valid or invalid."
  [hdr-payload-size message-info]
  (when-let [{:keys [payload-size]} message-info]
    (>= payload-size hdr-payload-size)))


(defn checksum-msb-state
  "This byte the msb of the checksum. Verify the checksum, if it verifies,
   then decode the message and return it. Use the state map to hold the return message,
   simply dissoc the fields that are not used. In case of error, update the statistics
   and restart the decode state machine."
  [{:keys [decode-sm decode-message-id 
           ^ByteBuffer input-buffer mavlink statistics] :as channel} a-byte]
  (let [{:keys [messages-by-id]} mavlink
        {:keys [crc-seed payload-size msg-key decode-fns]} (get messages-by-id @decode-message-id)
        last-idx (dec (.position input-buffer))
        checksum-lsb (byte-to-long (.get input-buffer (int last-idx)))
        checksum (bit-or (bit-and checksum-lsb 0xff)
                         (bit-and (bit-shift-left (byte-to-long a-byte) 8) 0xff00))
        checksum-calc (compute-checksum input-buffer 1 last-idx crc-seed)]
    (if (== checksum checksum-calc)
      (let [decode-payload-size (byte-to-long (.get input-buffer 1))
            payload-start  (if (= (.get input-buffer 0) MAVLINK1-START-BYTE)
                             MAVLINK1-HDR-SIZE
                             MAVLINK2-HDR-SIZE)]
        (.position input-buffer payload-start)
        (when (and (= (.get input-buffer 0) MAVLINK2-START-BYTE)
                   (> payload-size decode-payload-size))
          (println "FIlling in a trimmed message.")
          (doseq [i (range (- payload-size decode-payload-size))]
            (.put input-buffer (byte 0))))
        ; decode the message, restart the decode state machine, then
        ; save the message and return it!
        (let [message (apply merge
                             {:message-id msg-key
                              :sequence-id (if (= (.get input-buffer 0) MAVLINK1-START-BYTE)
                                             (byte-to-long (new Long (.get input-buffer 2)))
                                             (byte-to-long (new Long (.get input-buffer 4))))
                              :system-id (if (= (.get input-buffer 0) MAVLINK1-START-BYTE)
                                             (byte-to-long (new Long (.get input-buffer 3)))
                                             (byte-to-long (new Long (.get input-buffer 5))))
                              :component-id (if (= (.get input-buffer 0) MAVLINK1-START-BYTE)
                                             (byte-to-long (new Long (.get input-buffer 4)))
                                             (byte-to-long (new Long (.get input-buffer 6))))}
                             (map #(% input-buffer) decode-fns))]
          (swap! statistics assoc :messages-decoded (inc (:messages-decoded @statistics)))
          (reset! decode-sm start-state)
          message))
      (do
        (swap! statistics assoc :bad-checksums (inc (:bad-checksums @statistics)))
        (reset! decode-sm start-state)
        nil))))

(defn checksum-lsb-state
  "Save the byte in the checksum-lsb, then go to checkbyte-msb-state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm checksum-msb-state)
  nil)

(defn payload-state
  "Put the byte in the buffer, if this is the last byte of the payload,
   then go to checkbyte-lsb-state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (let [decode-payload-size (byte-to-long (.get input-buffer 1))]
    (when (>= (.position input-buffer) (if (= (.get input-buffer 0) MAVLINK1-START-BYTE)
                                         (+ MAVLINK1-HDR-SIZE decode-payload-size)
                                         (+ MAVLINK2-HDR-SIZE decode-payload-size)))
      (reset! decode-sm checksum-lsb-state)))
  nil)

(defn msg-id-byte3-state
  "Add the byte to the message id, then verify the message, like message-id state
   handling MAVlink 1 protocol."
  [{:keys [decode-sm decode-message-id ^ByteBuffer input-buffer mavlink] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (let [low-idx (- (.position input-buffer) 3)
        low-byte (byte-to-long (.get input-buffer low-idx))
        middle-byte (byte-to-long (.get input-buffer (inc low-idx)))
        high-byte (byte-to-long (.get input-buffer (+ low-idx 2)))
        {:keys [messages-by-id]} mavlink
        message-id (+ (bit-and (bit-shift-left high-byte 16) 0xff0000)
                      (bit-and (bit-shift-left middle-byte 8) 0xff00)
                      (bit-and low-byte 0xff))]
    (if (verify-payload (byte-to-long (.get input-buffer 1))
                        (get messages-by-id message-id))
      (do
        (reset! decode-sm payload-state)
        (reset! decode-message-id message-id))
      (reset! decode-sm start-state)))
  nil)

(defn msg-id-byte2-state
  "Add the byte to the message id, then go to message id byte 3 state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm msg-id-byte3-state)
  nil)

(defn msg-id-state
  "Verify that the message id is valid and the payload size is less than the payload
   of the message spec for the message id. If it isn't go back to the start state
   Then go to the message id state."
  [{:keys [decode-sm decode-message-id  ^ByteBuffer input-buffer mavlink] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (let [{:keys [messages-by-id]} mavlink
        message-id (byte-to-long a-byte)]
    (if (= (.get input-buffer 0) MAVLINK1-START-BYTE)
      (if (verify-payload (byte-to-long (.get input-buffer 1))
                          (get messages-by-id message-id))
        (do
          (reset! decode-sm payload-state)
          (reset! decode-message-id message-id))
        (reset! decode-sm start-state))
      (reset! decode-sm msg-id-byte2-state)))
  nil)

(defn compat-flags-state
  "Save the byte in the compat-flags, then go to sequence id state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm msg-id-state)
  nil)

(defn incompat-flags-state
  "Save the byte in the incompat-flags, then go to compat-flags-state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm compat-flags-state)
  nil)

(defn compid-state
  "Save the byte in the input buffer and save it as the component id.
   Then go to the message id state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm msg-id-state)
  nil)

(defn sysid-state
  "Save the byte in the input buffer and save it as the system id.
   Then go to the component id state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm compid-state)
  nil)

(defn seq-id-state
  "Save the byte in the input buffer and save it as the sequence id;
   then go to the system id state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm sysid-state)
  nil)

(defn payload-size-state
  "Save the byte in the input buffer, and save it as the payload size and go
   to the appropriate next state depending on the protocol."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm (if (= (.get input-buffer 0) MAVLINK1-START-BYTE)
                              seq-id-state
                              incompat-flags-state))
  nil)

(defn start-state
  "Decode start state, looking for start byte for either MAVlink 1 or MAVlink 2.
   Ignore every other byte. Go to Payload size state when a start byte is seen;
   don't save the byte regardless. When a start byte is seen, completely reset
   the state."
  [{:keys [decode-sm decode-message-id ^ByteBuffer input-buffer] :as channel} a-byte]
  (when (or (= a-byte MAVLINK1-START-BYTE)
            (= a-byte MAVLINK2-START-BYTE))
    (reset! decode-sm payload-size-state)
    (reset! decode-message-id nil)
    (.clear input-buffer)
    (.put input-buffer ^byte a-byte)
    nil))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; End decode state machine state functions.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn decode-byte
  "Add a byte to the mavlink input buffer, and if we have a message check the
   checksum. If the checksum matches decode the message and return it.
   Whenever an error occurs, go back to the start state. As soon as a start byte is recognized,
   start the state machine and attempt to decode a message.
   Both MAVlink 1.0 and MAVlink 2.0 are supported."
  [{:keys [statistics decode-sm last-values] :as channel} a-byte]
  {:pre [(instance? Byte a-byte)
         (fn? @decode-sm)]}
  (swap! statistics assoc :bytes-received (inc (:bytes-received @statistics)))
  (when-let [message (@decode-sm channel a-byte)]
    (when-let [message-key (:message-id message)]
      (when-let [msg-ref (message-key last-values)]
        (dosync (ref-set msg-ref message))))
    message))

(defn decode-byte-old
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
      ; when have at minimum the header and checksum, continue
      (when (>= (.position input-buffer) 8)
        (let [payload-size (byte-to-long (.get input-buffer 1))
              msg-id (byte-to-long (.get input-buffer 5))
              message-map (get messages-by-id msg-id)]
          (if (or (nil? message-map)
                  (> payload-size (:payload-size message-map)))
            (do
              (.clear input-buffer)
              nil)
            ; when I have the header, payload and checksum, continue
            (when (>= (.position input-buffer) (+ 8 payload-size))
              (let [{:keys [decode-fns msg-key last-value crc-seed]} message-map
                    checksum (let [lsb (.get input-buffer ^int (+ 6 payload-size))
                                   msb (.get input-buffer ^int (+ 7 payload-size))]
                               (bit-or (bit-and lsb 0xff)
                                       (bit-and (bit-shift-left msb 8) 0xff00)))
                    checksum2 (compute-checksum input-buffer 1 (+ 6 payload-size) crc-seed)
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
  ([channel ^bytes some-bytes]
   {:pre [(= (Class/forName "[B") (class some-bytes))]}
   (decode-bytes channel some-bytes (alength some-bytes)))
  ([channel ^bytes some-bytes num-bytes]
   {:pre [(= (Class/forName "[B") (class some-bytes))]}
   (loop [idx 0
          messages []]
     (if (>= idx num-bytes)
       (when-not (empty? messages)
         messages)
       (if-let [message (decode-byte channel (aget some-bytes idx))]
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
