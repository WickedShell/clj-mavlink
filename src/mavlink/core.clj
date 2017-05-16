(ns mavlink.core
  (:require [mavlink.checksum :refer :all]
            [mavlink.type :refer [byte-to-long]]
            [mavlink.mavlink-xml :refer :all])
  (:import [java.nio ByteBuffer ByteOrder]
           [java.security MessageDigest]
           [java.lang System]))

(use 'clojure.pprint)  ;; FIXME delete this sucker after testing

(defonce IFLAG-SIGNED 0x01)
(def sha256 ^MessageDigest (MessageDigest/getInstance "SHA-256"))

(def ^:const MAVLINK1-START-VALUE 254)
(defonce MAVLINK1-START-BYTE (.byteValue (new Long MAVLINK1-START-VALUE)))
(def ^:const MAVLINK1-HDR-SIZE 6)
(def ^:const MAVLINK1-HDR-CRC-SIZE 8)

(def ^:const MAVLINK2-START-VALUE 253)
(defonce MAVLINK2-START-BYTE (.byteValue (new Long MAVLINK2-START-VALUE)))
(def ^:const MAVLINK2-HDR-SIZE 10)
(def ^:const MAVLINK2-HDR-CRC-SIZE 12)
(def ^:const MAVLINK2-HDR-CRC-SIGN-SIZE 25)
(def ^:const SIGN-PACKETS-FLAG 0x1)

(def ^:const ONE-MINUTE 6000000)

(def ZERO-BYTE (byte 0))

(defn get-timestamp
 "Get current time timestamp."
 []
 (quot (System/nanoTime) 10000))

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

(defn sync-timestamps
  "Reinitialize the transmist and receive timestamps to the current time.
   Note they will be set to the identical time."
  [{:keys [rcv-timestamp tx-timestamp] :as channel} & args]
  (reset! rcv-timestamp (reset! tx-timestamp (get-timestamp))))

(defn update-channel
  "Update the MAVLink channel.
   Args is sequence of keyword value pairs (thus the count of args MUST be even)
   where the keyword is one of the following:
   :protocol :mavlink1 or :mavlink2 for encoding
   :secret-key a 32 byte array (the length MUST be 32) or nil to indicate don't sign
   :accept-unsigned-packets true or false
  "
  [{:keys [protocol secret-key rcv-timestamp tx-timestamp
           accept-unsigned-packets link-id] :as channel} & args]
  {:pre [(even? (count args))]}
  (loop [k (first args)
         v (second args)
         rest-args (rest (rest args))]
    (condp = k
      :link-id
        (if (instance? Long v)
          (reset! link-id (.byteValue (new Long ^long v)))
          (throw (ex-info "Link id not a Long."
                          {:cause :bad-link-id
                           :value v})))
      :protocol
        (if (or (= v :mavlink1)
                (= v :mavlink2))
          (reset! protocol v)
          (throw (ex-info "Bad protocol"
                          {:cause :bad-protocol
                           :value v})))
      :secret-key
        (if (or (nil? v)
                (>= (count v) 32)) ; see sign-packet, first 32 bytes will be used.
          (reset! secret-key v)
          (throw (ex-info "Bad secret key"
                          {:cause :bad-secret-key
                           :value v})))
      :rcv-timestamp
        (if (instance? Long v)
          (reset! rcv-timestamp v)
          (throw (ex-info "Timestamp not a Long."
                          {:cause :bad-timestamp
                           :value v})))
      :tx-timestamp
        (if (instance? Long v)
          (reset! tx-timestamp v)
          (throw (ex-info "Timestamp not a Long."
                          {:cause :bad-timestamp
                           :value v})))
      :accept-unsigned-packets
        (if (or (true? v)
                (false? v))
          (reset! accept-unsigned-packets)
          (throw (ex-info "Bad accept-unsigned-packets"
                          {:cause :bad-accept-unsigned-packets
                           :value v})))
      (throw (ex-info "Unknown MAVLink 2 channel key"
                      {:cause :unknown-channel-key
                       :key k
                       :value v})))
    (when-not (empty? rest-args)
      (recur (first rest-args)
             (second rest-args)
             (rest (rest rest-args))))))

(defn open-channel
  "Given a mavlink (the result of parse), and the protocol specifications,
   system-id and component-id to be used for enclding messages;
   returns a channel ready to use to encode and decode messages.
  
   mavlink - is the mavlink map returned by a call to parse.
   protocol :mavlink1 or :mavlink2
   system-id system id for encoding
   component-id component id for encoding
   link-id byte to use for link id for encoding

   See update-channel to instantiate MAVLink2 specific.

  Note the developer must setup signing and provide the link-id and secret-key
  in the open-channel protocol map or update the protocol when they are ready to start signing.

  The decode function will decode messages based on the type of message, which is determined
  by the message start byte 0xfe for MAVLink 1.0 and 0xfd for MAVLink 2.0. If the incompat-flags
  indicate signing, then signing will be verified (unless the :accept-unsigned-packets
  is set, in which case unsigned packets are accepted, but signed packets must verify.

  Timestamps just indicate forward progression (once a timestamp is seen, ignore
  anything earlier). So, the initial values of both the tx and rcv timestamps are 0.
  The :tx-timestamp and :rcv-timestamp of the protocol may be updated by the update-procotol function.

   "
  ([mavlink protocol system-id component-id] (open-channel mavlink protocol system-id component-id 0))
  ([mavlink protocol system-id component-id link-id]
  {:pre [(instance? Long system-id)
         (instance? Long component-id)
         (instance? Long link-id)
         (map? mavlink)
         (keyword? protocol)
         (map? (:messages-by-keyword mavlink))
         (map? (:messages-by-id mavlink))
         ]}
   (declare start-state)
   (when (= protocol :mavlink2)
     (println "FIXME currently the channel is per sys-id, comp-id, link-id ... this will not work in the long run, the channel should be per sys-id, comp-id for transmist, but decode-mavlink2 should track the incoming messages by sys-id, comp-id, link-id and keep a separate rcv-timestamp for each tuple."))
  {:mavlink mavlink
   :protocol (atom (or protocol :mavlink1))
   :secret-key (atom nil)                           ; MAVLink 2.0 encoding only
   :tx-timestamp (atom 0)                           ; MAVLInk 2.0 encoding only
   :link-id (atom (.byteValue (new Long ^long link-id)))  ; MAVLink 2.0 encoding only
   :accept-unsigned-packets (atom true)             ; MAVLink 2.0 decoding only
   :rcv-timestamp (atom (get-timestamp))            ; MAVLInk 2.0 decoding only
                                                    ; FIXME this needs to be set before first message is decoded when signing is active.
   :system-id system-id
   :component-id component-id
   :sequence-id-atom (atom 0)
   :last-values (apply merge (map #(let [{:keys [msg-key default-msg]} %]
                                     {msg-key (ref default-msg)})
                                  (vals (:messages-by-keyword mavlink))))
   :decode-sm (atom start-state)
   :decode-message-info (atom nil)
   :statistics (atom {:bytes-received 0
                      :messages-decoded 0
                      :messages-encoded 0
                      :skipped-tx-sequences 0
                      :bad-checksums 0
                      :bad-signatures 0
                      :bad-timestamps 0
                      :dropped-unsigned 0})
   :input-buffer
     (let [byte-buffer (ByteBuffer/allocate MAX-MESSAGE-SIZE)]
       (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
       byte-buffer)}))

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
        packed (byte-array (+ MAVLINK1-HDR-SIZE payload-size 2))]
    (aset-byte packed 0 MAVLINK1-START-BYTE)
    (aset-byte packed 1 (.byteValue (new Long payload-size)))
    (aset-byte packed 2 (.byteValue (new Long seq-id)))
    (aset-byte packed 3 (.byteValue (new Long sys-id)))
    (aset-byte packed 4 (.byteValue (new Long comp-id)))
    (aset-byte packed 5 (.byteValue (new Long msg-id)))
    (doseq [encode-fn encode-fns]
      (encode-fn payload message))
    ; now copy the array from the payload to the packed array.
    (System/arraycopy (.array payload) 0 packed MAVLINK1-HDR-SIZE (.position payload))
    ; finally calculate and put the checksum in, lsb first.
    (let [checksum (compute-checksum packed 1 (+ MAVLINK1-HDR-SIZE payload-size) crc-seed)]
      (aset-byte packed (+ MAVLINK1-HDR-SIZE payload-size) (.byteValue (new Long (bit-and checksum 0xff))))
      (aset-byte packed (+ 1 MAVLINK1-HDR-SIZE payload-size) (.byteValue (new Long (bit-and (bit-shift-right checksum 8) 0xff)))))
    packed))

(defn sign-packet
  "Sign the packet, it is assumed that the packet array has room for the
   13 bytes of the link-id, the 6 bytes of timestamp and the 6 bytes
   of the signature.

   The timestamp is determined from the system time every time, if the new
   timestamp is the same as the old, then add add 1.  Timestamps are
   in units of 10 microseconds.

   The link id and the first 6 bytes of the timestamp are appended to the
   packet starting at the signature-idx. Then the signature is calculated
   using SHA256 implemeneted by java.securty.MessageDigest.

   The signature is the first 6  bytes of the SHA256 hash of the bytes of the
   secret-key then the packet. Those bytes are then added to appended to the packet.
  "
  [{:keys [secret-key tx-timestamp] :as channel} ^bytes packet signature-start-idx link-id]
  {:pre [@secret-key]}
  (let [curr-timestamp (get-timestamp)
        sha256-start-idx (+ signature-start-idx 7) ; the packet plus the link id and timestamp
        timestamp-array (let [bb (ByteBuffer/allocate 8)]
                          (.order bb ByteOrder/LITTLE_ENDIAN)
                          (if (> curr-timestamp @tx-timestamp)
                            (reset! tx-timestamp curr-timestamp)
                            (swap! tx-timestamp inc))
                          (.putLong bb ^long @tx-timestamp)
                          (.array bb))]
    ; link ID
    (aset-byte packet signature-start-idx link-id) 
    ; timestamp
    (loop [idx (inc signature-start-idx)
           tidx 0]
      ; only take the first 6 bytes of the timestamp
      (when (< tidx 6) 
        (aset-byte packet idx (aget timestamp-array tidx)) 
        (recur (inc idx)
               (inc tidx))))
    ; SHA256 signature
    (.reset sha256)
    (.update sha256 @secret-key 0 32)
    (.update sha256 packet 0 sha256-start-idx)
    (let [sha256-bytes ^bytes (.digest sha256)]
      (loop [idx sha256-start-idx
             sidx 0]
        ; only take the first 6 bytes of the sha256 bytes
        (when (< sidx 6) 
          (aset-byte packet idx (aget sha256-bytes sidx)) 
          (recur (inc idx)
                 (inc sidx)))))))

(defn encode-mavlink2
  "Encodes a message map returning a byte array, suitable for sending over the wire.
   The system-id, component-id, link-id and sequence-id may all be specified in the message map;
   if specified in the message map, the values will override the default values. If the
   the sequence-id is specified, in addition to overriding the default sequence-id, the
   atom used to generate the default sequence-id is set to this value."
  ^bytes [{:keys [mavlink statistics system-id component-id link-id sequence-id-atom
                  secret-key] :as channel}
          {:keys [message-id] :as message-map}]
  {:pre [(message-id (:messages-by-keyword mavlink))
         (<= 0 (:msg-id (message-id (:messages-by-keyword mavlink))) 16777215)
         (instance? Long system-id)
         (instance? Long component-id)
         (map? (:messages-by-keyword mavlink))
         (instance? clojure.lang.Atom sequence-id-atom)
         (instance? clojure.lang.Atom secret-key)
         (instance? clojure.lang.Atom statistics)
         ]}
  (let [{:keys [messages-by-keyword enum-to-value]} mavlink
        {:keys [encode-fns extension-encode-fns ^long extension-payload-size
                crc-seed default-msg ^long msg-id msg-key]} (message-id messages-by-keyword)
        ^long sys-id (or (:system-id message-map) system-id)
        ^long comp-id (or (:component-id message-map) component-id)
        ^long seq-id (if-let [seq-id- (:sequence-id message-map)]
                       (reset! sequence-id-atom (mod seq-id- 256))
                       (swap! sequence-id-atom #(mod (inc %) 256)))
        merged-message (merge default-msg
                              (dissoc message-map :message-id :system-id :component-id :sequence-id :link-id))
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
        payload (let [byte-buffer (ByteBuffer/allocate extension-payload-size)]
                  (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
                  byte-buffer)
        packed (byte-array (+ extension-payload-size
                              (if @secret-key
                                MAVLINK2-HDR-CRC-SIGN-SIZE
                                MAVLINK2-HDR-CRC-SIZE)))
        incompat-flags (if @secret-key
                         SIGN-PACKETS-FLAG  ; only one possible flag, so no or'ing necessary
                         0)
        compat-flags 0]
    (aset-byte packed 0 MAVLINK2-START-BYTE)
    (aset-byte packed 1 (.byteValue (new Long extension-payload-size)))
    (aset-byte packed 2 (.byteValue (new Long incompat-flags)))
    (aset-byte packed 3 (.byteValue (new Long compat-flags)))
    (aset-byte packed 4 (.byteValue (new Long seq-id)))
    (aset-byte packed 5 (.byteValue (new Long sys-id)))
    (aset-byte packed 6 (.byteValue (new Long comp-id)))
    (aset-byte packed 7 (.byteValue (new Long (bit-and msg-id 0xff))))
    (aset-byte packed 8 (.byteValue (new Long (bit-and (bit-shift-right msg-id 8) 0xff))))
    (aset-byte packed 9 (.byteValue (new Long (bit-and (bit-shift-right msg-id 16) 0xff))))

    ; encode the payload
    (doseq [encode-fn (concat encode-fns extension-encode-fns)]
      (encode-fn payload message))
  
    ; trim the message and fix the payload size
    (while (and (pos? (.position payload))
                (zero? (.get payload (dec (.position payload)))))
      (.position payload (dec (.position payload))))

    (let [trimmed-payload-size (.position payload)]
      (aset-byte packed 1 (byte-to-long (.byteValue (new Long trimmed-payload-size))))

      ; now copy the array from the payload to the packed array.
      (when (pos? trimmed-payload-size)
        (System/arraycopy (.array payload) 0 packed MAVLINK2-HDR-SIZE trimmed-payload-size))

      ; finally calculate and put the checksum in, lsb first.
      (let [checksum (compute-checksum packed 1 (+ MAVLINK2-HDR-SIZE trimmed-payload-size) crc-seed)]
        (aset-byte packed (+ MAVLINK2-HDR-SIZE trimmed-payload-size)
                   (.byteValue (new Long (bit-and checksum 0xff))))
        (aset-byte packed (+ 1 MAVLINK2-HDR-SIZE trimmed-payload-size)
                   (.byteValue (new Long (bit-and (bit-shift-right checksum 8) 0xff)))))
      ; the packet is ready to go, if there is a secret-key, then the message should be signed
      (when @secret-key
        (sign-packet channel
                     packed
                     (+ MAVLINK2-HDR-CRC-SIZE trimmed-payload-size)
                     (or (:link-id message-map) @link-id)))  ; pass in the link id to sign the packet
      packed)))

(defn encode
  "Encodes a message map returning a byte array, suitable for sending over the wire.
   The system-id, component-id and sequence-id may all be specified in the message map;
   if specified in the message map, the calues will override the default values. If the
   the sequence-id is specified, in addition to overriding the default sequence-id, the
   atom used to generate the default sequence-id is set to this value."
  ^bytes [{:keys [mavlink statistics protocol] :as channel}
          {:keys [message-id] :as message-map}]
  {:pre [(keyword? message-id)
         (message-id (:messages-by-keyword mavlink))
         (map? (:messages-by-keyword mavlink))
         (instance? clojure.lang.Atom statistics)
         ]}
  (let [packed (condp = @protocol
                 :mavlink1 (encode-mavlink1 channel message-map)
                 :mavlink2 (encode-mavlink2 channel message-map)
                 (throw (ex-info "Bad protocol"
                                 {:cause :bad-protocol
                                  :protocol @protocol})))]
    (when packed
      (swap! statistics #(assoc % :messages-encoded (inc (:messages-encoded %)))))
    packed))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Start decode state machine state functions.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(declare start-state)

(defn decode-mavlink1
  [{:keys [decode-sm decode-message-info 
           ^ByteBuffer input-buffer mavlink statistics] :as channel}]
  (let [{:keys [crc-seed payload-size msg-key decode-fns]} @decode-message-info
        last-idx ^long (+ payload-size MAVLINK1-HDR-SIZE)
        checksum-lsb (byte-to-long (.get input-buffer last-idx))
        checksum (bit-or (bit-and checksum-lsb 0xff)
                         (bit-and (bit-shift-left (.get input-buffer (inc last-idx)) 8) 0xff00))
        checksum-calc (compute-checksum input-buffer 1 last-idx crc-seed)]
    (if (== checksum checksum-calc)
      (do
        (.position input-buffer MAVLINK1-HDR-SIZE)
        ; decode the message, restart the decode state machine, then
        ; save the message and return it!
        (let [message (apply merge
                             {:message-id msg-key
                              :sequence-id (byte-to-long (new Long (.get input-buffer 2)))
                              :system-id (byte-to-long (new Long (.get input-buffer 3)))
                              :component-id (byte-to-long (new Long (.get input-buffer 4)))}
                             (map #(% input-buffer) decode-fns))]
          (swap! statistics assoc :messages-decoded (inc (:messages-decoded @statistics)))
          (reset! decode-sm start-state)
          message))
      (do
        (swap! statistics assoc :bad-checksums (inc (:bad-checksums @statistics)))
        (reset! decode-sm start-state)
        nil))))

(defn decode-mavlink2
  "Decode the message int he channel's input buffer. It is assumed the message is a
   MAVLink 2 message. If there is a signature, it is assumed the signature has been
   verified and the link id extracted from the signature and passed in. This is because
   if the message was trimmed of trailing zeroes, the zeroes will be written on to the
   end of the message, possibly/probably overwriting the checksum and signature bytes
   before decoding the payload of the message."
  [{:keys [decode-sm decode-message-info 
           ^ByteBuffer input-buffer mavlink statistics] :as channel}
   link-id]
  (let [{:keys [extension-payload-size crc-seed msg-key decode-fns extension-decode-fns]} @decode-message-info
        msg-payload-size (byte-to-long (.get input-buffer 1))
        last-idx ^long (+ msg-payload-size MAVLINK2-HDR-SIZE)
        checksum-lsb (byte-to-long (.get input-buffer (int last-idx)))
        checksum (bit-or (bit-and checksum-lsb 0xff)
                         (bit-and (bit-shift-left (.get input-buffer (inc last-idx)) 8) 0xff00))
        checksum-calc (compute-checksum input-buffer 1 last-idx crc-seed)]
    (if (== checksum checksum-calc)
      (do
        (.position input-buffer (+ MAVLINK2-HDR-SIZE msg-payload-size))
        (when (> extension-payload-size msg-payload-size)
          (doseq [i (range (- extension-payload-size msg-payload-size))]
            (.put input-buffer ^byte ZERO-BYTE)))
        ; decode the message, restart the decode state machine, then
        ; save the message and return it!
        (.position input-buffer MAVLINK2-HDR-SIZE)
        (let [message (apply merge
                             {:message-id msg-key
                              :sequence-id (byte-to-long (new Long (.get input-buffer 4)))
                              :system-id (byte-to-long (new Long (.get input-buffer 5)))
                              :component-id (byte-to-long (new Long (.get input-buffer 6)))
                              :link-id link-id}
                             (map #(% input-buffer) (concat decode-fns extension-decode-fns)))]
          (swap! statistics assoc :messages-decoded (inc (:messages-decoded @statistics)))
          (reset! decode-sm start-state)
          message))
      (do
        (swap! statistics assoc :bad-checksums (inc (:bad-checksums @statistics)))
        (reset! decode-sm start-state)
        nil))))

(defn verify-signature
  "Given an entire packet (the byte array), including the 13 signing bytes,
   the start-signature-idx, and the decoded message, verify the signature of the packet by making
   sure the timetamp is valid (at least one hiher than the last rcv-timestamp and within one minute
   of the last timestamp) and that the first 6 bytes of the sha256 of thepacket mactches the sha256 bytes
   in the packet.

   If the timestamp and the signature are valid, the rcv-timestamp is updated and
   the decoded message is returned, otherwise the statistics are updated and an
   error is thrown. The error is thrown so that the decoding call can catch the error
   and if necessary (such as updating the rcv-timestamp) cab take corrective action. The decoded
   message is included in the thrown error."
  [{:keys [decode-message-info link-id secret-key rcv-timestamp statistics ^ByteBuffer input-buffer] :as channel}
   ^long start-signature-idx]
  (let [packet (.array input-buffer)
        payload-size (byte-to-long (aget packet 1))
        timestamp (let [bb (ByteBuffer/allocate 8)]
                    (.order bb ByteOrder/LITTLE_ENDIAN)
                    (System/arraycopy packet (inc start-signature-idx) (.array bb) 0 6)
                    (.put bb 6 ZERO-BYTE)
                    (.put bb 7 ZERO-BYTE)
                    (.getLong bb))
        start-sha256-idx (+ start-signature-idx 7)]
    (.reset sha256)
    (.update sha256 @secret-key 0 32)
    (.update sha256 packet 0 start-sha256-idx) ; The link-id and timestamps bytes are included
    (if (< @rcv-timestamp timestamp (+ @rcv-timestamp ONE-MINUTE))
      (let [sha256-bytes ^bytes (.digest sha256)
            valid-signature
              (loop [idx start-sha256-idx
                     sidx 0]
                (if (>= sidx 6) 
                  ; if we got through the first 6 bytes, it's valid
                  true
                  (if (not= (aget packet idx) (aget sha256-bytes sidx)) 
                    ; if any byte is invalid immediately return false
                    false
                    ; otherwise go to the next index
                    (recur (inc idx)
                           (inc sidx)))))]
        (if valid-signature
          (do
            (reset! rcv-timestamp timestamp)
            (decode-mavlink2 channel (.get input-buffer start-signature-idx)))
          (do
            (swap! statistics #(assoc % :bad-signatures (inc (:bad-signatures %))))
            (throw (ex-info "Decoding signature sha256 error."
                            {:cause :bad-signature
                             :msg-id (:msg-id @decode-message-info)
                             :timestamp timestamp})))))
      (do
        (swap! statistics #(assoc % :bad-timestamps (inc (:bad-timestamps %))))
        (throw (ex-info "Decoding signature timestamp error."
                        {:cause :bad-timestamp
                         :msg-id (:msg-id @decode-message-info)
                         :rcv-timestamp @rcv-timestamp
                         :timestamp timestamp}))))))

(defn signature-state
  [{:keys [decode-sm ^ByteBuffer input-buffer protocol statistics] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (let [payload-size (byte-to-long (.get input-buffer 1))]
    (when (>= (.position input-buffer) (+ MAVLINK2-HDR-CRC-SIGN-SIZE payload-size))
      ; have all the bytes, always return to start state regardless of signature verification
      (reset! decode-sm start-state)
      (let [start-signature-idx ^long (+ payload-size MAVLINK2-HDR-CRC-SIZE)]
        ; verify-signature will either return the decoded messages or throw an error
        (verify-signature channel start-signature-idx)))))

(defn checksum-msb-state
  "This byte the msb of the checksum. Verify the checksum, if it verifies,
   then decode the message and return it. Use the state map to hold the return message,
   simply dissoc the fields that are not used. In case of error, update the statistics
   and restart the decode state machine.t"
  [{:keys [statistics decode-sm ^ByteBuffer input-buffer accept-unsigned-packets] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (if (= (.get input-buffer 0) MAVLINK2-START-BYTE)
    (if (zero? (bit-and (.get input-buffer 2) IFLAG-SIGNED))
      ; No signature, if accepting unsigned messages then decode and return the message
      ; otherwise drop the message
      (if @accept-unsigned-packets
        (decode-mavlink2 channel nil)
        (do
          (swap! statistics assoc :dropped-unsigned (inc (:dropped-unsigned @statistics)))
          nil))
      ; The message is signed, must get all 13 signature bytes
      (do
        (reset! decode-sm signature-state)
        nil))
    (decode-mavlink1 channel)))

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
  (let [msg-payload-size (byte-to-long (.get input-buffer 1))]
    (when (>= (.position input-buffer) (if (= (.get input-buffer 0) MAVLINK1-START-BYTE)
                                         (+ MAVLINK1-HDR-SIZE msg-payload-size)
                                         (+ MAVLINK2-HDR-SIZE msg-payload-size)))
      (reset! decode-sm checksum-lsb-state)))
  nil)

(defn msg-id-byte3-state
  "Add the byte to the message id, then verify the message, like message-id state
   handling MAVlink 1 protocol."
  [{:keys [decode-sm decode-message-info ^ByteBuffer input-buffer mavlink] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (let [low-byte (byte-to-long (.get input-buffer 7))
        middle-byte (byte-to-long (.get input-buffer 8))
        high-byte (byte-to-long (.get input-buffer 9))
        {:keys [messages-by-id]} mavlink
        message-id (+ (bit-and (bit-shift-left high-byte 16) 0xff0000)
                      (bit-and (bit-shift-left middle-byte 8) 0xff00)
                      (bit-and low-byte 0xff))
        message-map (get messages-by-id message-id)
        msg-payload-size (byte-to-long (.get input-buffer 1))]
    (if (and message-map
             (<= msg-payload-size (:extension-payload-size message-map)))
      (do
        (if (zero? msg-payload-size)
          (reset! decode-sm checksum-lsb-state)
          (reset! decode-sm payload-state))
        (reset! decode-message-info message-map))
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
  [{:keys [decode-sm decode-message-info  ^ByteBuffer input-buffer mavlink] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (let [{:keys [messages-by-id]} mavlink
        message-id (byte-to-long a-byte)
        msg-payload-size (byte-to-long (.get input-buffer 1))]
    (if (= (.get input-buffer 0) MAVLINK1-START-BYTE)
      (let [message-map (get messages-by-id message-id)]
        (if (and message-map
                 (<= msg-payload-size (:payload-size message-map)))
          (do
            (if (zero? msg-payload-size)
              (reset! decode-sm checksum-lsb-state)
              (reset! decode-sm payload-state))
            (reset! decode-message-info message-map))
          (reset! decode-sm start-state)))
      (reset! decode-sm msg-id-byte2-state)))
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

(defn compat-flags-state
  "Save the byte in the compat-flags, then go to sequence id state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm seq-id-state)
  nil)

(defn incompat-flags-state
  "Save the byte in the incompat-flags, then go to compat-flags-state."
  [{:keys [decode-sm ^ByteBuffer input-buffer] :as channel} a-byte]
  (.put input-buffer ^byte a-byte)
  (reset! decode-sm compat-flags-state)
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
  [{:keys [decode-sm decode-message-info ^ByteBuffer input-buffer] :as channel} a-byte]
  (when (or (= a-byte MAVLINK1-START-BYTE)
            (= a-byte MAVLINK2-START-BYTE))
    (reset! decode-sm payload-size-state)
    (reset! decode-message-info nil)
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
    message))

(defn decode-byte-mavlink-no-sm
  "Add a byte to the mavlink input buffer, and if we have a message check the
   checksum. If the checksum matches decode the message, clear the buffer and
   return the message.  If the checksum mis-matches, clear the buffer and
   record the error in the statistics.
   The MAVLink 1 message consists of the following 6 bytes:
     Byte 0 MAVLINK1-START-BYTE
     Byte 1 Payload size
     Byte 2 Sequence id
     Byte 3 System id
     Byte 4 Component id
     Byte 5 Message id"
  [{:keys [mavlink ^ByteBuffer input-buffer statistics] :as channel} a-byte]
  {:pre [input-buffer
         (:messages-by-id mavlink)
         (instance? Byte a-byte)]}
  (swap! statistics assoc :bytes-received (inc (:bytes-received @statistics)))
  (if (zero? (.position input-buffer))
    (when (or (= a-byte MAVLINK1-START-BYTE)
              (= a-byte MAVLINK2-START-BYTE))
      (.put input-buffer ^byte a-byte)
      nil)
  ; else if this is MAVLink 1
  (if (== (.get input-buffer 0) MAVLINK1-START-BYTE)
    (do
      (.put input-buffer ^byte a-byte)
      (if (== (.position input-buffer) MAVLINK1-HDR-SIZE)
        (let [{:keys [messages-by-id]} mavlink
              payload-size (byte-to-long (.get input-buffer 1))
              msg-id (byte-to-long (.get input-buffer 5))
              message-map (get messages-by-id msg-id)]
          (when (or (nil? message-map)
                  (> payload-size (:payload-size message-map)))
            (.clear input-buffer))
          nil)
        (let [payload-size (byte-to-long (.get input-buffer 1))]
          ; when I have the header, payload and checksum, continue
          (when (>= (.position input-buffer) (+ MAVLINK1-HDR-CRC-SIZE payload-size))
            (let [{:keys [messages-by-id]} mavlink
                  msg-id (byte-to-long (.get input-buffer 5))
                  message-map (get messages-by-id msg-id)
                  {:keys [decode-fns msg-key last-value crc-seed]} message-map
                  checksum (let [lsb (.get input-buffer ^int (+ 6 payload-size))
                                 msb (.get input-buffer ^int (+ 7 payload-size))]
                             (bit-or (bit-and lsb 0xff)
                                     (bit-and (bit-shift-left msb 8) 0xff00)))
                  checksum-calc (compute-checksum input-buffer 1 (+ MAVLINK1-HDR-SIZE payload-size) crc-seed)
                  ]
              (.position input-buffer MAVLINK1-HDR-SIZE)
              (if (== checksum checksum-calc)
                (let [message (apply merge {:message-id msg-key
                                            :sequence-id (byte-to-long (.get input-buffer 2))
                                            :system-id (byte-to-long (.get input-buffer 3))
                                            :component-id (byte-to-long (.get input-buffer 4))}
                                           (map #(% input-buffer) decode-fns))]
                  (.clear input-buffer)
                  (swap! statistics assoc :messages-decoded (inc (:messages-decoded @statistics)))
                  message)
                (do
                  (.clear input-buffer)
                  (swap! statistics #(assoc % :bad-checksums (inc (:bad-checksums %))))
                  nil)))))))
  ; else if this is MAVLink 2
  (if (== (.get input-buffer 0) MAVLINK2-START-BYTE)
    (do
      (.put input-buffer ^byte a-byte)
      (if (== (.position input-buffer) MAVLINK2-HDR-SIZE)
        (let [low-byte (byte-to-long (.get input-buffer 7))
              middle-byte (byte-to-long (.get input-buffer 8))
              high-byte (byte-to-long (.get input-buffer 9))
              {:keys [messages-by-id]} mavlink
              msg-id (+ (bit-and (bit-shift-left high-byte 16) 0xff0000)
                        (bit-and (bit-shift-left middle-byte 8) 0xff00)
                        (bit-and low-byte 0xff))
              payload-size (byte-to-long (.get input-buffer 1))
              message-map (get messages-by-id msg-id)]
          (when (or (nil? message-map)
                    (> payload-size (:payload-size message-map)))
            (.clear input-buffer))
          nil)
        (let [payload-size (byte-to-long (.get input-buffer 1))]
          ; when I have the header, payload and checksum, continue
          (when (>= (.position input-buffer) (+ MAVLINK1-HDR-CRC-SIZE payload-size))
            (let [{:keys [messages-by-id]} mavlink
                  msg-id (byte-to-long (.get input-buffer 5))
                  message-map (get messages-by-id msg-id)
                  {:keys [decode-fns msg-key last-value crc-seed]} message-map
                  checksum (let [lsb (.get input-buffer ^int (+ 6 payload-size))
                                 msb (.get input-buffer ^int (+ 7 payload-size))]
                             (bit-or (bit-and lsb 0xff)
                                     (bit-and (bit-shift-left msb 8) 0xff00)))
                  checksum-calc (compute-checksum input-buffer 1 (+ MAVLINK1-HDR-SIZE payload-size) crc-seed)
                  ]
              (.position input-buffer MAVLINK1-HDR-SIZE)
              (if (== checksum checksum-calc)
                (let [message (apply merge {:message-id msg-key
                                            :sequence-id (byte-to-long (.get input-buffer 2))
                                            :system-id (byte-to-long (.get input-buffer 3))
                                            :component-id (byte-to-long (.get input-buffer 4))}
                                           (map #(% input-buffer) decode-fns))]
                  (.clear input-buffer)
                  (swap! statistics assoc :messages-decoded (inc (:messages-decoded @statistics)))
                  message)
                (do
                  (.clear input-buffer)
                  (swap! statistics #(assoc % :bad-checksums (inc (:bad-checksums %))))
                  nil)))))))
      (.clear input-buffer)))))

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
