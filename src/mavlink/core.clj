(ns mavlink.core
  (:require [mavlink.checksum :refer :all]
            [mavlink.type :refer [byte-to-long]]
            [mavlink.mavlink-xml :refer :all])
  (:import [java.nio ByteBuffer ByteOrder]
           [java.security MessageDigest]
           [java.lang System]))

(defonce INCOMPAT-FLAG-SIGNED 0x01)

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
    :undefined-enum         - A message value uses an unidentified enumerated value
    :bad-checksum           - obviously a bad checksum
  "
  [{:keys [descriptions xml-sources] :as options}]
  {:pre [(pos? (count xml-sources))]}
   (let [parsed (get-xml-zippers xml-sources)]
     (reduce (fn [mavlink source]
                 (add-mavlink mavlink (get-mavlink source options))) {} parsed)))

(defn update-channel
  "Update the MAVLink channel.
   Args is sequence of keyword value pairs (thus the count of args MUST be even)
   where the keyword is one of the following:

   :protocol :mavlink1 or :mavlink2 for encoding
   :secret-key a 32 byte array (the length MUST be 32) or nil to indicate don't sign
   :accept-unsigned-packets true or false
  "
  [{:keys [protocol link-id secret-key accept-unsigned-packets] :as channel}
   & args]
  {:pre [(even? (count args))]}
  (loop [k (first args)
         v (second args)
         rest-args (rest (rest args))]
    (condp = k
      :protocol
        (if (or (= v :mavlink1)
                (= v :mavlink2))
          (reset! protocol v)
          (throw (ex-info "Bad protocol"
                          {:cause :bad-protocol
                           :value v})))
      :link-id
        (if (instance? Long v)
          (reset! link-id (.byteValue (new Long ^long v)))
          (throw (ex-info "Bad Link id"
                          {:cause :bad-link-id
                           :value v})))
      :secret-key
        (if (or (nil? v)
                (>= (count v) 32)) ; see sign-packet, first 32 bytes will be used.
          (reset! secret-key v)
          (throw (ex-info "Bad secret key"
                          {:cause :bad-secret-key
                           :value v})))
      :accept-unsigned-packets (reset! accept-unsigned-packets v)
      (throw (ex-info "Unknown channel key"
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

   See update-channel to update the channel.

   To set-up signing, use update-channel to provide a secret-key.

   The decode function will decode messages based on the type of message, which is determined
   by the message start byte 0xfe for MAVLink 1.0 and 0xfd for MAVLink 2.0.
  
   For MAVLink 2.0:
     For decoding, if the incompat-flags indicate signing, then signing will be verified
     (unless the :accept-unsigned-packets is set, in which case unsigned packets are accepted,
     but signed packets must verify.
     A system id, component id, and link id for each message decoded defines a signing
     tuple. Each signing tuple has a timestamp associated with it. See verify-signing
     for handling the decoded timestamps.

   For encodingMAVLink 2.0:
     The system-id and component-id are taken from the channel, but maybe
     provided in the encode message-map. Note that if signing is active (i.e. the secret-key
     has a value) then the link-id must be given in the message-map, otherwise the default
     value will be used for the link id (see the encode function.)
     Timestamps just indicate forward progression (once a timestamp is seen, ignore
     anything earlier). So, the initial values of the encoding timestamp
     is 0. See the sign-packet function for timestamping for encoding.
   "
  [mavlink {:keys [protocol system-id component-id link-id] :as options}]
  {:pre [(instance? Long system-id)
         (instance? Long component-id)
         (map? mavlink)
         (keyword? protocol)
         (map? (:messages-by-keyword mavlink))
         (map? (:messages-by-id mavlink))
         ]}
   (declare start-state)
  {:mavlink mavlink
   :protocol (atom (or protocol :mavlink1))
   :secret-key (atom nil)               ; MAVLink 2.0 encoding only, if nil not signing
   :encode-timestamp (atom 0)           ; MAVLInk 2.0 encoding only, timestamp of last signed message
   :signing-tuples (atom {})            ; MAVLink 2.0 encoding only, decode timestamps
   :accept-unsigned-packets (atom true) ; MAVLink 2.0 decoding only
   :decode-sha256 (MessageDigest/getInstance "SHA-256")
   :encode-sha256 (MessageDigest/getInstance "SHA-256")
   :system-id system-id
   :component-id component-id
   :link-id (atom (or link-id 0)) ; MAVLink 2.0, encoding only
   :sequence-id-atom (atom 0)
   :decode-sm (atom start-state)
   :decode-message-info (atom nil)      ; internal to decode state machine
   :input-buffer                        ; internal to decoding
     (let [byte-buffer (ByteBuffer/allocate MAX-MESSAGE-SIZE)]
       (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
       byte-buffer)
   :statistics (atom {:bytes-received 0
                      :messages-decoded 0
                      :messages-encoded 0
                      :skipped-encode-sequences 0
                      :bad-checksums 0
                      :bad-signatures 0
                      :bad-timestamps 0
                      :dropped-unsigned 0})
   })

(defn encode-mavlink1
  "Encodes a MAVLink1 message."
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
  (let [{:keys [messages-by-keyword]} mavlink
        {:keys [encode-fns ^long payload-size crc-seed
                ^long msg-id msg-key]} (message-id messages-by-keyword)
        ^long sys-id (or (:system-id message-map) system-id)
        ^long comp-id (or (:component-id message-map) component-id)
        ^long seq-id (if-let [seq-id- (:sequence-id message-map)]
                       (reset! sequence-id-atom (mod seq-id- 256))
                       (swap! sequence-id-atom #(mod (inc %) 256)))
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
      (encode-fn mavlink payload message-map))
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

   signature = sha256(secret_key + header + payload + CRC + link-ID + timestamp)
  "
  [{:keys [secret-key encode-timestamp ^MessageDigest encode-sha256] :as channel} ^bytes packet
   signature-start-idx link-id]
  {:pre [@secret-key]}
  (let [curr-timestamp (get-timestamp)
        sha256-start-idx (+ signature-start-idx 7) ; the packet plus the link id and timestamp
        timestamp-array (let [bb (ByteBuffer/allocate 8)]
                          (.order bb ByteOrder/LITTLE_ENDIAN)
                          (if (> curr-timestamp @encode-timestamp)
                            (reset! encode-timestamp curr-timestamp)
                            (swap! encode-timestamp inc))
                          (.putLong bb ^long @encode-timestamp)
                          (.array bb))]
    ; add link ID to the packet
    (aset-byte packet signature-start-idx link-id) 
    ; add the timestamp to the packet
    (System/arraycopy timestamp-array 0 packet (inc signature-start-idx) 6)
    ; calculate the sha256 from the secret-key and the packet
    (.reset encode-sha256)
    (.update encode-sha256 @secret-key 0 32)
    (.update encode-sha256 packet 0 sha256-start-idx)
    (let [sha256-bytes ^bytes (.digest encode-sha256)]
      ; add the first 6 bytes of the sha256 to the packet
      (System/arraycopy sha256-bytes 0 packet sha256-start-idx 6))))

(defn encode-mavlink2
  "Encodes a MAVLink 2.0 message. If the message is to be signed (the secret-key of
   the channel is not nil, then the link-id must  be specified in the message map,
   otherwise the default value 0 is used.
  "
  ^bytes [{:keys [mavlink statistics system-id component-id sequence-id-atom
                  secret-key link-id] :as channel}
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
  (let [{:keys [messages-by-keyword]} mavlink
        {:keys [encode-fns extension-encode-fns ^long extension-payload-size
                crc-seed ^long msg-id msg-key]} (message-id messages-by-keyword)
        ^long sys-id (or (:system-id message-map) system-id)
        ^long comp-id (or (:component-id message-map) component-id)
        ^long link-id (or (:link-id message-map) @link-id)
        ^long seq-id (if-let [seq-id- (:sequence-id message-map)]
                       (reset! sequence-id-atom (mod seq-id- 256))
                       (swap! sequence-id-atom #(mod (inc %) 256)))
        payload (let [byte-buffer (ByteBuffer/allocate extension-payload-size)]
                  (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
                  byte-buffer)
        incompat-flags (if @secret-key
                         SIGN-PACKETS-FLAG  ; only one possible flag, so no or'ing necessary
                         0)
        compat-flags 0]
    ; encode the payload
    (doseq [encode-fn (concat encode-fns extension-encode-fns)]
      (encode-fn mavlink payload message-map))
  
    ; trim the message and fix the payload size
    (while (and (pos? (.position payload))
                (zero? (.get payload (dec (.position payload)))))
      (.position payload (dec (.position payload))))

    ; size of byte array now known, so can create it and fill it in
    (let [trimmed-payload-size (.position payload)
          packed (byte-array (+ trimmed-payload-size
                                (if @secret-key
                                  MAVLINK2-HDR-CRC-SIGN-SIZE
                                  MAVLINK2-HDR-CRC-SIZE)))]
      (aset-byte packed 0 MAVLINK2-START-BYTE)
      (aset-byte packed 1 (byte-to-long (.byteValue (new Long trimmed-payload-size))))
      (aset-byte packed 2 (.byteValue (new Long incompat-flags)))
      (aset-byte packed 3 (.byteValue (new Long compat-flags)))
      (aset-byte packed 4 (.byteValue (new Long seq-id)))
      (aset-byte packed 5 (.byteValue (new Long sys-id)))
      (aset-byte packed 6 (.byteValue (new Long comp-id)))
      (aset-byte packed 7 (.byteValue (new Long (bit-and msg-id 0xff))))
      (aset-byte packed 8 (.byteValue (new Long (bit-and (bit-shift-right msg-id 8) 0xff))))
      (aset-byte packed 9 (.byteValue (new Long (bit-and (bit-shift-right msg-id 16) 0xff))))

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
                     link-id))
      packed)))

(defn encode
  "Encodes a message map returning a byte array, suitable for sending over the wire.
   The system-id, component-id and sequence-id may all be specified in the message map;
   if specified in the message map, the values will override the default values. If the
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
;; Decode state machine state functions.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(declare start-state)

(defn decode-mavlink1
  "Decode a MAVLink 1.0 message in the channel's input-buffer. Return a message
   map of the decoded message."
  [{:keys [decode-sm decode-message-info 
           ^ByteBuffer input-buffer mavlink statistics] :as channel}]
  (let [{:keys [payload-size msg-key decode-fns]} @decode-message-info]
    ; position the buffer to the start of the payload
    (.position input-buffer MAVLINK1-HDR-SIZE)
    ; decode the message, restart the decode state machine, then
    ; save the message and return it!
    (let [message (persistent! (reduce (fn [message decode-fn] (decode-fn input-buffer message))
                                       (transient {:message-id msg-key
                                                   :sequence-id (byte-to-long (new Long (.get input-buffer 2)))
                                                   :system-id (byte-to-long (new Long (.get input-buffer 3)))
                                                   :component-id (byte-to-long (new Long (.get input-buffer 4)))})
                                       decode-fns))]
      (swap! statistics assoc :messages-decoded (inc (:messages-decoded @statistics)))
      message)))

(defn decode-mavlink2
  "Decode a MAVLink 2.0 message in the channel's input buffer.  If there is a
   signature, it is assumed the signature has been verified and the link id
   extracted from the signature and passed in. This is because if the message was
   trimmed of trailing zeroes, the zeroes will be written on to the end of the
   message, possibly/probably overwriting the checksum and signature bytes
   before decoding the payload of the message."
  [{:keys [decode-sm decode-message-info 
           ^ByteBuffer input-buffer mavlink statistics] :as channel}
   link-id]
  (let [{:keys [extension-payload-size msg-key decode-fns extension-decode-fns]} @decode-message-info
        msg-payload-size (byte-to-long (.get input-buffer 1))]
    ; position the buffer to the end of the payload
    (.position input-buffer (+ MAVLINK2-HDR-SIZE msg-payload-size))
    ; replace trimmed bytes
    (when (> extension-payload-size msg-payload-size)
      (doseq [i (range (- extension-payload-size msg-payload-size))]
        (.put input-buffer (byte 0))))
    ; position the buffer at the start of the payload
    (.position input-buffer MAVLINK2-HDR-SIZE)
    ; decode the message, restart the decode state machine, then
    ; save the message and return it!
    (let [message (persistent! (reduce (fn [message decode-fn] (decode-fn input-buffer message))
                                       (transient {:message-id msg-key
                                                   :sequence-id (byte-to-long (new Long (.get input-buffer 4)))
                                                   :system-id (byte-to-long (new Long (.get input-buffer 5)))
                                                   :component-id (byte-to-long (new Long (.get input-buffer 6)))
                                                   :link-id link-id})
                                       (concat decode-fns extension-decode-fns)))]
      (swap! statistics assoc :messages-decoded (inc (:messages-decoded @statistics)))
      message)))

(defn verify-signature
  "verify the signature of the MVLink 2.0 message in the channel's input-buffer.
   The start-signature-idx is the index of the first byte of the signature, which is
   also the length of the message including the CRC bytes. Verify the signature of the
   packet by making sure the timetamp is valid (at least one hiher than the last
   timestamp for the signing tuple and within one minute of the last timestamp)
   and that the first 6 bytes of the sha256 of the packet matches the sha256 bytes
   in the packet.

   If the timestamp and the signature are valid, the signing tuple timestamp is updated and
   the decoded message is returned, otherwise the statistics are updated and an
   error is thrown. The error is thrown so that the decoding call can catch the error
   The decoded message is included in the thrown error."
  [{:keys [decode-message-info secret-key statistics accept-unsigned-packets
           ^ByteBuffer input-buffer signing-tuples ^MessageDigest decode-sha256] :as channel}
   ^long start-signature-idx]
  {:pre [@secret-key]}
  (let [packet (.array input-buffer)
        tuple (sequence [(.get input-buffer 5)                      ; system id
                         (.get input-buffer 6)                      ; component id
                         (.get input-buffer start-signature-idx)])  ; link id
        tuple-timestamp (get @signing-tuples tuple)
        payload-size (byte-to-long (aget packet 1))
        timestamp (let [bb (ByteBuffer/allocate 8)]
                    (.order bb ByteOrder/LITTLE_ENDIAN)
                    (System/arraycopy packet (inc start-signature-idx) (.array bb) 0 6)
                    (.put bb 6 0)
                    (.put bb 7 0)
                    (.getLong bb))
        start-sha256-idx (+ start-signature-idx 7)]
    (.reset decode-sha256)
    (.update decode-sha256 @secret-key 0 32)
    (.update decode-sha256 packet 0 start-sha256-idx) ; The link-id and timestamps bytes are included
    (if (or (nil? tuple-timestamp)
            (< tuple-timestamp timestamp (+ tuple-timestamp ONE-MINUTE)))
      (let [sha256-bytes ^bytes (.digest decode-sha256)
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
        (if (or valid-signature @accept-unsigned-packets)
          (do
            (when valid-signature
              (swap! signing-tuples assoc tuple timestamp))
            ; FIXME: if timestamp is larger then encode-timestamp, reset encode-timestamp to timestamp
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
                         :signing-tuple tuple
                         :tuple-timestamp tuple-timestamp
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
   then decode and return MAVLink 1 messages and MAVLink 2 messages that are not
   signed. If the message is a signed MAVLink 2 message, then fetch the 13 bytes
   by moving to the signature-state.
  "
  [{:keys [statistics decode-sm ^ByteBuffer input-buffer accept-unsigned-packets
           decode-message-info] :as channel} a-byte]
  (reset! decode-sm start-state)
  (let [{:keys [crc-seed]} @decode-message-info
        last-idx (dec (.position input-buffer))
        checksum-lsb (byte-to-long (.get input-buffer last-idx))
        checksum (bit-or (bit-and checksum-lsb 0xff)
                         (bit-and (bit-shift-left a-byte 8) 0xff00))
        checksum-calc (compute-checksum input-buffer 1 last-idx crc-seed)]
    (if (== checksum checksum-calc)
      (if (= (.get input-buffer 0) MAVLINK2-START-BYTE)
        (if (zero? (bit-and (.get input-buffer 2) INCOMPAT-FLAG-SIGNED))
          ; No signature, if accepting unsigned messages then decode and return the message
          ; otherwise drop the message
          (if @accept-unsigned-packets
            (decode-mavlink2 channel nil)
            (do
              (swap! statistics assoc :dropped-unsigned (inc (:dropped-unsigned @statistics)))
              nil))
          ; The message is signed, go to signature-state to get signature
          (do
            (.put input-buffer ^byte a-byte)    ; put the MSB of the checksum
            (reset! decode-sm signature-state)
            nil))
        (decode-mavlink1 channel))
      (do
        (swap! statistics assoc :bad-checksums (inc (:bad-checksums @statistics)))
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
  [{:keys [statistics decode-sm] :as channel} a-byte]
  {:pre [(instance? Byte a-byte)
         (fn? @decode-sm)]}
  (swap! statistics assoc :bytes-received (inc (:bytes-received @statistics)))
  (@decode-sm channel a-byte))

(defn decode-bytes
  "Decodes a MAVLink message from a byte array.
   This function can be called with either a byte array or a single byte.
   A vector is returned that contains all the messages which were succesfully decoded."
  ([channel ^bytes some-bytes]
   {:pre [(= array-of-bytes-type (type some-bytes))]}
   (decode-bytes channel some-bytes (alength some-bytes)))
  ([channel ^bytes some-bytes num-bytes]
   {:pre [(= array-of-bytes-type (type some-bytes))]}
   (persistent! (reduce (fn [messages idx] 
               (if-let [message (decode-byte channel (aget some-bytes idx))]
                 (conj! messages message)
                 messages))
          (transient []) (range num-bytes)))))

(defn get-description
  "Return the description, only useful if descriptions were saved.
   Otherwise nil is returned."
  [{:keys [descriptions] :as mavlink} msg-key]
  (when descriptions
    (msg-key descriptions)))
