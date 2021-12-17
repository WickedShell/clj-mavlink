(ns mavlink.core
  (:require [clojure.core.async :as async]
            [mavlink.checksum :refer :all]
            [mavlink.type :refer [byte-to-long]]
            [mavlink.mavlink_xml :refer :all])
  (:import [java.io InputStream OutputStream DataOutputStream IOException]
           [java.nio ByteBuffer ByteOrder]
           [java.security MessageDigest]
           [java.lang System]))

(defonce ^:const INCOMPAT-FLAG-SIGNED 0x01)

(defonce ^:const MAVLINK1-START-VALUE 254)
(defonce MAVLINK1-START-BYTE (.byteValue (new Long MAVLINK1-START-VALUE)))
(defonce ^:const MAVLINK1-HDR-SIZE 6)
(defonce ^:const MAVLINK1-HDR-CRC-SIZE 8)

(defonce ^:const MAVLINK2-START-VALUE 253)
(defonce MAVLINK2-START-BYTE (.byteValue (new Long MAVLINK2-START-VALUE)))
(defonce ^:const MAVLINK2-HDR-SIZE 10)
(defonce ^:const MAVLINK2-HDR-CRC-SIZE 12)
(defonce ^:const MAVLINK2-HDR-CRC-SIGN-SIZE 25)
(defonce ^:const MAVLINK2-SIGN-SIZE 13)
(defonce ^:const SIGN-PACKETS-FLAG 0x1)

(defonce ^:const BUFFER-SIZE (+ MAVLINK2-HDR-CRC-SIGN-SIZE 256))
(defonce ^:const ONE-MINUTE 6000000)

(defonce ^:const start-bytes #{MAVLINK1-START-BYTE MAVLINK2-START-BYTE})

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Telemetry Log functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro write-tlog
  "Write timestamp and packet to DataOutputStream."
  [tlog packet length timestamp]
  `(do
     (.writeLong ~tlog (or ~timestamp (quot (System/nanoTime) 1000)))
     (.write ~tlog ~packet 0 ~length)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Encode support functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn- encode-mavlink1
  "Encodes a MAVLink1 message.

   channel - the internal channel map
   sequence-id - the sequence id to use in the message
   message - the message map as received from the application
   message-info - the mavlink message information
   "
  ^bytes [{:keys [mavlink system-id component-id]}
          ^long sequence-id
          message
          {:keys [encode-fns ^long payload-size crc-seed
                ^long msg-id]}]
  {:pre [(<= 0 msg-id 255)   ; mavlink 1.0 only
         (instance? Long system-id)
         (instance? Long component-id)
         ]}
  (let [^long sys-id (or (:system'id message) system-id)
        ^long comp-id (or (:component'id message) component-id)
        payload (let [byte-buffer (ByteBuffer/allocate payload-size)]
                  (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
                  byte-buffer)
        packed (byte-array (+ MAVLINK1-HDR-SIZE payload-size 2))]
    (aset-byte packed 0 MAVLINK1-START-BYTE)
    (aset-byte packed 1 (.byteValue (new Long payload-size)))
    (aset-byte packed 2 (.byteValue (new Long sequence-id)))
    (aset-byte packed 3 (.byteValue (new Long sys-id)))
    (aset-byte packed 4 (.byteValue (new Long comp-id)))
    (aset-byte packed 5 (.byteValue (new Long msg-id)))
    (doseq [encode-fn encode-fns]
      (encode-fn mavlink payload message))
    ; now copy the array from the payload to the packed array.
    (System/arraycopy
      (.array payload) 0 packed MAVLINK1-HDR-SIZE (.position payload))
    ; finally calculate and put the checksum in, lsb first.
    (let [checksum (compute-checksum packed 1 (+ MAVLINK1-HDR-SIZE payload-size)
                                     crc-seed)]
      (aset-byte packed (+ MAVLINK1-HDR-SIZE payload-size)
                 (.byteValue (new Long (bit-and checksum 0xff))))
      (aset-byte packed (+ 1 MAVLINK1-HDR-SIZE payload-size)
                 (.byteValue
                   (new Long (bit-and (bit-shift-right checksum 8) 0xff)))))
    packed))

(defn- sign-packet
  "Sign the packet, it is assumed there is a secret-key and
   that the packet array has room for the 13 bytes of the signature:
   the link-id, the 6 bytes of timestamp and the 6 bytes of the signature.

   The timestamp is determined from the system time, if the new
   timestamp is the same as the old, then add add 1.  Timestamps are
   in units of 10 microseconds.

   The link id and the first 6 bytes of the timestamp are appended to the
   packet starting at the signature-idx. Then the signature is calculated
   using SHA256 implemeneted by java.securty.MessageDigest.

   signature = sha256(secret_key + header + payload + CRC + link-ID + timestamp)

   encode-timestamp - the encode timestamp atom
   packet - the bytes of the packet (with uninitialized signing bytes)
   secret-key - the secret-key to sign the packet with
   encodesha256 - the MessageDigest for signing the packet
   signature-start-idx - the index of the start of the signature in the packet
   link-id - the link id to use in the signature
  "
  [encode-timestamp
   ^bytes packet
   secret-key
   ^MessageDigest encode-sha256
   signature-start-idx
   link-id]
  (let [curr-timestamp (quot (System/nanoTime) 10000) ; get the current timestamp
        sha256-start-idx (+ signature-start-idx 7) ; the packet plus the
                                                   ; link id and timestamp
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
    (.update encode-sha256 secret-key 0 32)
    (.update encode-sha256 packet 0 sha256-start-idx)
    (let [sha256-bytes ^bytes (.digest encode-sha256)]
      ; add the first 6 bytes of the sha256 to the packet
      (System/arraycopy sha256-bytes 0 packet sha256-start-idx 6))))

(defn- encode-mavlink2
  "Encodes a MAVLink 2.0 message. The caller determines whether the message
   is to be signed.  If the message is not to be signed, the secret-key should
   be nil.  If the message is to be signed, the secret-key is not nil, then
   the link-id must be specified in the message map, otherwise the default
   value 0 is used.

   channel - the internal channel map
   sequence-id - the sequence id to use in the message
   secret-key - the secret-key holding the key to sign the packets with
   encode-sha256 - the MessageDigest for signing encoded messages
   message - the message map as received from the application
   message-info - the mavlink message information
  "
  ^bytes [{:keys [mavlink system-id component-id
                  link-id encode-timestamp]}
          sequence-id
          secret-key
          ^MessageDigest encode-sha256
          message
          {:keys [encode-fns extension-encode-fns ^long extension-payload-size
                  crc-seed ^long msg-id]}]
  {:pre [(<= 0 msg-id 16777215)
         (instance? Long system-id)
         (instance? Long component-id)
         ]}
  (let [^long sys-id (or (:system'id message) system-id)
        ^long comp-id (or (:component'id message) component-id)
        ^long link-id (or (:link'id message) link-id)
        payload (let [byte-buffer (ByteBuffer/allocate extension-payload-size)]
                  (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
                  byte-buffer)
        incompat-flags (if secret-key
                         SIGN-PACKETS-FLAG  ; only one possible flag,
                                            ; so no or'ing necessary
                         0)
        compat-flags 0]
    ; encode the payload
    (doseq [encode-fn (concat encode-fns extension-encode-fns)]
      (encode-fn mavlink payload message))
  
    ; trim the message and fix the payload size
    (while (and (pos? (.position payload))
                (zero? (.get payload (dec (.position payload)))))
      (.position payload (dec (.position payload))))

    ; size of byte array now known, so can create it and fill it in
    (let [trimmed-payload-size (.position payload)
          packed (byte-array (+ trimmed-payload-size
                                (if secret-key
                                  MAVLINK2-HDR-CRC-SIGN-SIZE
                                  MAVLINK2-HDR-CRC-SIZE)))]
      (aset-byte packed 0 MAVLINK2-START-BYTE)
      (aset-byte packed 1 (.byteValue (new Long trimmed-payload-size)))
      (aset-byte packed 2 (.byteValue (new Long incompat-flags)))
      (aset-byte packed 3 (.byteValue (new Long compat-flags)))
      (aset-byte packed 4 (.byteValue (new Long ^long sequence-id)))
      (aset-byte packed 5 (.byteValue (new Long sys-id)))
      (aset-byte packed 6 (.byteValue (new Long comp-id)))
      (aset-byte packed 7 (.byteValue (new Long (bit-and msg-id 0xff))))
      (aset-byte packed 8
                 (.byteValue
                   (new Long (bit-and (bit-shift-right msg-id 8) 0xff))))
      (aset-byte packed 9
                 (.byteValue
                   (new Long (bit-and (bit-shift-right msg-id 16) 0xff))))

      ; now copy the array from the payload to the packed array.
      (when (pos? trimmed-payload-size)
        (System/arraycopy (.array payload) 0 packed
                          MAVLINK2-HDR-SIZE trimmed-payload-size))

      ; finally calculate and put the checksum in, lsb first.
      (let [checksum (compute-checksum packed 1
                                       (+ MAVLINK2-HDR-SIZE trimmed-payload-size)
                                       crc-seed)]
        (aset-byte packed (+ MAVLINK2-HDR-SIZE trimmed-payload-size)
                   (.byteValue (new Long (bit-and checksum 0xff))))
        (aset-byte packed (+ 1 MAVLINK2-HDR-SIZE trimmed-payload-size)
                   (.byteValue
                     (new Long (bit-and (bit-shift-right checksum 8) 0xff)))))
      ; the packet is ready to go, if there is a secret-key, then the message should be signed
      (when secret-key
        (sign-packet encode-timestamp
                     packed
                     secret-key
                     encode-sha256
                     (+ MAVLINK2-HDR-CRC-SIZE trimmed-payload-size)
                     link-id))
      packed)))

(defn- encode-messages
  "Encodes messages received from the input-channel.
   The system-id, component-id and sequence-id may all be specified in the
   message; if specified in the message, the values will override the default
   values. If the the sequence-id is specified, in addition to overriding
   the default sequence-id, the volatile  used to generate the default
   sequence-id is set to this value.  Loops continuously until the channel
   is closed (a nil is returned from the channel).

   Encoded messages will be written to the output link; if the output link is
   a stream the bytes will be writtenn to the stream, otherwise it is assumed
   the link is a channel and the byte array will be written to the channel.

   Note, the value of the protocol atom and secret-key are set by the
   application in the open-channel function and are then updated by the
   decode thread.

   Once the protocol is MAVlink 2 signed, all outgoing messages are encoded
   as signed MAVlink2 messages. To change back to an earlier protocol, the
   channel must be closed and reopened.

   channel - the internal channel map
   input-channel - a clojure channel to take messages from
   output-link - the stream to write the encoded bytes to or
                 a clojue channel to put the messages to
   "
  ^bytes [{:keys [mavlink continue protocol report-error
                  signing-options statistics ^DataOutputStream tlog-stream] :as channel}
          input-channel
          output-link]
  {:pre [(instance? clojure.lang.Atom statistics)
         ]}
  (let [link-is-stream (instance? OutputStream output-link)
        encode-sha256 (MessageDigest/getInstance "SHA-256")
        {:keys [secret-key]} signing-options
        sequence-id (volatile! 0)] 
    (loop [message (async/<!! input-channel)]
      ; return normally if continue
      (when @continue
        (when message
          (if (= (:message'id message) :clj-mavlink)
            (when-let [{new-protocol :protocol} message]
              (case new-protocol
                :mavlink2 (reset! protocol :mavlink2)
                :mavlink1 (when (= @protocol :mavlink2)
                            (when report-error
                              (report-error (ex-info "clj-mavlink cannot go from protocol MAVLink 2 to MAVLink1"
                                                     {:cause :bad-protocol
                                                      :error :clj-mavlink-protocol
                                                      :message message}))))
                (when report-error
                  (report-error (ex-info "clj-mavlink message specified unknown protocol"
                                         {:cause :bad-protocol
                                          :error :clj-mavlink-protocol
                                          :message message})))))
          ; not shutting down and message to encode received
          ; look up the message-info based on the :message'id of the message
            (if-let [message-info ((:message'id message)
                                   (:messages-by-keyword mavlink))]
              (try
                ; calculate the sequence id then encode the message
                ; don't update the mavlink sequence id until after message is sent
                (let [msg-seq-id (:sequence'id message)
                      new-seq-id (if msg-seq-id
                                   (mod msg-seq-id 256)
                                   (mod (inc @sequence-id) 256))]
                  (if-let [packet (case (or (:protocol' message)
                                            @protocol)
                                      :mavlink1
                                        (if (>= (:msg-id message-info) 256)
                                          (do
                                            (swap! statistics update-in
                                                   [:bad-protocol] inc)
                                            (throw (ex-info "MAVlink 2 message id, current protocol is MAVLink 1"
                                                            {:cause :bad-protocol
                                                             :error :encode-failed
                                                             :message message})))
                                          (encode-mavlink1 channel new-seq-id
                                                           message message-info))
                                      :mavlink2
                                        (encode-mavlink2 channel new-seq-id
                                                         @secret-key encode-sha256
                                                         message message-info)
                                        )]
                    ; message successfully encoded,
                    (do
                      ; write the packet out
                      (if link-is-stream
                        (do
                          (.write ^OutputStream output-link ^bytes packet)
                          (.flush ^OutputStream output-link))
                        (async/>!! output-link packet))
                      ;
                      ;update the statistics
                      (swap! statistics update-in [:messages-encoded] inc)
                      ;
                      ; write the tlog
                      (when tlog-stream
                        (locking tlog-stream
                          (write-tlog tlog-stream packet (count ^bytes packet) nil)))
                      ;
                      ; now update mavlink sequence id
                      (vreset! sequence-id new-seq-id))
                    ; message failed to encode due to error in encode function
                    (do
                      (swap! statistics update-in [:encode-failed] inc)
                      (when report-error
                        (report-error (ex-info "Encoding failed" 
                                               {:cause :encode-failed
                                                :message message}))))))
                (catch Exception e (if report-error
                                     (report-error (if (ex-data e)
                                                     e
                                                     (ex-info "Encoding exception."
                                                              {:cause :encode-failed
                                                               :message message
                                                               :exception e})))
                                     (throw e))))
               ; message failed to encode because invalid :message'id
               (do
                 (swap! statistics update-in [:encode-failed] inc)
                 (when report-error
                   (report-error (ex-info "Encoding failed."
                                          {:cause :invalid-message-id
                                           :error :encode-failed
                                           :message message}))))))
          (recur (async/<!! input-channel)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Decode state machine support functions.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(declare start-state)

(defn update-decode-statistics
  [^long system-id ^long sequence-id statistics]
  (let [{:keys [^longs last-seq-ids ^longs messages-decoded ^longs messages-skipped]} @statistics
        last-sys-seq-id (aget last-seq-ids system-id)
        last-sys-decoded (aget messages-decoded system-id)
        last-sys-skipped (aget messages-skipped system-id)
        difference (- sequence-id (mod (inc last-sys-seq-id) 256))
        skipped (if (neg? last-sys-seq-id)
                  0
                  (if (neg? difference)
                    (+ difference 255)
                    difference))]
    (aset last-seq-ids system-id sequence-id)
    (aset messages-decoded system-id (inc last-sys-decoded))
    (aset messages-skipped system-id (long (+ last-sys-skipped skipped)))))

(defn- decode-mavlink1
  "Decode a MAVLink 1.0 message in the channel's buffer Return a message
   map of the decoded message.

   Message-info - the mavlink message information to use to decode the message
   buffer - the buffer to decode
   statistics - the statistics atom
   "
  [{:keys [system'id sequence'id] :as message}
   decode-fns
   ^ByteBuffer buffer
   statistics]
  ; position the buffer to the start of the payload
  (.position buffer MAVLINK1-HDR-SIZE)
  ; decode the message, restart the decode state machine, then
  ; save the message and return it!
  (let [message (persistent!
                  (reduce (fn [message decode-fn] (decode-fn buffer message))
                          (transient message)
                          decode-fns))]
    (update-decode-statistics system'id sequence'id statistics)
    message))

(defn- decode-mavlink2
  "Decode a MAVLink 2.0 message in the channel's input buffer.  If there is a
   signature, it is assumed the signature has been verified and the link id
   extracted from the signature and passed in. This is because if the message
   was trimmed of trailing zeroes, the zeroes will be written on to the end
   of the message, possibly/probably overwriting the checksum and signature
   bytes before decoding the payload of the message.

   It is assumed that the buffer is large enough to hold the bytes the trailing
   zero bytes of the message when it was encoded. The bytes are added back
   before decoding begins.

   Message-info - the mavlink message information to use to decode the message
   buffer - the buffer to decode
   msg-payload-sie - the payload size of the message
   statistics - the statistics atom
   "
  [{:keys [system'id sequence'id] :as message}
   message-info
   ^ByteBuffer buffer
   msg-payload-size
   statistics
   ]
  (let [{:keys [extension-payload-size decode-fns extension-decode-fns]}
          message-info]
    ; position the buffer to the end of the payload
    (.position buffer (int (+ MAVLINK2-HDR-SIZE msg-payload-size)))
    ; replace trimmed bytes
    (when (> extension-payload-size msg-payload-size)
      (dotimes [_ (- extension-payload-size msg-payload-size)]
        (.put buffer (byte 0))))
    ; position the buffer at the start of the payload
    (.position buffer MAVLINK2-HDR-SIZE)
    ; decode the message, and return it!
    (let [message (persistent! (reduce (fn [message decode-fn]
                                         (decode-fn buffer message))
                                       (transient message)
                                       (concat decode-fns
                                               extension-decode-fns)))]
      (update-decode-statistics system'id sequence'id statistics)
      message)))

 (defn- try-secret-key
   "The try to  match the signature with the given secret-key, return true if it
    matches or return nil if it doesn't match.
    
    decode-sha256 - The MessageDigest to use to try decrypting a the packet's
                    signature
    secret-key - the key to try to use to decrypt the signature
    packet - the packet with the signature to try
    start-sha256-idx - the start index of the sha256 bytes in the signature
    "
   ^Boolean [^MessageDigest decode-sha256
             secret-key
             ^bytes packet
             start-sha256-idx]
  ; reset the MessageDigest
  (.reset decode-sha256)
  (.update decode-sha256 secret-key 0 32)
  (.update decode-sha256 packet 0 start-sha256-idx) ; The link-id and timestamps
                                                    ; bytes are included
  (let [sha256-bytes ^bytes (.digest decode-sha256)]
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
                 (inc sidx)))))))

(defn- verify-signature
  "Verify the signature of the MVLink 2.0 message in the buffer.
   The start-signature-idx is the index of the first byte of the signature.
   Verify the signature of the packet by making sure the timetamp is valid
   (at least one higher than the last timestamp for the signing tuple and
    within one minute of the last timestamp)
   and that the first 6 bytes of the sha256 of the packet matches the sha256
   bytes in the packet.

   If the timestamp and the signature are valid, the signing tuple timestamp
   is updated and true is returned. Otherwise the statistics are updated and
   false is returned.
   
   channel - internal mavlink channel map
   buffer - buffer holding bytes of the message
   payload-size - the size of the payload of the message in the buffer
   unsigned-packets-handler - handler for unsigned packets, returns true if
                              packet should be accepted.
   start-signature-idx - the start of the signature of the message in the buffer
   statistics - the statistics
   "
  ^Boolean
  [secret-key secret-keyset
   signing-tuples
   ^MessageDigest decode-sha256
   encode-timestamp
   ^ByteBuffer buffer payload-size
   start-signature-idx
   message-info
   statistics]
  (let [packet (.array buffer)
        tuple (sequence [(.get buffer 5)                      ; system id
                         (.get buffer 6)                      ; component id
                         (.get buffer ^long start-signature-idx)])  ; link id
        tuple-timestamp (get @signing-tuples tuple)
        timestamp (let [bb (ByteBuffer/allocate 8)]
                    (.order bb ByteOrder/LITTLE_ENDIAN)
                    (System/arraycopy packet
                                      (inc start-signature-idx)
                                      (.array bb) 0 6)
                    (.put bb 6 (byte 0))
                    (.put bb 7 (byte 0))
                    (.getLong bb))
        start-sha256-idx (+ start-signature-idx 7)]
    (if (or (nil? tuple-timestamp)
            (< tuple-timestamp timestamp (+ tuple-timestamp ONE-MINUTE)))
      (let [valid-signature? (or (and @secret-key
                                      (try-secret-key decode-sha256
                                                      @secret-key
                                                      packet
                                                      start-sha256-idx))
                                 (loop [key-to-try (first secret-keyset)
                                        rest-keys (rest secret-keyset)]
                                   (when key-to-try
                                     (if (try-secret-key decode-sha256
                                                         key-to-try
                                                         packet
                                                         start-sha256-idx)
                                       (do
                                         (reset! secret-key key-to-try)
                                         true)
                                       (recur (first rest-keys)
                                              (rest rest-keys))))))]
          (if valid-signature?
            (do ; housekeeping stuff
              (swap! signing-tuples assoc tuple timestamp)
              (if (> timestamp @encode-timestamp)
                (reset! encode-timestamp timestamp)
                (swap! encode-timestamp inc))
              true)
            (do ; bad signature, update statistics
              (swap! statistics update-in [:bad-signatures] inc)
              false)))
      (do
        (swap! statistics update-in [:bad-timestamps] inc)
        false))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; decode state machine state functions. (started via trampoline in open-channel)
;;   Each state will return a function to handle the next state or
;;   nil if the state machine should stop normally.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn- read-bytes
  "Read the indicated number of bytes from the stream into the indicated buffer.
   Returns true is the number of bytes requested were added to the buffer.

   If the InputStream read operation throws an excception, that exception will
   be caught by the open channel decode thread call (i.e. see open-channel for
   exception handling).
   
   statistics - the statistics
   input-stream - the stream to read the btes from
   buffer - the buffer to place the bytes (the position must be correct before
   the call) num-bytes - the number of bytes to read
   "
  ^Boolean [statistics
            ^InputStream input-stream
            ^ByteBuffer buffer
            num-bytes]
  (let [buffer-array (.array buffer)
        num-bytes-read (.read input-stream
                              buffer-array
                              (.position buffer)
                              num-bytes)]
    (if (neg? num-bytes-read) ; end of stream has been reached
      false
      (do
        (swap! statistics update-in [:bytes-read] #(+ num-bytes-read %))
        (.position buffer (+ (.position buffer) num-bytes-read))
        (if (>= num-bytes-read num-bytes)
          true
          (recur statistics input-stream buffer (- num-bytes num-bytes-read)))))))

(defn- verify-checksum
  "Given a buffer, the crc seed, and the position of lsb of checksum in the 
   buffer, extract the lsb and msb of the checksum, compute the checksum on
   the buffer and return whether the checksum matches. The msb of the checksum
   always follows the lsb of the checksum.
   
   buffer - buffer to check CRC bytes
   crc-seed - CRC seed for calculating the checksum
              (specific to the message type)
   lsb-idx - the index of the LSB of the CRC (the MSB follows the LSB)
   "
  ^Boolean [^ByteBuffer buffer crc-seed ^long lsb-idx]
  (let [checksum-lsb (byte-to-long (new Long (.get buffer lsb-idx)))
        checksum-msb (byte-to-long (new Long (.get buffer (inc lsb-idx))))
        checksum (bit-or (bit-and checksum-lsb 0xff)
                         (bit-and (bit-shift-left checksum-msb 8) 0xff00))
        checksum-calc (compute-checksum buffer 1 lsb-idx crc-seed)]
    (== checksum checksum-calc)))

(defn- mavlink2-payload-state
  "Decode Mavlink 2 payload state, get the Mavlink2 payload bytes and CRC bytes.
   Verify the CRC. If the message  is signed, then get and verify the signature.
   Then decode the message and put the decoded message into output channel.
   When the stream is closed, just return nil which stops the decoding
   state machine.
   
   channel - internal mavlink channel
   buffer - buffer holding message to decode
   payload-size - the payload size
   input-stream - stream to get bytes to decode from
   output-channel - the clojure channel to write the decoded message to
   message-info - mavlink message information for message in buffer
   statistics - statistics
   "
  [{:keys [encode-timestamp signing-options protocol ^DataOutputStream tlog-stream] :as channel}
   ^ByteBuffer buffer
   payload-size
   ^InputStream input-stream
   output-channel
   message
   message-info
   statistics]
  (let [{:keys [accept-message-handler decode-sha256 secret-key
                secret-keyset signing-tuples]} signing-options
        signed-message (not
                         (zero? (bit-and (.get buffer 2) INCOMPAT-FLAG-SIGNED)))
        bytes-to-read (if signed-message
                        ; read payload, CRC, and the signature
                        (+ payload-size 2 MAVLINK2-SIGN-SIZE)
                        ; read only the payload and CRC
                        (+ payload-size 2))
        bytes-in-message (+ MAVLINK2-HDR-SIZE bytes-to-read)
        ]
    (when (read-bytes statistics input-stream buffer bytes-to-read)

      (if (verify-checksum buffer
                           (:crc-seed message-info)
                           ; compute the checksum LSB
                           (+ MAVLINK2-HDR-SIZE payload-size))
        (let [signature-verified (when signed-message
                             ; verify-signature counts bad signatures,
                             ; updates the secret-key if it changes, and
                             ; returns whether the signature verified
                             (or (verify-signature secret-key secret-keyset
                                                   signing-tuples
                                                   decode-sha256
                                                   encode-timestamp
                                                   buffer payload-size
                                                   (+ MAVLINK2-HDR-CRC-SIZE
                                                      payload-size)
                                                   message-info statistics)
                                 (when accept-message-handler
                                   (accept-message-handler
                                     (assoc message
                                            :signed'message signed-message
                                            :current'protocol @protocol)))))
              okay-to-decode (case @protocol
                               :mavlink1
                                 (when (or (not signed-message)
                                           (and signed-message
                                                signature-verified))
                                   (reset! protocol :mavlink2))
                               :mavlink2
                                 (if signed-message
                                   signature-verified    ; signed and verified?
                                   (or (not @secret-key) ; okay to not be signed
                                       (do ; should be signed not okay
                                         (swap! statistics update-in
                                                [:unsigned-messages] inc)
                                         false))))]
          ; if okay to decode
          (when okay-to-decode
            ; write telemetry log first (because decode-mavlink2 will replace the
            ; trimmed zero bytes in the buffer, overwriting the packet as received.
            (when tlog-stream
              (locking tlog-stream
                (write-tlog tlog-stream (.array ^ByteBuffer buffer) bytes-in-message (:timestamp' message))))
            ;
            ; decode and output the message
            (async/>!! output-channel
                       (decode-mavlink2 (assoc message
                                               :signed'message signed-message)
                                        message-info
                                        buffer
                                        payload-size
                                        statistics))
            ;
            ; update statistics
            (swap! statistics update-in [:bytes-decoded] #(+ bytes-in-message %))))
        ; update statistics on messages dropped due to bad checksums
        (swap! statistics update-in [:bad-checksums] inc)))

      ; regardless of what happened, go to the start state
      #(start-state channel buffer input-stream output-channel statistics)))

(defn- mavlink2-header-state
  "Decode Mavlink 2 header state, get the Mavlink2 header bytes, then verify the
   bytes are appropriate for a Mavlink2 header, and return the function to
   execute next.
   When the stream is closed, just return nil which stops the decoding
   state machine.
   Header bytes are [start-byte
                     payload-size
                     incompat-flags
                     compat-flags
                     seq id
                     system id
                     component id
                     msg id byte1
                     msg id byte2
                     msg id byte3]

   channel - internal mavlink channel
   buffer - buffer to hold message to decode
   input-stream - stream to get bytes to decode
   output-channel - channel to write decoded messages to
   statistics statistics
   "
  [{:keys [mavlink]  :as channel}
   ^ByteBuffer buffer
   ^InputStream input-stream
   output-channel
   statistics timestamp]
  (when (read-bytes statistics input-stream buffer (dec MAVLINK2-HDR-SIZE))
    ; now verify the header bytes
    (let [low-byte (byte-to-long (new Long (.get buffer 7)))
          middle-byte (byte-to-long (new Long (.get buffer 8)))
          high-byte (byte-to-long (new Long (.get buffer 9)))
          msg-id (+ (bit-and (bit-shift-left high-byte 16) 0xff0000)
                    (bit-and (bit-shift-left middle-byte 8) 0xff00)
                    (bit-and low-byte 0xff))
          message-info (get (:messages-by-id mavlink) msg-id)]
      ; select and then return function to execute the next state
      (if message-info
        #(mavlink2-payload-state channel
                                 buffer
                                 (byte-to-long (new Long (.get buffer 1)))
                                 input-stream
                                 output-channel 
                                 {:timestamp' timestamp
                                  :message'id (:msg-key message-info)
                                  :protocol' :mavlink2
                                  :sequence'id
                                    (byte-to-long (new Long (.get buffer 4)))
                                  :system'id
                                    (byte-to-long (new Long (.get buffer 5)))
                                  :component'id
                                    (byte-to-long (new Long (.get buffer 6)))}
                                 message-info
                                 statistics)
        #(start-state channel buffer input-stream output-channel statistics)))))

(defn- mavlink1-payload-state
  "Decode Mavlink 1 payload state, get the Mavlink1 payload bytes and CRC bytes.
   Verify the CRC, decode the message and put the decoded message into output
   channel.  When the stream is closed, just return nil which stops the decoding
   state machine.
   
   channel - internal mavlink channel
   buffer - buffer holding message to decode
   payload-size - the payload size
   input-stream - stream to get bytes to decode from
   output-channel - the clojure channel to write the decoded message to
   message-info - mavlink message information for message in buffer
   statistics - statistics
   "
  [{:keys [protocol ^DataOutputStream tlog-stream] :as channel}
   ^ByteBuffer buffer
   payload-size
   ^InputStream input-stream
   output-channel
   message
   message-info
   statistics]
  (let [bytes-to-read (+ payload-size 2)]
    (when (read-bytes statistics input-stream buffer bytes-to-read)
      (if (verify-checksum buffer
                           (:crc-seed message-info)
                           ; compute checksum LSB
                           (+ MAVLINK1-HDR-SIZE payload-size))
        (if (or (= @protocol :mavlink1)
                (and (= @protocol :mavlink2)
                     (when-let [accept-message-handler (:accept-message-handler (:signing-options channel))]
                       (accept-message-handler (assoc message :current'protocol @protocol)))))
          (do
            ; write telemetry log
            (when tlog-stream
              (locking tlog-stream
                (write-tlog tlog-stream
                            (.array ^ByteBuffer buffer)
                            (+ MAVLINK1-HDR-CRC-SIZE payload-size)
                            (:timestamp' message))))
            ; decode and output the message
            (async/>!! output-channel
                       (decode-mavlink1 message
                                        (:decode-fns message-info)
                                        buffer
                                        statistics))
            ; update statistics
            (swap! statistics update-in
                   [:bytes-decoded] #(+ % MAVLINK1-HDR-SIZE bytes-to-read)))
          (swap! statistics update-in [:bad-protocol] inc))
        (swap! statistics update-in [:bad-checksums] inc))))

  ; always return function to execute start-state
  #(start-state channel buffer input-stream output-channel statistics))

(defn- mavlink1-header-state
  "Decode Mavlink 1 header state, get the Mavlink1 header bytes (remember the
   start-byte has already been read), then verify the bytes are appropriate
   for a Mavlink1 header, and return the function to execute next.
   When the stream is closed, just return nil which stops the decoding
   state machine.
   Header bytes are [start-byte
                     payload-size
                     seq id
                     system id
                     component id
                     msg id]

   channel - internal mavlink channel
   buffer - buffer to hold message to decode
   input-stream - stream to get bytes to decode
   output-channel - channel to write decoded messages to
   statistics statistics
   "
  [{:keys [mavlink]  :as channel}
   ^ByteBuffer buffer
   ^InputStream input-stream
   output-channel
   statistics timestamp]
  (when (read-bytes statistics input-stream buffer (dec MAVLINK1-HDR-SIZE))
    ; now verify the header bytes
    (let [msg-id (byte-to-long (new Long (.get buffer 5)))
          msg-payload-size (byte-to-long (new Long (.get buffer 1)))
          {:keys [messages-by-id]} mavlink
          message-info (get messages-by-id msg-id)]
      ; select state to execute next and return function to execute the state
      (if (and message-info
               (<= msg-payload-size (:payload-size message-info)))
        #(mavlink1-payload-state channel
                                 buffer
                                 msg-payload-size
                                 input-stream
                                 output-channel
                                 {:timestamp' timestamp
                                  :message'id (:msg-key message-info)
                                  :protocol' :mavlink1
                                  :sequence'id
                                    (byte-to-long (new Long (.get buffer 2)))
                                  :system'id
                                    (byte-to-long (new Long (.get buffer 3)))
                                  :component'id
                                    (byte-to-long (new Long (.get buffer 4)))}
                                 message-info
                                 statistics)
        #(start-state channel buffer input-stream output-channel statistics)))))

(defn- start-state
  "Decode start state, looking for start byte for either MAVlink 1 or MAVlink 2.
   Ignore every other byte. Continue getting bytes until either a nil byte
   is returned, which indicates the stream was closed, or a start-byte is
   returned. When the stream is closed, just return nil which stops the decoding
   state machine. Otherwise, return a function to execute the function to get
   the header of the message (see clojure trampline documentation).

   Timestamp every decoded message with the system time, unless the input stream
   is a tlog, in which case get the timestamp from the tlog.

   channel - internal mavlink channel
   buffer - buffer to hold message to decode
   input-stream - stream to get bytes to decode
   output-channel - channel to write decoded messages to
   statistics statistics
   "
  [{:keys [continue input-is-tlog?] :as channel}
   ^ByteBuffer buffer
   ^InputStream input-stream
   output-channel
   statistics]
  (.clear buffer)
  (when-let [timestamp (or 
                         (when input-is-tlog? ; read timestamp and start byte
                           (when (read-bytes statistics input-stream buffer (inc (Long/BYTES)))
                               (loop []
                                 (let [sb (.get buffer (Long/BYTES))]
                                   (if (contains? start-bytes sb)
                                     ; get the timestamp to return, clear the buffer,
                                     ; then put back the start byte
                                     (let [ts (do
                                                (.position buffer 0)
                                                (Long/reverseBytes (.getLong buffer)))]
                                        (.clear buffer)
                                        (.put buffer sb)
                                        ts)
                                     ; didn't get the a start byte, so the timestamp isn't right either
                                     ; shift the buffer and read another byte
                                     (do
                                       (doseq [i (range (Long/BYTES))]
                                         (.put buffer (int i) (.get buffer ^long (inc i))))
                                       (.position buffer (Long/BYTES))
                                       (when (read-bytes statistics input-stream buffer 1)
                                         (recur))))))))
                           (loop [] ; just find the start byte
                             (.position buffer 0)
                             (when (read-bytes statistics input-stream buffer 1)
                               (if (contains? start-bytes (.get buffer 0))
                                 (quot (System/nanoTime) 1000)
                                 (recur)))))]
    ; return nil and stop the decode state machine "normally"
    (when (and @continue
               timestamp)
      ; return function to select and execute the header state
      (condp = (.get buffer 0)
        MAVLINK1-START-BYTE #(mavlink1-header-state channel buffer
                             input-stream output-channel statistics timestamp)
        MAVLINK2-START-BYTE #(mavlink2-header-state channel buffer
                             input-stream output-channel statistics timestamp)
        (when-let [report-error (:report-error channel)]
          (report-error (ex-info "clj-mavlink internal error decoding"
                                 {:cause :decode-failed
                                  :error :start-byte-not-found
                                  :message "Could not find start of a MAVLink message"}))
          nil)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; End decode state machine state functions.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Public functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn get-description
  "Return the description, only useful if descriptions were saved.
   Otherwise nil is returned."
  [{:keys [descriptions]} msg-key]
  (when descriptions
    (msg-key descriptions)))

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
                               :xml-source (-> test/resources/test-parse.xml
                                               io/input-stream)}]
                :descriptions true}

  Possible :cause failures from ExceptionInfo exceptions:
    :bad-checksum           - obviously a bad checksum
    :enum-conflicts         - there is a name conflict in the enumerated types
    :message-id-conflicts   - there is a conflict with the message id values
                              in an XML file
    :message-name-conflicts - there is a conflict witht he message names in
                              an XML file
    :missing-xml-include    - an XML file is included, but no source was
                              identified for this file
    :missing-xml-file-id    - an XML source is missing an XML file indicator
    :no-read-fn             - the type is missing a read function
    :no-write-fn            - the type is missing a write function
    :null-pointer           - obviously a null pointer
    :string-not-number      - string conversion to a number failed usually
                              due to non numeric characters
    :undefined-enum         - A message value uses an unidentified
                              enumerated value
    :unknown-type           - unknown type specifier in an XML file
  "
  [{:keys [xml-sources] :as options}]
  {:pre [(pos? (count xml-sources))]}
   (let [parsed (get-xml-zippers xml-sources)
         mavlink
           (reduce #(add-mavlink-enums %1 (get-mavlink-enums %2))
                   {} parsed)]
     ; The mavlink map holds the merged enum data of all the XML sources
     ; now add on all the message information from the XML sources
     (reduce
       #(add-mavlink-messages %1 (get-mavlink-messages %2 options mavlink))
       mavlink parsed)))

(defn open-channel
  "Given a mavlink (the result of parse), and the open channel options
   an encode and a decode thread are started. A map is returned with
   the following bindings:
     :statistics - the atom the encode/decode threads will update with 
                   encoding/decoding statistics
     :close-channel-fn - a function with no arguments to call to close
                   the channel, in other words to stop the encoding/decoding
                   threads.

  A note about MAVlink message protocol.
     Messages are (en/de)coded based on the current protocol and the value
     of the secret-key.
  
     If the protocol is :mavlink1 then messages are encoded MAVlink 1 and
     all received messages are expected to be MAVlink 1, until a MAVlink 2
     message is received. If the message is successfully decoded,
     this will change the protocol to :mavlink2.

     If the protocol is :mavlink2, then all messages will be encoded MAVlink 2.
     Whether the message is signed or not is controlled by the secret key in the
     signing options. If the key is not nil, the message is signed.

     Once the secret key is set it is never cleared, and only updated when a
     signed MAVlink 2 message's signature is successfully decrypted using a
     different key, then the secret key is updated to the key used to decrypt
     the signature.

     Thus, the MAVlink protocol is either :mavlink1, :mavlink2 without
     signing (because the secret key is nil) or :mavlink2 with signing
     (because the secret key is set). And the protocol can only move forward
     through those 'states'. Thus the application can start using MAVlink 1
     MAVlink 2 signed or unsigned by setting the procotol and secret-key.
     Once running, the decoding process itself will update the protocol
     based on the decoding process. 

     The accept-message-handler provides a method for the application to
     indicate whether or not to accept a message that is MAVlink 1 when the
     current protocol is MAVlink 2. Or the message is unsigned when it should
     be signed. Of it is signed and no key was found to decode it. The handler
     is called with one argument, a message map with the following fields:
                                      :message'id       - from the message
                                      :sequence'id      - from the message
                                      :system'id        - from the message
                                      :component'id     - from the message
                                      :current'protocol - :mavlink1 or :mavlink2
                                      :signed'message   - true or false
     The handler should return true if the message should be accepted,
     false otherwise.

     
   mavlink - is the mavlink map returned by a call to parse.
   options - is a hashmap of channel options
     accept-message-handler- a function to all to ask the application whether
                             to accept a message that it would otherwise drop.
     component-id          - the encode component id
     decode-input-stream   - the stream to read bytes to decode from
     decode-output-channel - the channel to write decoded messages to
     encode-input-channel  - the channel to receive messages to decode on
     encode-output-link    - either an output stream to write the encoded bytes to
                             or a channel to write the encoded byte array to
                             (anything else will cause an exception)
     exception-handler     - exception handler function,
                             if nil the exception is thrown
                             otherwise this function is called with the exception
                             as the sole argument. Exception's generally will
                             be an IException, the ex-data map will have :cause,
                             :message (if the message is known), and :exception.
     link-id               - the encode link id, if not given and protocol 
                             is :mavlink2, then 0 is used
     protocol              - the encode protocol to use
                             :mavlink1 - decode mavlink1 ignore mavlink2 messages
                             :mavlink2 - decode mavlink2 using signing options
     report-error          - function to report non-fatal errors and exceptions,
                             particularly encoding errors, it is passed an IException
                             (see exception -handler for a description of the error
                             data message map bindings.
     signing-options {
         secret-key        - The current secret-key to use to encode, the
                             first key to try when decoding signed messages.
                             The secret-key can be set on open-channel.
                             When signed messages are decoded, the secret-key
                             is set to the key that was used to validate the
                             signature. If signed message cannot be validated
                             with any keys, then the secret-key is not updated.
         secret-keyset     - a sequable collection of valid keys
                             while decoding, if the currnet secret-key fails to
                             validate the message, all the keys will be tried in
                             order. If no valid key is found the secret-key is
                             not updated, if a valid key is found the secret-key is updated.
         accept-message-handler - a function to all if the message header
                                  indicates the message is not of the expected
                                  protocol.
         }
     system-id             - the encode system id
     tlog-stream           - OutputStream to write telemetry log to. Should be nil if
                             no telemetry log is desired.

   For encodingMAVLink 2.0:
     The system-id and component-id are taken from the channel, but maybe
     provided in the encode message-map. Note that if signing is active
     (i.e. the secret-key has a value) then the link-id must be given in the
     message-map, otherwise the default value will be used for the link id
     (see the encode function.)
     Timestamps just indicate forward progression (once a timestamp is seen,
     ignore anything earlier). So, the initial values of the encoding timestamp
     is 0. See the sign-packet function for timestamping for encoding.
   "
  [mavlink {:keys [component-id
                   input-is-tlog?
                   decode-input-stream
                   decode-output-channel
                   encode-input-channel
                   encode-output-link
                   exception-handler
                   link-id
                   protocol
                   report-error
                   signing-options
                   system-id
                   tlog-stream]}]
  {:pre [(instance? Long system-id)
         (instance? Long component-id)
         (instance? InputStream decode-input-stream)
         (or (instance? OutputStream encode-output-link)
             encode-output-link)
         decode-output-channel
         encode-input-channel
         (map? mavlink)
         (keyword? protocol)
         (map? (:messages-by-keyword mavlink))
         (map? (:messages-by-id mavlink))
         (or (nil? tlog-stream)
             (instance? java.io.OutputStream tlog-stream))
         ]}
  ; start with a buffer size bigger then is possible
  (let [buffer (ByteBuffer/allocate BUFFER-SIZE)
        statistics (atom {:bytes-read 0
                          :bytes-decoded 0
                          :messages-decoded (long-array 256 0)
                          :last-seq-ids (long-array 256 -1)
                          :messages-skipped (long-array 256 0)
                          :messages-encoded 0
                          :encode-failed 0
                          :bad-checksums 0
                          :bad-protocol 0
                          :bad-signatures 0
                          :unsigned-messages 0
                          :bad-timestamps 0
                          :bad-mavlink2 0})
        signing-options-map {:accept-message-handler
                               (:accept-message-handler signing-options)
                             :decode-sha256
                               (MessageDigest/getInstance "SHA-256")
                             :secret-key (atom (:secret-key signing-options))
                             :secret-keyset (:secret-keyset signing-options)
                             :signing-tuples (atom {})
                             }
        continue (atom true)
        channel {:component-id component-id
                 :input-is-tlog? input-is-tlog?
                 :encode-timestamp (atom 0) ; MAVLInk 2.0 encoding and decoding
                 :link-id (or link-id 0)    ; MAVLink 2.0, encoding only
                 :mavlink mavlink           ; returned by parse
                 :protocol (atom protocol)
                 :report-error report-error ; function to call to report errors
                 :continue continue         ; encode/decode thread continue flag
                 :signing-options signing-options-map
                 :statistics statistics
                 :system-id system-id
                 :tlog-stream (when tlog-stream
                                (DataOutputStream. tlog-stream))
                 }
        shutdown-fn (fn[e]
                      ; This function can be called by the application
                      ; or internally by the channel threads in case of an error
                      ; or internally or when a thread's input source closes
                      ; this function sets the shutdown flag to true;
                      ; the threads ; will stop when they poll the continue flag
                      ; NOTE the application is responsible for managing the
                      ; input and output sources. Thus it is not enough to
                      ; call the returned close function, the applicaiton
                      ; must also close the input/output sources.
                      (reset! continue false)
                      (when e
                          (if exception-handler
                            (exception-handler e)
                            (throw e))))
        ]
    (.order buffer ByteOrder/LITTLE_ENDIAN)

    (async/thread    ; decoding thread
      (try
        ; start the decoding state machine
        (trampoline start-state channel
                                buffer
                                decode-input-stream
                                decode-output-channel
                                statistics)
        ; when the state machine stops, output the state of the signing tuples
        (async/>!! decode-output-channel
                   {:message'id :SigningTuples
                    :signing-tuples (:signing-tuples channel)})
        ; shutdown the encode thread normally
        (shutdown-fn nil)
        (catch IOException e (shutdown-fn (ex-info "clj-mavlink IOException occurred, probably due to shutdown of the link."
                                                   {:cause :io-exception
                                                    :thread "Mavlink Decode"
                                                    :exception e})))
        (catch Exception e (shutdown-fn (ex-info "clj-mavlink decode thread Exception"
                                                 {:cause :decode
                                                  :exception e})))))

    (async/thread    ; encoding thread
      (try
        ; start encoding messages
        (encode-messages channel
                         encode-input-channel
                         encode-output-link)
        ; shutdown the decode thread normally
        (shutdown-fn nil)
        (catch IOException e (shutdown-fn (ex-info "clj-mavlink IOException occurred, probably due to shutdown of the link."
                                                   {:cause :io-exception
                                                    :thread "Mavlink Encode"
                                                    :exception e})))
        (catch Exception e (shutdown-fn (ex-info "clj-mavlink encode thread Exception"
                                                 {:cause :encode
                                                  :exception e})))))

    ; Return a map holding the statistics atom and the close-channel function
    {:statistics statistics
     :close-channel-fn #(shutdown-fn nil)}))

(defn get-enum
  "Look up value in an enum group and return the enum-key for that value.
   nil in case of error."
  [mavlink group-id v]
  (get-in mavlink [:enums-by-group group-id v]))

(defn get-enum-group
  "Given a group id, return the map of key/values for that group."
  [mavlink group-id]
  (group-id (:enums-by-group mavlink)))

