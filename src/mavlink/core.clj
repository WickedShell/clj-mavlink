(ns mavlink.core
  (:require [clojure.core.async :as async]
            [mavlink.checksum :refer :all]
            [mavlink.type :refer [byte-to-long]]
            [mavlink.mavlink-xml :refer :all])
  (:import [java.io InputStream OutputStream]
           [java.nio ByteBuffer ByteOrder]
           [java.security MessageDigest]
           [java.lang System]))

(defonce ^:const INCOMPAT-FLAG-SIGNED 0x01)

(def ^:const MAVLINK1-START-VALUE 254)
(defonce MAVLINK1-START-BYTE (.byteValue (new Long MAVLINK1-START-VALUE)))
(def ^:const MAVLINK1-HDR-SIZE 6)
(def ^:const MAVLINK1-HDR-CRC-SIZE 8)

(def ^:const MAVLINK2-START-VALUE 253)
(defonce MAVLINK2-START-BYTE (.byteValue (new Long MAVLINK2-START-VALUE)))
(def ^:const MAVLINK2-HDR-SIZE 10)
(def ^:const MAVLINK2-HDR-CRC-SIZE 12)
(def ^:const MAVLINK2-HDR-CRC-SIGN-SIZE 25)
(def ^:const MAVLINK2-SIGN-SIZE 13)
(def ^:const SIGN-PACKETS-FLAG 0x1)

(defonce ^:const BUFFER_SIZE (+ MAVLINK2-HDR-CRC-SIGN-SIZE 256))
(def ^:const ONE-MINUTE 6000000)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Encode support functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn- encode-mavlink1
  "Encodes a MAVLink1 message.

     channel - the internal channel map
     sequencce-id - the sequence id to use in the message
     message - the message map as received from the application
     message-info - the mavlink message information
   "
  ^bytes [{:keys [mavlink system-id component-id ] :as channel}
	  ^long sequence-id
          message
          message-info]
  {:pre [(<= 0 (:msg-id message-info) 255)   ; mavlink 1.0 only
         (instance? Long system-id)
         (instance? Long component-id)
         ]}
  (let [{:keys [encode-fns ^long payload-size crc-seed
                ^long msg-id msg-key]} message-info
        ^long sys-id (or (:system-id message-info) system-id)
        ^long comp-id (or (:component-id message-info) component-id)
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
      (encode-fn mavlink payload message-info))
    ; now copy the array from the payload to the packed array.
    (System/arraycopy (.array payload) 0 packed MAVLINK1-HDR-SIZE (.position payload))
    ; finally calculate and put the checksum in, lsb first.
    (let [checksum (compute-checksum packed 1 (+ MAVLINK1-HDR-SIZE payload-size) crc-seed)]
      (aset-byte packed (+ MAVLINK1-HDR-SIZE payload-size) (.byteValue (new Long (bit-and checksum 0xff))))
      (aset-byte packed (+ 1 MAVLINK1-HDR-SIZE payload-size) (.byteValue (new Long (bit-and (bit-shift-right checksum 8) 0xff)))))
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

     channel - the internal channel map
     packet - the bytes of the packet (with uninitialized signing bytes)
     secret-key - the secret-key to sign the packet with
     encodesha256 - the MessageDigest for signing the packet
     signature-start-idx - the index of the start of the signature in the packet
     link-id - the link id to use in the signature
  "
  [{:keys [encode-timestamp] :as channel}
   ^bytes packet
   secret-key
   ^MessageDigest encode-sha256
   signature-start-idx
   link-id]
  (let [curr-timestamp (quot (System/nanoTime) 10000)  ; get the current timestamp
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
    (.update encode-sha256 secret-key 0 32)
    (.update encode-sha256 packet 0 sha256-start-idx)
    (let [sha256-bytes ^bytes (.digest encode-sha256)]
      ; add the first 6 bytes of the sha256 to the packet
      (System/arraycopy sha256-bytes 0 packet sha256-start-idx 6))))

(defn- encode-mavlink2
  "Encodes a MAVLink 2.0 message. The caller determines whether the message is to be signed.
   If the message is not to be signed, the secret-key should be nil.
   If the message is to be signed, the secret-key is not nil, then the link-id must
   be specified in the message map, otherwise the default value 0 is used.

   channel - the internal channel map
   sequencce-id - the sequence id to use in the message
   secret-key - the secret-key to sign the packets with (or nil if not signing packets)
   encode-sha256 - the MessageDigest for signing encoded messages
   message - the message map as received from the application
   message-info - the mavlink message information
  "
  ^bytes [{:keys [mavlink system-id component-id link-id] :as channel}
	  sequence-id
	  secret-key
	  ^MessageDigest encode-sha256
          message
	  message-info]
  {:pre [(<= 0 (:msg-id message-info) 16777215)
         (instance? Long system-id)
         (instance? Long component-id)
         ]}
  (let [{:keys [encode-fns extension-encode-fns ^long extension-payload-size
                crc-seed ^long msg-id msg-key]} message-info
        ^long sys-id (or (:system-id message) system-id)
        ^long comp-id (or (:component-id message) component-id)
        ^long link-id (or (:link-id message) link-id)
        payload (let [byte-buffer (ByteBuffer/allocate extension-payload-size)]
                  (.order byte-buffer ByteOrder/LITTLE_ENDIAN)
                  byte-buffer)
        incompat-flags (if secret-key
                         SIGN-PACKETS-FLAG  ; only one possible flag, so no or'ing necessary
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
      (aset-byte packed 1 (byte-to-long (.byteValue (new Long trimmed-payload-size))))
      (aset-byte packed 2 (.byteValue (new Long incompat-flags)))
      (aset-byte packed 3 (.byteValue (new Long compat-flags)))
      (aset-byte packed 4 (.byteValue (new Long ^long sequence-id)))
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
      (when secret-key
        (sign-packet channel
                     packed
		     secret-key
		     encode-sha256
                     (+ MAVLINK2-HDR-CRC-SIZE trimmed-payload-size)
                     link-id))
      packed)))

(defn- encode-messages
  "Encodes messages received from the input-channel.
   The system-id, component-id and sequence-id may all be specified in the message;
   if specified in the message, the values will override the default values. If the
   the sequence-id is specified, in addition to overriding the default sequence-id, the
   volatile  used to generate the default sequence-id is set to this value.
   Loops continuously until the channel is closed (a nil is returned from the channel).
   Encoded messages will be written to the output link; if the output link is a stream
   the bytes will be writtenn to the stream, otherwise it is assumed the link is a channel
   and the byte array will be written to the channel.

   channel - the internal channel map
   input-channel - a clojure channel to take messages from
   output-link - the stream to write the encoded bytes to or
                 a clojue channel to put the messages to
   "
  ^bytes [{:keys [mavlink continue encode-protocol signing-options statistics] :as channel}
          input-channel
	  output-link]
  {:pre [(instance? clojure.lang.Atom statistics)
         ]}
  (let [link-is-stream (instance? java.io.OutputStream)
        encode-sha256 (MessageDigest/getInstance "SHA-256")
        sequence-id (volatile! 0)] 
    (loop [message (async/<!! input-channel)]
      ; return normally if continue
      (when @continue
	(when message
	  ; not shutting down and message to encode received
	  ; look up the message-info based on the :messae-id of the message
	  (if-let [message-info ((:message-id message) (:messages-by-keyword mavlink))]
	    (do
	     ; update the sequence id then encode the message
	      (if-let [seq-id- (:sequence-id message-info)]
		 (vreset! sequence-id (mod seq-id- 256))
		 (vswap! sequence-id #(mod (inc %) 256)))
	      (if-let [packed (case (or (:mavlink-protocol message)
				        @encode-protocol))
				  :mavlink1
				    (encode-mavlink1 channel @sequence-id message message-info)
				  :mavlink1-mavlink2
				    (encode-mavlink1 channel @sequence-id message message-info)
				  :mavlink2-signed
				    ; FIXME if supposed to be signing, but not key, will go unsigned
				    ; or should it be dropped?
				    (encode-mavlink2 channel
						     @sequence-id
						     (:secret-key signing-options)
						     encode-sha256 message message-info)
				  :mavlink2-unsigned
				    (encode-mavlink2 channel @sequence-id nil encode-sha256 message message-info)
				    )]
	        ; message successfully encoded, update statistics and send it out
	        (do
		  (swap! statistics update-in [:messages-encoded] inc)
		  (if link-is-stream
		    (.write ^OutputStream output-link ^bytes packed)
		    (async/>!! output-link packed)))
		; message failed to encode due to error in encode function
	        (swap! statistics update-in [:encode-failed] inc)))
	     ; message failed to encode because invalid :message-id
	     (swap! statistics update-in [:encode-failed] inc))
	  (recur (async/<!! input-channel)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Decode state machine support functions.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(declare start-state)

(defn- decode-mavlink1
  "Decode a MAVLink 1.0 message in the channel's buffer Return a message
   map of the decoded message.

   Message-info - the mavlink message information to use to decode the message
   buffer - the buffer to decode
   statistics - the statistics atom
   "
  [message
   message-info
   ^ByteBuffer buffer
   statistics]
  (let [{:keys [msg-key decode-fns]} message-info]
    ; position the buffer to the start of the payload
    (.position buffer MAVLINK1-HDR-SIZE)
    ; decode the message, restart the decode state machine, then
    ; save the message and return it!
    (let [message (persistent! (reduce (fn [message decode-fn] (decode-fn buffer message))
				       (transient message)
                                       decode-fns))]
      (swap! statistics update-in [:messages-decoded] inc)
      message)))

(defn- decode-mavlink2
  "Decode a MAVLink 2.0 message in the channel's input buffer.  If there is a
   signature, it is assumed the signature has been verified and the link id
   extracted from the signature and passed in. This is because if the message was
   trimmed of trailing zeroes, the zeroes will be written on to the end of the
   message, possibly/probably overwriting the checksum and signature bytes
   before decoding the payload of the message.

   It is assumed that the buffer is large enough to hold the bytes the trailing
   zero bytes of the message when it was encoded. The bytes are added back before
   decoding begins.

   Message-info - the mavlink message information to use to decode the message
   buffer - the buffer to decode
   msg-payload-sie - the payload size of the message
   statistics - the statistics atom
   signed-message - whether the message was signed
   "
  [message message-info ^ByteBuffer buffer msg-payload-size statistics signed-message]
  (let [{:keys [extension-payload-size msg-key decode-fns extension-decode-fns]} message-info]
    ; position the buffer to the end of the payload
    (.position buffer (+ MAVLINK2-HDR-SIZE msg-payload-size))
    ; replace trimmed bytes
    (when (> extension-payload-size msg-payload-size)
      (doseq [i (range (- extension-payload-size msg-payload-size))]
        (.put buffer (byte 0))))
    ; position the buffer at the start of the payload
    (.position buffer MAVLINK2-HDR-SIZE)
    ; decode the message, restart the decode state machine, then
    ; save the message and return it!
    (let [message (persistent! (reduce (fn [message decode-fn] (decode-fn buffer message))
				       (transient (assoc! message :mavlink-signed-message
						                  signed-message}))
                                       (concat decode-fns extension-decode-fns)))]
      (swap! statistics update-in [:messages-decoded] inc)
      message)))

 (defn- try-secret-key
   "The try to  match the signature with the given secret-key, return true if it
    matches or return nil if it doesn't match.
    
    decode-sha256 - The MessageDigest to use to try decrypting a the packet's signature
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
  (.update decode-sha256 @secret-key 0 32)
  (.update decode-sha256 packet 0 start-sha256-idx) ; The link-id and timestamps bytes are included
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

(defn- verify-unsigned-packet
  "Call the application provided unsigned-packet-handler to verify an unsigned-packet
   handler - the unsigned packet handler
   buffer - the buffer holding all the message bytes
   msg-key - the message id
   payload-size - the payload size (needed to calculate the link id location
  "
  [handler
   ^ByteBuffer buffer
   message-info
   payload-size]
  (when handler
    (when-let [message (decode-mavlink2 message-info
			                buffer
					payload-size
					statistics
					link-id)]
      (handler message))))

(defn- verify-signature
  "Verify the signature of the MVLink 2.0 message in the buffer.
   The start-signature-idx is the index of the first byte of the signature.
   Verify the signature of the packet by making sure the timetamp is valid
   (at least one higher than the last timestamp for the signing tuple and
    within one minute of the last timestamp)
   and that the first 6 bytes of the sha256 of the packet matches the sha256 bytes
   in the packet.

   If the timestamp and the signature are valid, the signing tuple timestamp is updated and
   true is returned. Otherwise the statistics are updated and false is returned.
   
   channel - internal mavlink channel map
   buffer - buffer holding bytes of the message
   payload-size - the size of the payload of the message in the buffer
   unsigned-packets-handler - handler for unsigned packets, returns true if
                              packet should be accepted.
   start-signature-idx - the start of the signature of the message in the buffer
   statistics - the statistics
   "
  ^Boolean
  [{:keys [signing-options
           signing-tuples
	   ^MessageDigest decode-sha256
           encode-protocol
	   encode-timestamp] :as channel}
    ^ByteBuffer buffer
    payload-size
    start-signature-idx
    message-info
    statistics]
  (let [{:keys [secret-key secret-keyset]} signing-options
        packet (.array buffer)
        tuple (sequence [(.get buffer 5)                      ; system id
                         (.get buffer 6)                      ; component id
                         (.get buffer ^long start-signature-idx)])  ; link id
        tuple-timestamp (get @signing-tuples tuple)
        timestamp (let [bb (ByteBuffer/allocate 8)]
                    (.order bb ByteOrder/LITTLE_ENDIAN)
                    (System/arraycopy packet (inc start-signature-idx) (.array bb) 0 6)
                    (.put bb 6 0)
                    (.put bb 7 0)
                    (.getLong bb))
        start-sha256-idx (+ start-signature-idx 7)]
    (if (or (nil? tuple-timestamp)
            (< tuple-timestamp timestamp (+ tuple-timestamp ONE-MINUTE)))
	(let [curr-key (when (and @secret-key
			        (try-secret-key decode-sha256
					        @secret-key
					        packet
					        start-sha256-idx))
			 @secret-key)
	      valid-key (or curr-key
			    (loop [key-to-try (first secret-keyset)
				   rest-keys (rest secret-keyset)]
			      (when key-to-try
				(if (try-secret-key decode-sha256
						    key-to-try
						    packet
						    start-sha256-idx)
				  key-to-try
				  (recur (first rest-keys)
					 (rest rest-keys))))))]
	  ; if the current secret-key is invalid (or there wasn't one) reset
	  ; the secret-key to the valid-key. This will cause encoding to start
	  ; using the new key in the next signing operation, or if it is nil
	  ; that will cause encoding to stop signing messages. (If a message is in
	  ; the middle of encoding it may or may not be signed correctly, however
	  ; the next message will be signed correctly.)
	  (when-not curr-key
	    (swap! signing-options assoc :secret-key valid-key)
	    (when (and valid-key
		       (not= @encode-protocol :mavlink2-signed))
              (swap! encode-protocol :mavlink2-signed)))
	  (if valid-key
	    (do
              (swap! signing-tuples assoc tuple timestamp)
               (if (> timestamp @encode-timestamp)
                 (reset! encode-timestamp timestamp)
                 (swap! encode-timestamp inc))
              true)
	    (do
	      (swap! statistics update-in [:bad-signatures] inc)
	      false)))
      (do
        (swap! statistics update-in [:bad-timestamps] inc)
	false))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; decode state machine state functions. (started via trampoline in open-channel)
;;   Each state will return a function to handle the next state or
;;   nil if the state machine should stop normally.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn- read-bytes
  "Read the indicated number of bytes from the stream into the indicated buffer.
   Returns true is the number of bytes requested were added to the buffer.

   If the InputStream read operation throws an excception, that exception will be
   caught by the open channel decode thread call (i.e. see open-channel for excception
   handling).
   
   statistics - the statistics
   input-stream - the stream to read the btes from
   buffer - the buffer to place the bytes (the position must be correct before the call)
   num-bytes - the number of bytes to read
   "
  ^Boolean [statistics
            ^Input-Stream input-stream
	    ^ByteByffer buffer
	    num-bytes]
  (if-let [buffer-array (.array buffer)]
    (loop [num-bytes-read (.read buffer-array (.position buffer) num-bytes)]
      (swap! statistics update-in [:bytes-read] #(+ num-bytes-read %))
      (if (neg? num-bytes-read) ; end of stream has been reached
        false
	(if (== num-bytes num-bytes-read)
	  true
	  (recur (+ num-bytes-read
		  (.read buffer-array (.position buffer) (- num-bytes num-bytes-read)))))))
    false))

(defn- verify-checksum
  "Given a buffer, the crc seed, and the position of lsb of checksum in the buffer, extract the
   lsb and msb of the checksum, compute the checksum on the buffer and return whether
   the checksum matches. The msb of the checksum always follows the lsb of the checksum.
   
   buffer - buffer to check CRC bytes
   crc-seed - CRC seed for calculating the checksum (specific to the message type)
   lsb-idx - the index of the LSB of the CRC (the MSB follows the LSB)
   "
  ^Boolean [buffer crc-seed lsb-idx]
  (let [checksum-lsb (byte-to-long (.get input-buffer lsb-idx))
        checksum-msb (byte-to-long (.get input-buffer (inc lsb-idx)))
	checksum (bit-or (bit-and checksum-lsb 0xff)
			 (bit-and (bit-shift-left checksum-msb 8) 0xff00))
	checksum-calc (compute-checksum buffer 1 (dec lsb-idx) crc-seed)]
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
  [{:keys [unsigned-packet-handler signing-options decode-protocol] :as channel}
   ^ByteBuffer buffer
   payload-size
   ^java.io.Inputstream input-stream
   output-channel
   message
   message-info
   statistics]
  (let [signed-message (not (zero? (bit-and (.get input-buffer 2) INCOMPAT-FLAG-SIGNED)))
        bytes-to-read (if signed-messages
			; read payload, CRC, and the signature
	                (+ payload-size 2 MAVLINK2-SIGN-SIZE)
			; read only the payload and CRC
	                (+ payload-size 2))
        bytes-in-message (+ MAVLINK2-HDR-SIZE bytes-to-read)]
    (when (read-bytes statistics input-stream buffer bytes-to-read)
      (if (verify-checksum buffer
			   (:crc-seed message-info)
			   (+ MAVLINK2-HDR-SIZE payload-size))  ; compute the checksum LSB
        ; if okay to decode
	(if (if signed-message
	      (verify-signature channel buffer payload-size start-signature-idx message-info)
	      (verify-unsigned-packet unsigned-packet-handler buffer message))
	  ; okay to decode, decode the message
	  (if-let [message (decode-mavlink2 message-info
					    buffer
					    payload-size
					    statistics
					    signed-message)]
	      (do
		(swap! statistics update-in [:bytes-decoded] #(+ bytes-in-message %))
		(async/>!! output-channel message))
	      ; not okay to decode, so update the dropped mavlink2
	      (when-not signed-message
		(swap! statistics update-in [:bad-mavlink2] inc)))
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
   ^java.io.Inputstream input-stream
   output-channel
   statistics]
  (when (read-bytes statistics input-stream buffer MAVLINK2-HDR-SIZE)
    ; now verify the header bytes
    (let [low-byte (byte-to-long (.get buffer 7))
	  middle-byte (byte-to-long (.get buffer 8))
	  high-byte (byte-to-long (.get buffer 9))
	  {:keys [messages-by-id]} mavlink
	  msg-id (+ (bit-and (bit-shift-left high-byte 16) 0xff0000)
		    (bit-and (bit-shift-left middle-byte 8) 0xff00)
		    (bit-and low-byte 0xff))
	  message-info (get messages-by-id message-id)
	  msg-payload-size (byte-to-long (.get buffer 1))]
      ; select and then return function to execute the next state
      (if (and message-info
	       (<= msg-payload-size (:extension-payload-size message-info)))
	#(mavlink2-payload-state channel
	                         buffer
				 msg-payload-size
				 input-stream
				 output-channel 
				 {:message-id (:msg-key message-info)
				  :sequence-id (byte-to-long (new Long (.get buffer 4)))
				  :system-id (byte-to-long (new Long (.get buffer 5)))
				  :component-id (byte-to-long (new Long (.get buffer 6)))}
				 message-info
				 statistics)
	#(start-state channel buffer input-stream output-channel statistics)))))

(defn- mavlink1-payload-state
  "Decode Mavlink 1 payload state, get the Mavlink1 payload bytes and CRC bytes.
   Verify the CRC, decode the message and put the decoded message into output channel.
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
  [{:keys [encode-protocol] :as channel}
   ^ByteBuffer buffer
   payload-size
   ^java.io.Inputstream input-stream
   output-channel
   message
   message-info
   statistics]
  (let [bytes-to-read (+ payload-size 2)]
    (when (read-bytes statistics input-stream buffer (+ payload-size 2))
      (let [{:keys [crc-seed]} message-info]
	(if (verify-checksum buffer
			     (:crc-seed message-info) 
			     (+ MAVLINK1-HDR-SIZE payload-size)) ; compute checksum LSB
	  (when-not (= @encode-protocol :mavlink2-signed)
	    (async/>!! output-channel
	               (decode-mavlink1 message
					message-info
					buffer
					statistics))
	    (swap! statistics update-in [:bytes-decoded] #(+ % (MAVLINK1-HDR-SIZE bytes-to-read))))
	  (swap! statistics update-in [:bad-checksums] inc))
	; always return function to execute start-state
	#(start-state channel buffer input-stream output-chanel statistics)))))

(defn- mavlink1-header-state
  "Decode Mavlink 1 header state, get the Mavlink1 header bytes, then verify the
   bytes are appropriate for a Mavlink1 header, and return the function to
   execute next.
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
   ^java.io.Inputstream input-stream
   output-channel
  statistics]
  (when (read-bytes statistics input-stream buffer MAVLINK1-HDR-SIZE)
    ; now verify the header bytes
    (let [msg-id (.get buffer 5)
	  msg-payload-size (.get buffer 1)
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
			         {:message-id (:msg-key message-info)
				  :sequence-id (byte-to-long (new Long (.get buffer 2)))
				  :system-id (byte-to-long (new Long (.get buffer 3)))
				  :component-id (byte-to-long (new Long (.get buffer 4)))}
				 message-info
				 statistics)
	#(start-state channel buffer input-stream output-channel statistics)))))

(defn- start-state
  "Decode start state, looking for start byte for either MAVlink 1 or MAVlink 2.
   Ignore every other byte. Continue getting bytes until either a nil byte
   is returned, which indicates the stream was closed, or a start-byte is
   returned. When the stream is closed, just return nil which stops the decoding
   state machine. Otherwise, return a function to execute the function to get
   the header of the message (see clojure trampline documentation.

   channel - internal mavlink channel
   buffer - buffer to hold message to decode
   input-stream - stream to get bytes to decode
   output-channel - channel to write decoded messages to
   statistics statistics
   "
  [{:keys [continue] :as channel}
   ^ByteBuffer buffer
   ^java.io.Inputstream input-stream
   output-channel
   statistics]
  (.clear buffer)
  ; return nil and stop the decode state machine "normally"
  (when (and @continue
             (read-bytes statistics input-stream buffer 1))
    ; return function to select and execute the next state
    #(case (.get input-buffer 0)]
       MAVLINK1-START-BYTE (mavlink1-header-state channel buffer
			    input-stream output-channel statistics)
       MAVLINK2-START-BYTE (mavlink2-header-state channel buffer
			    input-stream output-channel statistics)
       (start-state channel buffer input-stream output-channel statistics))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; End decode state machine state functions.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Public functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn get-description
  "Return the description, only useful if descriptions were saved.
   Otherwise nil is returned."
  [{:keys [descriptions] :as mavlink} msg-key]
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
                               :xml-source (-> test/resources/test-parse.xml io/input-stream)}]
                :descriptions true}

  Possible :cause failures from ExceptionInfo exceptions:
    :bad-checksum           - obviously a bad checksum
    :enum-conflicts         - there is a name conflict in the enumerated types
    :message-id-conflicts   - there is a conflict with the message id values in an XML file
    :message-name-conflicts - there is a conflict witht he message names in an XML file
    :missing-xml-include    - an XML file is included, but no source was identified for this file
    :missing-xml-file-id    - an XML source is missing an XML file indicator
    :no-read-fn             - the type is missing a read function
    :no-write-fn            - the type is missing a write function
    :null-pointer           - obviously a null pointer
    :string-not-number      - string conversion to a number failed usually due to non numeric characters
    :undefined-enum         - A message value uses an unidentified enumerated value
    :unknown-type           - unknown type specifier in an XML file
  "
  [{:keys [descriptions xml-sources] :as options}]
  {:pre [(pos? (count xml-sources))]}
   (let [parsed (get-xml-zippers xml-sources)]
     (reduce (fn [mavlink source]
                 (add-mavlink mavlink (get-mavlink source options))) {} parsed)))

(defn open-channel
  "Given a mavlink (the result of parse), and the open channel options
   an encode and a decode thread are started. A map is returned with
   the following bindings:
     :statistics - the atom the encode/decode threads will update with 
                   encoding/decoding statistics
     :close-channel - a function with no arguments to call to close the channel,
                      in other words to stop the encoding/decoding threads.
  
   mavlink - is the mavlink map returned by a call to parse.
   options - is a hashmap of channel options
     accept-unsigned-packets-handler- a function to all if unsigned pckets aren't accepted
                                      and an unsigned packet comes in, this functio returns
				      true or false if the packet should be accpeted.
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
                             as the sole argument. 
     link-id               - the encode link id, if not given and protocol 
                             is :mavlink2, then 0 is used
     decode-protocol       - the decode protocol to use
                             :mavlink1 - decode mavlink1 ignore mavlink2 messages
			     :mavlink2-signed - decode mavlink2 using signing options
			     :mavlink2-unsigned - decode mavlink2 accept signed
			                          and unsigned packets
			     :mavlink1-:mavlink2 - accept mavlink1 messages until a
			                           mavlink2 message is received. When
						   a mavlink2 message is recieved
						 *** If it is unsigned, swap to protocol
						   :mavlink2-unsigned for both decoding
						   and encoding.
						 *** If it is signed and a valid secret-key
						   is found to decrypt the signature, update
						   the signing-options to use this key for
						   decoding and encoding; swap both the encode
						   and decode protocols to :mavlink2-signed
						 *** If it is signed and no valid secret-key
						   is found to decrypt the signature, drop
						   the message and don't change the protocol
     encode-protocol       - the encode protocol to use
                             :mavlink1 - start and remain Mavlink1 encoding
			                 regardless of decoding
			     :mavlink2-unsigned - start and stay Mavlink2 encoding;
			                          don't sign the messages.
			     :mavlink2-signed - start and stay Mavlink2 encoding;
			                 If there is no secret-key in the signing-options,
					 then encode unsigned.
			     :mavlink1-:mavlink2 -
			                 start encoding messages as Mavlink1.
					 When a Mavlink2 message is decoded
					 then swap encoding to match decoding
     signing-options {
	 secret-key              - The current secret-key to use to encode/decode
			  	   The secret-key can be set on open-channel.
			 	   When signed messages are decoded, the secret-key
			 	   is set to the key that was used to validate the
			 	   signature. If signed message cannot be validated
			 	   with any keys, then the secret-key is set to
			 	   nil and the encoded messages will still be Mavlink2,
			 	   but they will not be signed.
	 secret-keyset           - a sequable collection of valid keys
				   while decoding, if the currnet secret-key fails to
				   validate the message, all the keys will be tried in
				   order. If no valid key is found the secret-key is not
				   updated, if a valid key is found the secret-key is updated.
         unsigned-packet-handler - a function to all if unsigned pckets aren't accepted
                                   and an unsigned packet comes in, this functio returns
				   true or false if the packet should be accpeted.
         }
     system-id             - the encode system id

   The decode function will decode messages based on the type of message, which is determined
   by the message start byte 0xfe for MAVLink 1.0 and 0xfd for MAVLink 2.0.
  
   For MAVLink 2.0:
     For decoding, if the incompat-flags indicate signing, then signing will be verified
     (unless the :accept-unsigned-packets is set, in which case unsigned packets are accepted,)
     FIXME MICHAEL - if the packet is signed, but this flag is set, do we verify any of the signature (e.g. the timestamp), verfy the signature, or ignore the signature? Current code is verifying the timestamp but not the encrypted signature bytes.
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
  [mavlink {:keys [accept-unsigned-packets
                   accept-unsigned-packets-handler
                   component-id
                   decode-input-stream
		   decode-output-channel
                   encode-input-channel
		   encode-output-link
		   exception-handler
		   link-id
                   decode-protocol
                   encode-protocol
		   signing-options
                   system-id] :as options}]
  {:pre [(instance? Long system-id)
         (instance? Long component-id)
         (instance? java.io.InputStream decode-input-stream)
         (or (instance? java.io.OutputStream encode-output-link)
             encode-output-link)
         decode-output-channel
         encode-input-channel
         (map? mavlink)
         (keyword? decode-protocol)
         (keyword? encode-protocol)
         (map? (:messages-by-keyword mavlink))
         (map? (:messages-by-id mavlink))
         ]}
  (let [buffer (ByteBuffer/allocate BUFFER-SIZE) ; start with a buffer size bigger then is possible
	;; the goal is to return the statistics atom ONLY; although internally the channel can carry
	;; needed info in it's map try to get as much as possible into function call arguments
	;; leave the rest in the internal channel map
        statistics (atom {:bytes-read 0
			  :bytes-decoded
			  :messages-decoded 0
			  :messages-encoded 0
			  :encode-failed 0
			  :skipped-encode-sequences 0
			  :bad-checksums 0
			  :bad-signatures 0
			  :bad-timestamps 0
			  :dropped-bad-mavlink2 0})
	continue (atom true)
        channel {:accept-unsigned-packets (atom accept-unsigned-packets) ; MAVLink 2.0 decoding only
	         :accept-unsigned-packets-handler accept-unsigned-packets-handler
		 :component-id component-id
		 :decode-sha256 (MessageDigest/getInstance "SHA-256")
		 :encode-timestamp (atom 0)           ; MAVLInk 2.0 encoding and decoding
		 :link-id (or link-id 0)       ; MAVLink 2.0, encoding only
	         :mavlink mavlink		      ; returned by parse
		 :decode-protocol (atom decode-protocol)
		 :encode-protocol (atom encode-protocol)
	         :continue continue	              ; encode/decode thread continue flag
		 :signing-options (atom signing-options)
		 :signing-tuples (atom {})            ; MAVLink 2.0 decoding only, decode timestamps
		 :statistics statistics
		 :system-id system-id
		 }
        shutdown-fn (fn[e] ; This function can be called by the application
			   ; or internally by the channel threads in case of an error
			   ; or internally or when a thread's input source closes
			   ; this function sets the shitdown flag to true; the threads
			   ; will stop when they poll the continue flag
			   ; NOTE the applicatio is responsible for managing the input and
			   ; output sources. Thus it is not enough to call the returned
			   ; close function, the applicaiton must also close the input/output
			   ; sources.
		      ; set the continue atom to false cause the encode and decode threads to stop
		      (reset! continue false)
		      ; when there is an exception, either call the provided exception handler
		      ; or throw the exception.
		      (when e
                          (if exception-handler
                            (exception-handler e)
                            (throw e)))))
  ]
     (.order buffer ByteOrder/LITTLE_ENDIAN)

     (async/thread    ; decoding thread
       (try
         (trampoline start-state channel
				 buffer
				 decode-input-stream
				 decode-output-channel
				 statistics)
	  (async/>!! decode-output-channel
		     {:message-id :SigningTuples
		      :signing-tuples (:signing-tuples channel)})
	  (shutdown-fn nil)
        (catch Exception e (shutdown-fn (ex-info "clj-mavlink decode error"
						 {:cause :decode}
						 e)))))

     (async/thread    ; encoding thread
       (try
         (encode-messages channel
			  encode-input-channel
			  encode-output-link)
          (shutdown-fn nil)
        (catch Exception e (shutdown-fn (ex-info "clj-mavlink encode error"
						 {:cause :encode}
						 e)))))

     ; Return a map holding the statistics atom and the close-channel function
     {:statistics statistics
      :close-channel #(shutdown-fn nil)}))
