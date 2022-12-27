(ns mavlink.test_utilities
  (:require [clojure.test :refer :all]
            [clojure.java.io :as io]
            [clojure.core.async :as async]
            [mavlink.checksum :refer :all]
            [mavlink.core :refer :all]
            [mavlink.type :refer :all])
  (:import [com.mavlink CRC]
           [java.io PipedInputStream PipedOutputStream]
           [java.nio ByteBuffer ByteOrder]))

(defn mkbytes
  [^String s]
  (into-array Byte/TYPE (mapv #(.byteValue (Long/valueOf % 16))
                              (clojure.string/split s #" "))))

(defn mk-pipe
  []
  (let [pipe-in (PipedInputStream.)]
    {:pipe-in pipe-in                             ; the read end of the pipe
     :pipe-out (PipedOutputStream. pipe-in) ; the write end of the pipe
     }))

(defn get-test-value
  "Given a type, return a test value specific for that type.
   Note that if it is an unknown type, a simple 0 is returned.
   If the length is given, then generate a full array of test values."
  ([type-key i]
    (condp = type-key
      :char     (char (+ (byte \A) (mod i 26)))
      :int8_t   (bit-and 0xff (+ 5 i))
      :uint8_t  (bit-and 0xff (+ 5 i))
      :uint8_t_mavlink_version  3
      :int16_t  (bit-and 0xffff (+ 17235 (* i 52)))
      :uint16_t (bit-and 0xffff (+ 17235 (* i 52)))
      :int32_t  (bit-and 0xffffffff (+ 963497464 (* i 52)))
      :uint32_t (bit-and 0xffffffff (+ 963497464 (* i 52)))
      :float    (float (+ 17.0 (* i 7)))
      :double   (float (+ 177777.0 (* i 7)))
      :int64_t   9223372036854775807
      :uint64_t  (bigint (new java.math.BigInteger "9223372036854775807"))
      0))
  ([type-key i length]
   (if length
     (if (= type-key :char)
       (reduce str (map #(get-test-value type-key %) (range length)))
       (reduce conj [] (map #(get-test-value type-key %) (range length))))
     (get-test-value type-key i))))

(defn get-test-bitmask
  "Return a bitmask vector for encoding."
  [enum-group]
  (reduce #(let [[enum-value enum-key] %2]
             (conj %1 enum-key))
          []
          (take 3 enum-group)))

(defn- get-test-message 
  "Given a message's specification map, generate a test message-map for it."
  [mavlink {:keys [msg-key fields] :as message-spec}]
  {:pre [msg-key
         (not (empty? fields))]}
  (merge {:message'id msg-key}
         (apply merge (map #(let [{:keys [name-key type-key enum-type length bitmask]} %
                                  enum-group (when enum-type
                                               (enum-type (:enums-by-group mavlink)))]
                              {name-key (if (and bitmask enum-type)
                                          (get-test-bitmask enum-group)
                                          (let [value (get-test-value type-key  5 length)]
                                            (if enum-type
                                              (get enum-group value value)
                                              value)))})
                           fields))))

(defn compare-messages
  "Compare two messages, if they are the same return true, otherwise return false."
  [mavlink sent-msg rcv-msg]
  (let [{:keys [fields ext-fields]} ((:message'id sent-msg) (:messages-by-keyword mavlink))
        all-fields (if ext-fields
                     (concat fields ext-fields)
                     fields)]
    (reduce #(let [{:keys [name-key enum-type bitmask]} %2
                   sent-val (name-key sent-msg)
                   rcv-val (name-key rcv-msg)
                   field-match (if bitmask
                                 (cond
                                   (number? sent-val) (== sent-val (:raw-val rcv-val))
                                   (sequential? sent-val) (== (reduce (fn[result e]
                                                                        (bit-or result (get (:enum-to-value mavlink) e)))
                                                                      0
                                                                      sent-val)
                                                              (:raw-value rcv-val))
                                   :else false)
                                 (if (number? sent-val)
                                   (== sent-val rcv-val)
                                   (= sent-val rcv-val)))]
               (and %1 field-match))
            true
            all-fields)))

; For testing purposes a mavlink connection is used to support both encode and decoding
; of messages connected via a pipe. Encoded messages are written to the outstream side
; of the pipe (the inbound side of the pipe). The messages to decoded are read from
; the instream side of the pipe (the outbound end of the pipe).
;
; To roundtrip a message:
;                         the message to encode is put to the encode-input-channel
;                             (mavlink encodes the message and sends to the encode-output-channel)
;                         the encoded message is taken from the encode-out-channel
;                             (the encoded message is written to the pipe's outstream,
;                              mavlink takes the encoded message from the pipe's instream
;                              which is the decode-input-stream. mavlink decodes the message
;                              and sends it to the decode-output-channel)
;                         the message is taken from the decode-output-channel and returned
;
; Oneway encoding simply puts the message on the encode-input-channel (this is to test 
; message encoding failures).
;
; The mavlink channel maps contains the following fields which must NOT be null:
;  :encode-input           - where messages to encode are taken from
;  :encode-output-link     - where the encoded messages is to be sent
;  :decode-input-stream    - where decoded messages are to be taken from
;  :decode-output-channel  - where the decoed messages are to be sent
;
; The mavlink channel maps contains the following fields which MAY be null:
;  :tlog-stream    - where tlog messages are to be saved
;

(defn open-mavlink-connection
  "Unless otherwise, assume a pipe is to be created as a communication link for
   roundtripping a message (via roundtrip function below) through encoding
   and decoding."
  [mavlink-sources
   {:keys [encode-input-channel encode-output-link decode-input-stream decode-output-channel] :as connection}]
  (let [mavlink (parse mavlink-sources)
        pipe-instream (PipedInputStream.)                 ; the read end of the pipe
        pipe-outstream (PipedOutputStream. pipe-instream) ; the write end of the pipe
        encode-input-channel (or encode-input-channel (async/chan 300))
        encode-output-link (or encode-output-link (async/chan 300))
        decode-input-stream (or decode-input-stream pipe-instream)
        decode-output-channel (or decode-output-channel (async/chan 300))]
    (assoc (open-channel mavlink
                         (assoc connection :encode-input-channel encode-input-channel
                                           :encode-output-link encode-output-link
                                           :decode-input-stream decode-input-stream
                                           :decode-output-channel decode-output-channel))
           :pipe-outstream pipe-outstream
           :mavlink mavlink
           :get-test-message #(get-test-message mavlink %1)
           :compare-messages #(compare-messages mavlink %1 %2)
           :encode-oneway #(async/>!! encode-input-channel %1)
           :encode-roundtrip #(do
                                ; (println "ENCODING roundtrip :") (pprint %1)
                                (async/>!! encode-input-channel %1)
                                (when-let [message-bytes (async/<!! encode-output-link)]
                                   ; (println "The encoded bytes:")
                                   ; (doseq [b message-bytes] (print (str (bit-and 0xff b) " "))) (println)
                                  (.write ^PipedOutputStream pipe-outstream message-bytes 0 (count message-bytes))
                                  (.flush ^PipedOutputStream pipe-outstream)
                                  (async/<!! decode-output-channel))))))

(defn close-mavlink-connection
  [{:keys [close-channel-fn]}]
  (close-channel-fn))
