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

(use 'clojure.pprint)

(defn mkbytes
  [^String s]
  (into-array Byte/TYPE (mapv #(.byteValue (Long/valueOf % 16))
                              (clojure.string/split s #" "))))

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

(defn compare-messages
  "Compare two messages, pprint them if they are different and return false,
   otherwise return true."
  [mavlink sent-msg rcv-msg]
  (let [{:keys [fields ext-fields]} ((:message-id sent-msg) (:messages-by-keyword mavlink))
        all-fields (if ext-fields
                     (concat fields ext-fields)
                     fields)]
    (reduce #(let [{:keys [name-key enum-type bitmask]} %2
                   sent-val (name-key sent-msg)
                   rcv-val (name-key rcv-msg)
                   field-match (if bitmask
                                 (if (number? sent-val)
                                   (== sent-val (:raw-val rcv-val))
                                   (loop [bit (first sent-val)
                                          rest-bits (rest sent-val)]
                                     (if (nil? bit)
                                       true
                                       (if-not ((:bit-set? rcv-val) bit)
                                         false
                                         (recur (first rest-bits)
                                                (rest rest-bits))))))
                                 (if (number? sent-val)
                                   (== sent-val rcv-val)
                                   (= sent-val rcv-val)))]
               (or %1 field-match))
            true
            all-fields)))

(defn mk-pipe
  []
  (let [pipe-in (PipedInputStream.)
        pipe-out (PipedOutputStream. pipe-in)]
    {:pipe-in pipe-in
     :pipe-out pipe-out}))

