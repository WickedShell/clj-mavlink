(ns mavlink.checksum
  (:import [java.nio ByteBuffer]))

(defonce ^:const  array-of-bytes-type (Class/forName "[B"))

(defn update-crc
  "Add the next byte to the CRC abd return the result."
  ^long [^long crc next-byte]
  (let [tmp (bit-xor (bit-and next-byte 0xff) (bit-and crc 0xff))
        tmp2 (bit-xor tmp (bit-and (bit-shift-left tmp 4) 0xff))]
    (bit-xor (bit-and (bit-shift-right crc 8) 0xff)
             (bit-shift-left tmp2 8)
             (bit-shift-left tmp2 3)
             (bit-and (bit-shift-right tmp2 4) 0xf))))

(defn- compute-checksum-bytes
  [^bytes the-bytes start-idx last-idx crc-seed]
  (loop [crc 0xffff
         idx start-idx]
    (if (< idx last-idx)
      (recur (update-crc crc (aget the-bytes idx)) (inc idx))
      crc)))

(defn- compute-checksum-byte-buffer
  [^ByteBuffer the-bytes start-idx last-idx crc-seed]
  (loop [crc 0xffff
         idx start-idx]
    (if (< idx last-idx)
      (recur (update-crc crc (.get the-bytes ^int idx)) (inc idx))
      crc)))

(defn compute-checksum
  "Compute the checksum of a string and return it or of
   of a array with an optional magic byte, or part of a byte
   array and an optional magic byte, and then return checksum."
  ([^bytes input] (compute-checksum (ByteBuffer/wrap input) 0 (alength input) nil))
  ([the-bytes crc-seed] (compute-checksum the-bytes 0 (count the-bytes) crc-seed))
  ([the-bytes start-idx last-idx crc-seed]
   (let [crc (if (= array-of-bytes-type (type the-bytes))
               (compute-checksum-bytes the-bytes start-idx last-idx crc-seed)
               (compute-checksum-byte-buffer the-bytes start-idx last-idx crc-seed))]
     (if crc-seed
       (update-crc crc crc-seed)
       crc))))
