(ns mavlink.checksum
  (:import [java.nio ByteBuffer]))

(def ^:const CRC-START-VALUE 0xffff)

(defn update-crc
  "Add the next byte to the CRC abd return the result."
  ^long [^long crc next-byte]
  (let [tmp (bit-xor (bit-and next-byte 0xff) (bit-and crc 0xff))
        tmp2 (bit-xor tmp (bit-and (bit-shift-left tmp 4) 0xff))]
    (bit-xor (bit-and (bit-shift-right crc 8) 0xff)
             (bit-shift-left tmp2 8)
             (bit-shift-left tmp2 3)
             (bit-and (bit-shift-right tmp2 4) 0xf))))

(defn compute-checksum
  "Compute the checksum of a string and return it or of
   of a array with an optional magic byte, or part of a byte
   array and an optional magic byte, and then return checksum."
  ([^String s] (compute-checksum (ByteBuffer/wrap (.getBytes s)) 0 (count s) nil))
  ([the-bytes crc-seed] (compute-checksum the-bytes 0 (count the-bytes) crc-seed))
  ([the-bytes start-idx last-idx crc-seed]
   (loop [idx start-idx
          crc CRC-START-VALUE]
     (if (>= idx last-idx)
       (if crc-seed
         (update-crc crc crc-seed)
         crc)
       (recur (inc idx)
              (update-crc crc
                          (if (= (Class/forName "[B") (class the-bytes))
                            (aget ^bytes the-bytes idx)
                            (.get ^ByteBuffer the-bytes ^int idx))))))))
