(ns mavlink.type
  (:import [java.nio ByteBuffer])
  (:gen-class))

;; type priority for placing fields in the MAVLINK packet. Equal priority
;; fields are placed in the order they are specified in the XML file.
(defn get-type-priority
 [type-key]
  (condp = type-key
    :uint64_t 1
    :int64_t 1
    :double 1
    :uint32_t 2
    :int32_t 2
    :float 2
    :uint16_t 4
    :int16_t 4
    :uint8_t 5
    :uint8_t_mavlink_version 5
    :int8_t 5
    :char 5
    (throw (ex-info "Unknown priority type"
                    {:cause :unknown-type
                     :type type-key}))))

;; number of bytes for each type.
(defonce type-size {:uint64_t 8
                    :int64_t 8
                    :double 8
                    :uint32_t 4
                    :int32_t 4
                    :float 4
                    :uint16_t 2
                    :int16_t 2
                    :uint8_t 1
                    :uint8_t_mavlink_version 1
                    :int8_t 1
                    :char 1})
 
(defonce UINT64-MAX-VALUE (.toBigInteger 18446744073709551615N))

;; Function to read a typed value from a ByteBuffer
(defonce read-payload
  (hash-map
    :char     (fn [^ByteBuffer rdr] (char (.get rdr)))
    :int8_t   (fn [^ByteBuffer rdr] (.get rdr))
    :uint8_t  (fn [^ByteBuffer rdr] (bit-and (short (.get rdr)) 0xFF))
    :uint8_t_mavlink_version  (fn [^ByteBuffer rdr] (bit-and (short (.get rdr)) 0xFF))
    :int16_t  (fn [^ByteBuffer rdr] (.getShort rdr))
    :uint16_t (fn [^ByteBuffer rdr] (bit-and (int (.getShort rdr)) 0xFFFF))
    :int32_t  (fn [^ByteBuffer rdr] (.getInt rdr))
    :uint32_t (fn [^ByteBuffer rdr] (bit-and (long (.getInt rdr)) 0xFFFFFFFF))
    :float    (fn [^ByteBuffer rdr] (.getFloat rdr))
    :double   (fn [^ByteBuffer rdr] (.getDouble rdr))
    :int64_t  (fn [^ByteBuffer rdr] (.getLong rdr))
    :uint64_t (fn [^ByteBuffer rdr] (let [uint64-bytes (byte-array 8)]
                                       (doseq [idx (range 7 -1 -1)]
                                         (aset uint64-bytes idx (.get rdr)))
                                       (bigint (.and (new java.math.BigInteger uint64-bytes)
                                                     UINT64-MAX-VALUE )))))) 

;; Function to write a typed value to a ByteBuffer
(defonce write-payload
  (hash-map
    :char     (fn [^ByteBuffer wrtr value] (.put wrtr (byte value)))
    :int8_t   (fn [^ByteBuffer wrtr value] (.put wrtr (byte value)))
    :uint8_t  (fn [^ByteBuffer wrtr value] (.put wrtr (.byteValue (new Long (long value)))))
    :uint8_t_mavlink_version  (fn [^ByteBuffer wrtr value] (.put wrtr (.byteValue (new Long (long value)))))
    :int16_t  (fn [^ByteBuffer wrtr value] (.putShort wrtr (short value)))
    :uint16_t (fn [^ByteBuffer wrtr value] (.putShort wrtr (.shortValue (new Long (long value)))))
    :int32_t  (fn [^ByteBuffer wrtr value] (.putInt wrtr (int value)))
    :uint32_t (fn [^ByteBuffer wrtr value] (.putInt wrtr (.intValue (new Long (long value)))))
    :float    (fn [^ByteBuffer wrtr value] (.putFloat wrtr (float value)))
    :double   (fn [^ByteBuffer wrtr value] (.putDouble wrtr (double value)))
    :int64_t  (fn [^ByteBuffer wrtr value] (.putLong wrtr (long value)))
    :uint64_t (fn [^ByteBuffer wrtr value] (.putLong wrtr (.longValue (bigint value))))))

(defn get-default-value-fn
  "Given a type, return the default value for that type cast to the correct type.
   Note that if it is an unknown type, a simple 0 is returned."
  [type-key mavlink-version]
  (condp = type-key
    :char     \o000
    :int8_t   (byte 0)
    :uint8_t  (byte 0)
    :uint8_t_mavlink_version  (byte mavlink-version)
    :int16_t  (short 0)
    :uint16_t (short 0)
    :int32_t  (int 0)
    :uint32_t (int 0)
    :float    (float 0)
    :double   (double 0)
    :int64_t  (long 0)
    :uint64_t (long 0)
    (throw (ex-info "Unknown default value type"
                     {:cause :unknown-type
                      :type type-key}))))
(def get-default-value (memoize get-default-value-fn))

(defn get-default-array
  "Given a type, return the default value for that type cast to the correct type.
   Note that if it is an unknown type, a simple 0 is returned."
  [type-key length]
  (condp = type-key
    :char     (char-array length \o000)
    :int8_t   (byte-array length (byte 0))
    :uint8_t  (byte-array length (byte 0))
    :int16_t  (short-array length (short 0))
    :uint16_t (short-array length (short 0))
    :int32_t  (int-array length (int 0))
    :uint32_t (int-array length (int 0))
    :float    (float-array length (float 0))
    :double   (double-array length (float 0))
    :int64_t  (long-array length (long 0))
    :uint64_t (long-array length (long 0))
    (throw (ex-info "Unknown default array type"
                     {:cause :unknown-type
                      :type type-key}))))

(defn byte-to-long
  "Accept a byte, return it as a long."
  [b]
  (bit-and (long b) 0xff))
