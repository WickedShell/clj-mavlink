(ns mavlink.core)

(defn load-spec
  "Load a mavlink spec from a sequence of XML message definition files, returns a spec map"
  [xml-sequence]
  {})

(defn encode-packet
  "Encodes a packet to a byte array, suitable for sending over the wire"
  ^bytes [spec packet]
  )

(defn decode-byte
  "Reconstructs a MAVLink packet byte.
  
  Returns a map containg the valid packet if a packet was decoded, otherwise returns nil"
  [spec data])
