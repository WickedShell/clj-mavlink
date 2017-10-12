# clj-mavlink

[![Build Status](https://semaphoreci.com/api/v1/wickedshell/clj-mavlink/branches/master/badge.svg)](https://semaphoreci.com/wickedshell/clj-mavlink)

Clojure MAVLink bindings.

## Usage

Add the following to your `project.clj` dependencies:

```
[clj-mavlink "0.1.0"]
```

### Using the library

Pull in the mavlink.core, for example

```
(:require [mavlink.core :as mavlink])
```

`clj-mavlink` builds a mavlink interface from XML MAVLink specifications.
The clj-mavlink interface accepts message hash maps, encodes them, and then
writes the encoded message onto an OutputStream. The clj-mavlink interface reads
an InputStream, decodes messages, and writes the resulting message to a clojure
async channel (see clojure.sore.async).

### Building the clj-mavlink database

The first step is to bring the MAVLink XML files into your project.
`mavlink/parse` takes an xml-sources specification and returns the clj-mavlink database.

For example:
```
(defonce ^:const ardupilotmega-xml "ardupilotmega.xml")
(defonce ^:const common-xml "common.xml")
(defonce ^:const uavionix-xml "uAvionix.xml")

(defonce mavlink-info
         (future
           (mavlink/parse
             {:xml-sources [{:xml-file ardupilotmega-xml
                             :xml-source (-> ardupilotmega-xmls
                                             io/resource io/input-stream)}
                            {:xml-file common-xml
                             :xml-source (-> common-xml
                                             io/resource io/input-stream)}
                            {:xml-file uavionix-xml
                             :xml-source (-> uavionix-xml
                                             io/resource io/input-stream)}]})))

```

The xml-sources is a vector of hash-maps. Each hash map specifies the name of
the source file and the InputStream to read that file. The clj-mavlink database
can be reused any number of times to open mavlink interfaces, i.e. channels, between
the application and the autopilot.

### Opening a channel

`mavlink/open-channel` establishes a communication interface between the
autopilot and the applicaiton. `open-channel` takes a hash-map of options to configure
the channel.

For example this code open a a MAVLink 1 protocol connection, encoded messages
will use the specified system id and component id (unless it is overriden by the
message hash-map), the decode input stream and encode-output-link are defined by
the communication protocol (e.g. a serial link or TCP link). autopilot-send and
autopilot-receive are clojure.core.async channels that the application uses to
send message hash maps to be encoded and to receive decoded messages as message hash-maps.

```
    (reset! mavlink-channel
            (mavlink/open-channel @mavlink-info
                                  {:protocol :mavlink1
                                   :system-id swiftgcs-sysid
                                   :component-id 0
                                   :link-id 0
                                   :decode-input-stream decode-input
                                   :decode-output-channel autopilot-receive
                                   :encode-input-channel autopilot-send
                                   :encode-output-link encode-output
                                   :exception-handler #(log/error "clj-mavlink/test exception:\n" %1)
                                   :signing-options {:secret-key nil ; no signing
                                                     :secret-keyset nil ; secret-keyset
                                                     :accept-message-handler  ; log but don't accept the message
                                                       #(log/error "clj-mavlink/accept-message:\n"
                                                                   %1)
                                                     }}))
```

See the `mavlink/open-channel` doc string for more information on the options.

#### A note about the protocol

The messages will be encoded as specified in the `mavlink/open-channel` options hash-map.
In this example, it encodes as MAVLink 1.0 messages. HOwever, if a MAVLink 2.0 message is
decoded, the encoding will automatically switch to MAVLink 2.0, as if the protocol had
benn specified :mavlink2. Messages will be signed based on whether or not the :secret-key
of the :signing-options is not nil. If a signed MAVLink 2.0 message is decoded, then the
signature is verified by first trying the :secret-key (if it is not nil), if it doesn't verify,
then the keys in the secret-keyset is tried until a match is found, in which case the secret-key is 
set to that key so that encoded messages are signed with that key. If no matching key is found
to verify the signature, the message is dropped.

### Messages

The format of the message maps to send (encode and output) and received
(input and decode). The bindings are for the message id and the fields of the message.

##### message ids

`:message-id` holds a keyword of the name of the message. This is based on the <message> 
name attribute, for example name=HEARTBEAT". The name is converted to lower case and all underscores
are replaced with hyphens. Examples: `:heartbeat :sys-status :system-time`

##### fields

fields are defined by their name attribute of the <field> tag. The field names are simply converted to keywords.
Examples from the heartbeat message: `:custom_mode :type :autopilot :base_mode :system_status`

Valid values are defined in the MAVLink specification, for example `uint16_t char[16] uint64_t int8_t`
Because of the way clojure builds hash-maps, all numeric values will be held as boxed numbers, so there
is no need to worry about the type of the value beyond an in integer, a float, or character.
clj-mavlink will perform the appropriate conversions to send/receive the values appropriately.
Note that char arrays can be specified as strings. Arrays are specified as
vectors.

Note that unspecified fields will be given a value of 0.

##### system-id component-id sequence-id link-id

A message can overide the default system-id, component-id, sequence-id and link-id by specifiying it as a keyword value binding in a message hash-map, for example

```
{:message-id :heartbeat
 :system-id 0xff
 :sequence-id 0}
```

Note that a side effect of specifying the sequence id is to reset the last used sequence id to this
value. Succeeding messages sequence id will start from that value.

#### Enumerated types

One of the big benefits of clj-mavlink is that enumerated type values are supported as keywords.
For example, the `type` field of the heartbeat message is an enumerated type defined in the
MAVLink XML as MAV_TYPE, the MAV_TYPE <enum> has 29 different <entry>'s, for example,
value 0 is MAV_TYPE_GENERIC; clj-mavlink defines a keyword as it did for message ids,
convert to lower case and replace underscores with hyphens, for example
`:mav-type-generic :mav-type-fixed-wing :mav-type-coaxial :mav-type-gcs`

#### Sending messages

This example shows building and sending a heartbeat message. Remember,
the system id and component id were specified in the open channel call,
and unless specified here, the sequence id is automaticlaly calculated and added to the message.

```
(let [msg message {:message-id :heartbeat
                   :custom_mode 0
                   :type :mav-type-gcs
                   :autopilot :mav-autopilot-invalid
                   :base_mode 0
                   :system_status :mav-state-uninit})]
  (async/>!! autopilot-send msg))
```

#### Receiving messages

There are different approaches to receiving and distributing messages, the approach shown here
uses clojure.async publish/subscribe.

A message publisher is defined for the entire application based on the :message-id:

```
(defonce message-publisher (async/pub autopilot-receive
                                      :message-id
                                      (constantly (async/sliding-buffer 10))))
```

When a module wishes to receive messages, it subscribes to that partiuclar message id:

```
(async/sub message-publisher :heartbeat connect-heartbeat-chan)
```

```
(let [msg (async/<! connect-heartbeat-chan)]
  ;; process the message
)
```

## License

Copyright © 2017 Michael du Breuil

Distributed under the Eclipse Public License either version 1.0 or (at
your option) any later version.
