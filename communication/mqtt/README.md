# MQTT 

Component `Mqtt`

## `coyot3::communication::mqtt::MqttGateway`

>> To be deprecated in favor of `coyot3::communication::mqtt::Client`

It is a client mqtt that permits to:

* make a configuration of the connection, including the possibility configure user/pass and mqtt-id and ssl communication.
* prepare the subscriptions, and link the incoming data to specific callbacks
* simplify the publication operations. It will also ensure the publication of *important* tagged messages.
* the connector will keep the connection up, and on reboot of the broker or any communication interruption, it will redo the prepared subscriptions.

### `MqttGatewayAcrp`

This class is built on the top of the `mqtt::MqttGateway` library.

The `ACRP` is a protocol I imagined to solve a mayor issue when trying to communicate endpoints throw MQTT. However, this problem is currently being solved by `AMQP` protocol.

This `ACRP` (*Acknowledge Communications Redundancy Protocol*) is intended to be applied to some specific "channels / topics" when messages need to be served to an external endpoint, while the defined protocol does not inform to the emitter about the reception of that packet. *For example, the information of a relevant event, and the protocol does not define an ACK.*

To be able for the emitter to acknowledge the deliver, this protocol is created by *duplicating* the number of topics used: one for the emission, and other for the reception of the acknowledge. This way, alas the lack of definition of the MQTT protocol to inform to the emitter about the reception of the packet by an external application, it is possible for the emitter to acknowledge this reception.


## dev notes

Basically: the `MqttGateway` it is a wrapper for mosquitto-dev library.

