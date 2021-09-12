# ODriveFlexCAN

An implementation of the ODrive Canbus protocol for the Arduino platform. Uses Wetmelon's https://github.com/Wetmelon/CAN-Helpers.

Refer to [examples in FlexCAN_t4](https://github.com/tonton81/FlexCAN_T4/blob/master/examples/CAN2.0_example_FIFO_with_interrupts/CAN2.0_example_FIFO_with_interrupts.ino) to get Canbus like this up and running.

## Intro
Each Canbus message is wrapped in a functor, which:
- Encodes a `CAN_Message_t` (from [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4 "FlexCAN_T4").h) so that it can be written asynchronously to a Canbus FIFO;
- Exposes the payload of those messages that come from the ODrive.

The motivation for this approach is that Canbus is a message-based protocol. If an API such as this were to implement a synchronous version of `GetEncoderEstimates()`, it would have to send the message, block until it is received, and return the data. I'm not arguing there's anything wrong with that approach. But it's not what I wanted.

The example sketch will calibrate and spin a motor.

## Constructing the object

The constructor accepts an array of `uin32_t` containing the *Canbus Node IDs* of the ODrives you want to communicate with. This is not (necessarily) the same as the Axis number. You can set/get the Node ID via `<axis>.config.can_node_id`. (Note that while ODrive's Canbus protocol supports changing the Node ID, that message is not implemented in this API becuase it would break the node ID lookup. Sorry!)

    #include "ODriveFlexCAN.h"
    uint32_t node_ids[] = {0, 1};
    ODriveFlexCAN odrive(node_ids);
    
This object is used directly to filter the incoming Canbus traffic via `filter()`.

Additionally, invoking `operator()(uint32_t node_id)` on this object returns a reference to a particular ODrive Canbus 'node' - the one whose ID matches the argument. This 'node' is not meant to be referenced directly by the user. It is just an abstraction used to access a message's functor, which is used for encoding and decoding data.
   
## Listening for data

Each ODrive axis *always* outputs a heartbeat every `<axis>.config.can_heartbeat_rate_ms` ms. (The default is 200 ms.) The heartbeat message contains payload - the axis error code and the axis state.

(Note: The heartbeat message is unique in this protocol. It is the only message that ODrive transmits without being asked.)

`canSniff()` is a callback for FlexCAN's `onReceive()` and now all the Canbus traffic is going into `filter()` (whether it comes from an ODrive or not):

    void canSniff(const CAN_message_t &msg)
    {
        odrive.filter(msg);
    }

So now, if there is an ODrive on the bus with a node ID matching one of the values passed to the ODriveFlexCAN constructor (`0` in this example), then the error and state can be accessed:

    AxisState state = odrive(0).Heartbeat.state;
    AxisError error = odrive(0).Heartbeat.error;

## Sending messages

ODriveFlexCAN does not send messages. You use FlexCAN_t4 to send messages encoded by ODriveFlexCAN. 

FlexCAN_t4's `write()` expects a `CAN_message_t`, so that's what ODriveFlexCAN provides. Simple as that.

Consider `SetControllerModes`. According to the protocol, there is no response to the `SetControllerModes` message. It can be sent:

    Can0.write( odrive(0).SetControllerModes(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH) );
    
The state can be verified using the Heartbeat message, and there is no readback of the input mode on this protocol.

Messages that contain no payload are similarly sent:

    Can0.write( odrive(0).ClearErrors() );
    
Some messages are sent with *no* payload, but are returned from the ODrive *with* payload. For example, for `GetEncoderEstimates`, you must first send the message:

    Can0.write( odrive(0).GetEncoderEstimates() );

Then wait a little while. ODrive will receive the message, and re-transmit it *sometime later* with payload populated. (In my tests on a Teensy 4 and ODrive 3.6, the round-trip time was around 580 us.)

Then as long as you're passing all your incoming Canbus `msg`es to `odrive.filter(msg);`, at any time, you can use the functor `GetEncoderEstimates` to access the most recent payload:

    float pos = odrive(0).GetEncoderEstimates.pos;
    float vel = odrive(0).GetEncoderEstimates.vel;

But if you stop sending `GetEncoderEstimates()` then the payload will be stale! So you need to keep sending messages to ODrive requesting data you want at each iteration of your program. Not too frequently though, because then you'll flood the bus. And not too infrequently either, because then your data will be old.
