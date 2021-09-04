# ODriveFlexCAN

An implementation of the ODrive Canbus protocol for the Arduino platform. Uses Wetmelon's https://github.com/Wetmelon/CAN-Helpers with a [small code change](https://github.com/Wetmelon/CAN-Helpers/pull/1) to get it to build under Arduino.

Each Canbus message is wrapped in a functor, which:
- Uses `operator()` to encode a `CAN_Message_t` (from [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4 "FlexCAN_T4").h) so that it can be written asynchronously to a Canbus FIFO;
- Uses public data members to expose the payload of those messages that come from the ODrive.

The motivation for this approach is that Canbus is a message-based protocol. If an API such as this were to implement a synchronous version of `GetEncoderEstimates()`, it would have to send the message, block until it is received, and return the data. I'm not arguing there's anything wrong with that approach. But it's not what I wanted.

Instead, this API makes the user send each message via their own `FlexCAN_T4` instance. So if you want to know the encoder estimates, you must first send the message:

`Can0.write(odrive(0).GetEncoderEstimates());`

Then wait a little while. (In my tests on a Teensy 4 and ODrive 3.6, the round-trip time was around 580 us.)

At any time, you can use the functor `GetEncoderEstimates` to access the most recent payload:

`float pos = odrive(0).GetEncoderEstimates.pos;`
`float vel = odrive(0).GetEncoderEstimates.vel;`

But if you stop sending `GetEncoderEstimates()` then the payload will be stale.

The example sketch will calibrate and spin a motor.
