#include <Arduino.h>
#include <AsyncDelay.h>
#include "TaskManagerIO.h"

AsyncDelay slow_loop;
AsyncDelay fast_loop;

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

#define ODRIVE_AXIS_COUNT 2
#include "ODriveFlexCAN.h"
uint32_t node_ids[] = {0, 1};
ODriveFlexCAN odrive(node_ids);

void canSniff(const CAN_message_t &msg)
{
    odrive.filter(msg);
}

// some changes

void setup()
{
    Serial.begin(115200);
    delay(5000);

    Can0.begin();
    Can0.setBaudRate(500000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(canSniff);
    Can0.mailboxStatus();

    taskManager.scheduleOnce(
        10, []
        {
            Serial.println(" *************************** starting cal ");
            Can0.write(odrive(0).SetAxisRequestedState(ODrive::AXIS_STATE_MOTOR_CALIBRATION));
        },
        TIME_SECONDS);
    taskManager.scheduleOnce(
        15, []
        { Can0.write(odrive(0).SetAxisRequestedState(ODrive::AXIS_STATE_ENCODER_INDEX_SEARCH)); },
        TIME_SECONDS);
    taskManager.scheduleOnce(
        20, []
        { Can0.write(odrive(0).SetAxisRequestedState(ODrive::AXIS_STATE_ENCODER_OFFSET_CALIBRATION)); },
        TIME_SECONDS);

    taskManager.scheduleOnce(
        25, []
        {
            Can0.write(odrive(0).SetAxisRequestedState(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL));
            Can0.write(odrive(0).SetControllerModes(ODrive::CONTROL_MODE_VELOCITY_CONTROL, ODrive::INPUT_MODE_PASSTHROUGH));
        },
        TIME_SECONDS);

    slow_loop.start(1000, AsyncDelay::MILLIS);
    fast_loop.start(500, AsyncDelay::MICROS);
}

void loop()
{
    //taskManager.runLoop();

    Can0.events();

    if (fast_loop.isExpired())
    {
        //float freq = 0.5;
        //float signal = 20.0 * sin(freq * 2.0 * PI * (float)millis() / (1000.0));

        if (odrive(0).Heartbeat.state == ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL)
        {
            Can0.write(odrive(0).SetInputVel(0));
        }

        fast_loop.repeat();
    }

    if (slow_loop.isExpired())
    {
        Can0.write(odrive(0).SetLimits(200, 10));
        Can0.write(odrive(0).GetEncoderEstimates());
        Can0.write(odrive(0).GetVbusVoltage());

        float vbus = odrive(0).GetVbusVoltage.vbus;
        float pos = odrive(0).GetEncoderEstimates.pos;
        float vel = odrive(0).GetEncoderEstimates.vel;

        Serial.print("vbus: ");
        Serial.print(vbus, 2);
        Serial.print("; pos: ");
        Serial.print(pos, 2);
        Serial.print("; vel: ");
        Serial.println(vel, 2);

        int state = odrive(0).Heartbeat.state;
        int error = odrive(0).Heartbeat.error;

        Serial.print("state: ");
        Serial.print(state);
        Serial.print("; error: ");
        Serial.println(error);

        // Can0.write(odrive.node(0)->SetControllerModes->encode(ODrive::CONTROL_MODE_VELOCITY_CONTROL, ODrive::INPUT_MODE_PASSTHROUGH));
        Can0.write(odrive(0).SetControllerModes(ODrive::CONTROL_MODE_VELOCITY_CONTROL, ODrive::INPUT_MODE_PASSTHROUGH));

        slow_loop.repeat();
    }
}