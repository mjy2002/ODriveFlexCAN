#ifndef ODriveCanbus_h
#define ODriveCanbus_h

#include "Arduino.h"
#include <FlexCAN_T4.h>

// Upper 6 bits - Node ID (axis id)
// Lower 5 bits - Command ID
// can_id = axis_id << 5 | cmd_id
// node_id = can_id >> 5
// cmd_id = can_id & 0b00011111 (0x1F)
#define GET_CANBUS_ID(node_id, msg_id) ((node_id << 5) + msg_id)
#define GET_NODE_ID(canbus_id) (canbus_id >> 5)
#define GET_MSG_ID(canbus_id) (canbus_id & 0x1F)

#ifndef ODRIVE_AXIS_COUNT
#define ODRIVE_AXIS_COUNT 2
#endif

class ODriveCanbus
{
public:
    enum AxisState_t
    {
        AXIS_STATE_UNDEFINED = 0,                  //<! will fall through to idle
        AXIS_STATE_IDLE = 1,                       //<! disable PWM and do nothing
        AXIS_STATE_STARTUP_SEQUENCE = 2,           //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,  //<! run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,          //<! run motor calibration
        AXIS_STATE_SENSORLESS_CONTROL = 5,         //<! run sensorless control
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6,       //<! run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8         //<! run closed loop control
    };

    enum ControlMode_t
    {
        CONTROL_MODE_VOLTAGE_CONTROL = 0,
        CONTROL_MODE_TORQUE_CONTROL = 1,
        CONTROL_MODE_VELOCITY_CONTROL = 2,
        CONTROL_MODE_POSITION_CONTROL = 3
    };

    enum InputMode_t
    {
        INPUT_MODE_INACTIVE = 0,
        INPUT_MODE_PASSTHROUGH = 1,
        INPUT_MODE_VEL_RAMP = 2,
        INPUT_MODE_POS_FILTER = 3,
        INPUT_MODE_MIX_CHANNELS = 4,
        INPUT_MODE_TRAP_TRAJ = 5,
        INPUT_MODE_TORQUE_RAMP = 6,
        INPUT_MODE_MIRROR = 7,
        INPUT_MODE_Tuning = 8
    };

private:
    enum MessageID_t
    {
        CANOpen_NMT = 0x00,
        ODrive_Heartbeat = 0x01,
        ODrive_EStop = 0x02,
        GetMotorError = 0x03,
        GetEncoderError = 0x04,
        GetSensorlessError = 0x05,
        SetAxisNodeId = 0x06,
        SetAxisRequestedState = 0x07,
        SetAxisStartupConfig = 0x08,
        GetEncoderEstimates = 0x09,
        GetEncoderCount = 0x0A,
        SetControllerModes = 0x0B,
        SetInputPos = 0x0C,
        SetInputVel = 0x0D,
        SetInputTorque = 0x0E,
        SetLimits = 0x0F,
        StartAnticogging = 0x10,
        SetTrajVelLimit = 0x11,
        SetTrajAccelLimits = 0x12,
        SetTrajInertia = 0x13,
        GetIQ = 0x14,
        GetSensorlessEstimates = 0x15,
        RebootODrive = 0x16,
        GetVbusVoltage = 0x17,
        ClearErrors = 0x18,
        SetLinearCount = 0x19,
        CANOpenHeartbeat = 0x700
    };

    static bool is_rtr_message(uint32_t message_id)
    {
        return message_id == MessageID_t::GetMotorError ||
               message_id == MessageID_t::GetEncoderError ||
               message_id == MessageID_t::GetSensorlessError ||
               message_id == MessageID_t::GetEncoderEstimates ||
               message_id == MessageID_t::GetEncoderCount ||
               message_id == MessageID_t::GetIQ ||
               message_id == MessageID_t::GetSensorlessEstimates;
    };

    class MessageBase_t
    {
    public:
        MessageBase_t(uint32_t node_id, uint32_t message_id)
            : _node_id(node_id),
              _message_id(message_id){};

    protected:
        const uint32_t _node_id;
        const uint32_t _message_id;

        static float buffer_to_float(const uint8_t *buffer)
        {
            float y;
            uint32_t *const py = (uint32_t *)&y;

            *py = ((uint32_t)buffer[3] << 24) |
                  ((uint32_t)buffer[2] << 16) |
                  ((uint32_t)buffer[1] << 8) |
                  ((uint32_t)buffer[0] << 0);

            return y;
        }

        static uint32_t buffer_to_uint32(const uint8_t *buffer)
        {
            return uint32_t((buffer[3] << 24) |
                            (buffer[2] << 16) |
                            (buffer[1] << 8) |
                            buffer[0]);
        }

        static void uint32_to_buffer(uint32_t payload, uint8_t *buffer)
        {
            buffer[3] = (payload >> 24) & 0xFF;
            buffer[2] = (payload >> 16) & 0xFF;
            buffer[1] = (payload >> 8) & 0xFF;
            buffer[0] = payload & 0xFF;
        }

        static void float_to_buffer(float payload, uint8_t *buffer)
        {
            uint8_t *f_byte = reinterpret_cast<uint8_t *>(&payload);
            memcpy(buffer, f_byte, 4);
        }

        CAN_message_t encode_empty()
        {
            CAN_message_t msg;
            msg.id = GET_CANBUS_ID(_node_id, _message_id);
            msg.len = 8;
            *msg.buf = 0;
            msg.flags.remote = is_rtr_message(_message_id);
            return msg;
        }

        CAN_message_t encode_uint32(uint32_t payload1 = 0, uint32_t payload2 = 0)
        {
            CAN_message_t msg;
            msg.id = GET_CANBUS_ID(_node_id, _message_id);
            msg.len = 8;
            uint32_to_buffer(payload1, msg.buf);
            uint32_to_buffer(payload2, msg.buf + 4);
            msg.flags.remote = is_rtr_message(_message_id);
            return msg;
        }

        CAN_message_t encode_float(float payload1 = 0, float payload2 = 0)
        {
            CAN_message_t msg;
            msg.id = GET_CANBUS_ID(_node_id, _message_id);
            msg.len = 8;
            float_to_buffer(payload1, msg.buf);
            float_to_buffer(payload2, msg.buf + 4);
            msg.flags.remote = is_rtr_message(_message_id);
            return msg;
        }
    };

    class Heartbeat_t : public MessageBase_t
    {
    public:
        uint32_t error;
        AxisState_t state;

        Heartbeat_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::ODrive_Heartbeat),
              error(0),
              state(AxisState_t::AXIS_STATE_UNDEFINED){};

        void decode(const uint8_t *buf, uint8_t len)
        {
            error = buffer_to_uint32(buf);
            state = AxisState_t(buffer_to_uint32(buf + 4));
        }
    };

    // class EStop_t {};
    // class GetMotorError_t {};
    // class GetSensorlessError_t {};
    // class SetAxisNodeID_t {};

    class SetAxisRequestedState_t : public MessageBase_t
    {
    public:
        SetAxisRequestedState_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetAxisRequestedState){};

        CAN_message_t encode(AxisState_t requested_state)
        {
            return MessageBase_t::encode_uint32((uint32_t)requested_state);
        }
    };

    // class SetAxisStartupConfig_t {};

    class GetEncoderEstimates_t : public MessageBase_t
    {
    public:
        float pos;
        float vel;
        GetEncoderEstimates_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetEncoderEstimates),
              pos(0),
              vel(0){};

        CAN_message_t encode()
        {
            return MessageBase_t::encode_empty();
        }

        void decode(const uint8_t *buf, uint8_t len)
        {
            pos = buffer_to_float(buf);
            vel = buffer_to_float(buf + 4);
        }
    };

    // class GetEncoderCount_t {};

    class SetControllerModes_t : public MessageBase_t
    {
    public:
        SetControllerModes_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetControllerModes){};

        CAN_message_t encode(uint32_t control_mode, uint32_t input_mode)
        {
            return MessageBase_t::encode_uint32(control_mode, input_mode);
        }
    };

    // class SetInputPos_t {};

    class SetInputVel_t : public MessageBase_t
    {
    public:
        SetInputVel_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputVel){};

        CAN_message_t encode(float input_vel)
        {
            //float ff = 0.0;
            return MessageBase_t::encode_float(input_vel); //, ff);
        }
    };

    // class SetInputTorque_t {};

    class SetLimits_t : public MessageBase_t
    {
    public:
        SetLimits_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetLimits){};

        CAN_message_t encode(float vel_lim, float cur_lim)
        {
            return MessageBase_t::encode_float(vel_lim, cur_lim);
        }
    };

    // class StartAnticogging_t {};
    // class SetTrajVelLimit_t {};
    // class SetTrajAccelLimits_t {};
    // class SetTrajIntertia_t {};
    // class GetIQ_t {};
    // class GetSensorlessEsitaimtes_t {};
    // class RebootOdrive_t {};
    // class GetVbusVoltage_t {};
    // class ClearErrors_t {};
    // class SetLinearCount_t {};

private:
    class ODriveNode_t
    {
    public:
        ODriveNode_t(uint32_t node_id)
        {
            Heartbeat = new Heartbeat_t(node_id);
            // Estop
            // GetMotorError = new GetMotorError_t(_node_id);
            // GetSensorlessError
            // SetAxisNodeID
            SetAxisRequestedState = new SetAxisRequestedState_t(node_id);
            // SetAxisStartupConfig
            GetEncoderEstimates = new GetEncoderEstimates_t(node_id);
            // GetEncoderCount
            SetControllerModes = new SetControllerModes_t(node_id);
            // SetInputPos
            SetInputVel = new SetInputVel_t(node_id);
            // SetInputTorque
            SetLimits = new SetLimits_t(node_id);
            // StartAnticogging
            // SetTrajVelLimit
            // SetTrajAccelLimits
            // SetTrajIntertia
            // GetIQ
            // GetSensorlessEsitaimtes
            // RebootOdrive
            // GetVbusVoltage
            // ClearErrors
            // SetLinearCount
        };

    public:
        uint32_t _node_id;
        Heartbeat_t *Heartbeat;
        // Estop_t* Estop;
        // GetMotorError_t* GetMotorError;
        // GetSensorlessError
        // SetAxisNodeID
        SetAxisRequestedState_t *SetAxisRequestedState;
        // SetAxisStartupConfig
        GetEncoderEstimates_t *GetEncoderEstimates;
        // GetEncoderCount
        SetControllerModes_t *SetControllerModes;
        // SetInputPos
        SetInputVel_t *SetInputVel;
        // SetInputTorque
        SetLimits_t *SetLimits;
        // StartAnticogging
        // SetTrajVelLimit
        // SetTrajAccelLimits
        // SetTrajIntertia
        // GetIQ
        // GetSensorlessEsitaimtes
        // RebootOdrive
        // GetVbusVoltage
        // ClearErrors
        // SetLinearCount
    };

    ODriveNode_t *_nodes[ODRIVE_AXIS_COUNT];

public:
    ODriveCanbus(uint32_t *node_ids)
    {
        for (uint32_t i = 0; i < ODRIVE_AXIS_COUNT; i++)
        {
            _nodes[i] = new ODriveNode_t(node_ids[i]);
            _nodes[i]->_node_id = node_ids[i];
        }
    }

    const ODriveNode_t *node(uint32_t node_id) const
    {
        for (uint32_t i = 0; i < ODRIVE_AXIS_COUNT; i++)
            if (_nodes[i]->_node_id == node_id)
                return _nodes[i];
    }

    void decode(uint32_t canbus_id, const uint8_t *buf, uint8_t len)
    {
        uint32_t node_id = GET_NODE_ID(canbus_id);
        uint32_t msg_id = GET_MSG_ID(canbus_id);

        for (uint32_t i = 0; i < ODRIVE_AXIS_COUNT; i++)
        {
            if (_nodes[i]->_node_id == node_id)
            {
                //Serial.print("decode ");
                //Serial.println(msg_id);
                if (msg_id == MessageID_t::ODrive_Heartbeat)
                {
                    _nodes[i]->Heartbeat->decode(buf, len);
                }
                if (msg_id == MessageID_t::GetMotorError)
                {
                    //_nodes[i]->GetMotorError->decode(buf, len);
                }
                if (msg_id == MessageID_t::GetEncoderError)
                {
                    //_nodes[i]->GetEncoderError->decode(buf, len);
                }
                if (msg_id == MessageID_t::GetSensorlessError)
                {
                    //_nodes[i]->GetSensorlessError->decode(buf, len);
                }
                if (msg_id == MessageID_t::GetEncoderEstimates)
                {
                    _nodes[i]->GetEncoderEstimates->decode(buf, len);
                }
                if (msg_id == MessageID_t::GetEncoderCount)
                {
                    //_nodes[i]->GetEncoderCount->decode(buf, len);
                }
                if (msg_id == MessageID_t::GetIQ)
                {
                    //_nodes[i]->GetIQ->decode(buf, len);
                }
                if (msg_id == MessageID_t::GetSensorlessEstimates)
                {
                    //_nodes[i]->GetSensorlessEstimates->decode(buf, len);
                }
            }
        }
    }
};

#undef GET_CANBUS_ID
#undef GET_NODE_ID
#undef GET_MSG_ID
#endif