#ifndef ODriveCanbus_h
#define ODriveCanbus_h

#include "Arduino.h"

#ifndef ODRIVE_AXIS_COUNT
#define ODRIVE_AXIS_COUNT 2
#endif

#ifndef _FLEXCAN_T4_H_
#error The FlexCAN header must be included above ODriveFlexCAN so that the 'CAN_message_t' type gets defined.
#endif

// Upper 6 bits - Node ID (axis id)
// Lower 5 bits - Command ID
// can_id = axis_id << 5 | cmd_id
// node_id = can_id >> 5
// cmd_id = can_id & 0b00011111 (0x1F)
#define GET_CANBUS_ID(node_id, msg_id) ((node_id << 5) + msg_id)
#define GET_NODE_ID(canbus_id) (canbus_id >> 5)
#define GET_MSG_ID(canbus_id) (canbus_id & 0x1F)

namespace CanbusConverters
{
    float buffer_to_float(const uint8_t *buffer)
    {
        float y;
        uint32_t *const py = (uint32_t *)&y;

        *py = ((uint32_t)buffer[3] << 24) |
              ((uint32_t)buffer[2] << 16) |
              ((uint32_t)buffer[1] << 8) |
              ((uint32_t)buffer[0] << 0);

        return y;
    }

    uint32_t buffer_to_uint32(const uint8_t *buffer)
    {
        return uint32_t((buffer[3] << 24) |
                        (buffer[2] << 16) |
                        (buffer[1] << 8) |
                        buffer[0]);
    }

    void uint32_to_buffer(uint32_t payload, uint8_t *buffer)
    {
        buffer[3] = (payload >> 24) & 0xFF;
        buffer[2] = (payload >> 16) & 0xFF;
        buffer[1] = (payload >> 8) & 0xFF;
        buffer[0] = payload & 0xFF;
    }

    void float_to_buffer(float payload, uint8_t *buffer)
    {
        uint8_t *f_byte = reinterpret_cast<uint8_t *>(&payload);
        memcpy(buffer, f_byte, 4);
    }
};

namespace ODrive
{
#include "ODrive\Arduino\ODriveArduino\ODriveEnums.h"
};

class ODriveFlexCAN
{
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

        CAN_message_t encode_uint32(uint32_t payload1 = 0, uint32_t payload2 = 0) const
        {
            CAN_message_t msg;
            msg.id = GET_CANBUS_ID(_node_id, _message_id);
            msg.len = 8;
            CanbusConverters::uint32_to_buffer(payload1, msg.buf);
            CanbusConverters::uint32_to_buffer(payload2, msg.buf + 4);
            msg.flags.remote = is_rtr_message(_message_id);
            return msg;
        }
        CAN_message_t encode_float(float payload1 = 0, float payload2 = 0) const
        {
            CAN_message_t msg;
            msg.id = GET_CANBUS_ID(_node_id, _message_id);
            msg.len = 8;
            CanbusConverters::float_to_buffer(payload1, msg.buf);
            CanbusConverters::float_to_buffer(payload2, msg.buf + 4);
            msg.flags.remote = is_rtr_message(_message_id);
            return msg;
        }
    };
    class Heartbeat_t : public MessageBase_t
    {
    public:
        ODrive::AxisError error;
        ODrive::AxisState state;

        Heartbeat_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::ODrive_Heartbeat),
              error(ODrive::AxisError::AXIS_ERROR_NONE),
              state(ODrive::AxisState::AXIS_STATE_UNDEFINED){};
    };
    class EStop_t : public MessageBase_t
    {
    public:
        EStop_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::ODrive_EStop){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    class GetMotorError_t : public MessageBase_t
    {
    public:
        ODrive::MotorError error;
        GetMotorError_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetMotorError),
              error(ODrive::MotorError::MOTOR_ERROR_NONE){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    class GetEncoderError_t : public MessageBase_t
    {
    public:
        ODrive::EncoderError error;
        GetEncoderError_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetEncoderError),
              error(ODrive::EncoderError::ENCODER_ERROR_NONE){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    class GetSensorlessError_t : public MessageBase_t
    {
    public:
        ODrive::SensorlessEstimatorError error;
        GetSensorlessError_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetSensorlessError),
              error(ODrive::SensorlessEstimatorError::SENSORLESS_ESTIMATOR_ERROR_NONE){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    // class SetAxisNodeID_t {};
    class SetAxisRequestedState_t : public MessageBase_t
    {
    public:
        SetAxisRequestedState_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetAxisRequestedState){};

        CAN_message_t operator()(ODrive::AxisState requested_state) const
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

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    class GetEncoderCount_t : public MessageBase_t
    {
    public:
        int32_t shadow_count;
        int32_t count_cpr;
        GetEncoderCount_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetEncoderCount),
              shadow_count(0),
              count_cpr(0){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    class SetControllerModes_t : public MessageBase_t
    {
    public:
        SetControllerModes_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetControllerModes){};

        CAN_message_t operator()(ODrive::ControlMode control_mode, ODrive::InputMode input_mode) const
        {
            return MessageBase_t::encode_uint32(control_mode, input_mode);
        }
    };
    class SetInputPos_t : public MessageBase_t
    {
    public:
        SetInputPos_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputPos){};

        CAN_message_t operator()(float input_pos, int8_t vel_ff, int8_t torque_ff) const
        {
            return MessageBase_t::encode_float(input_pos, input_pos); // check
        }
    };
    class SetInputVel_t : public MessageBase_t
    {
    public:
        SetInputVel_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputVel){};

        CAN_message_t operator()(float input_vel) const
        {
            //float ff = 0.0;
            return MessageBase_t::encode_float(input_vel); //, ff);
        }
    };
    class SetInputTorque_t : public MessageBase_t
    {
    public:
        SetInputTorque_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputTorque){};

        CAN_message_t operator()(float input_torque) const
        {
            //float ff = 0.0;
            return MessageBase_t::encode_float(input_torque); //, ff);
        }
    };
    class SetLimits_t : public MessageBase_t
    {
    public:
        SetLimits_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetLimits){};

        CAN_message_t operator()(float vel_lim, float cur_lim) const
        {
            return MessageBase_t::encode_float(vel_lim, cur_lim);
        }
    };
    class StartAnticogging_t : public MessageBase_t
    {
    public:
        StartAnticogging_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::StartAnticogging){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    // class SetTrajVelLimit_t {};
    // class SetTrajAccelLimits_t {};
    // class SetTrajIntertia_t {};
    class GetIQ_t : public MessageBase_t
    {
    public:
        float iq;
        GetIQ_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetIQ),
            iq(0) {};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    class GetSensorlessEstimates_t : public MessageBase_t
    {
    public:
        float pos;
        float vel;
        GetSensorlessEstimates_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetSensorlessEstimates),
                pos(0),
                vel(0){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    class RebootOdrive_t : public MessageBase_t
    {
    public:
        RebootOdrive_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::RebootODrive){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    class GetVbusVoltage_t : public MessageBase_t
    {
    public:
        float vbus;
        GetVbusVoltage_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetVbusVoltage),
                vbus(0){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    class ClearErrors_t : public MessageBase_t
    {
    public:
        ClearErrors_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::ClearErrors){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::encode_uint32();
        }
    };
    // class SetLinearCount_t {};

private:
    class ODriveNode_t
    {
    public:
        ODriveNode_t(uint32_t node_id)
            : _node_id(node_id),
              Heartbeat(node_id),
              EStop(node_id),
              GetMotorError(node_id),
              GetEncoderError(node_id),
              GetSensorlessError(node_id),
              //SetAxisNodeID(node_id),
              SetAxisRequestedState(node_id),
              //SetAxisStartupConfig(node_id),
              GetEncoderEstimates(node_id),
              GetEncoderCount(node_id),
              SetControllerModes(node_id),
              SetInputPos(node_id),
              SetInputVel(node_id),
              SetInputTorque(node_id),
              SetLimits(node_id),
              StartAnticogging(node_id),
              // SetTrajVelLimit
              // SetTrajAccelLimits
              // SetTrajIntertia
              GetIQ(node_id),
              GetSensorlessEstimates(node_id),
              RebootOdrive(node_id),
              GetVbusVoltage(node_id),
              ClearErrors(node_id)
              // SetLinearCount

              {};

    private:
        uint32_t _node_id;

    public:
        uint32_t getNodeId() { return _node_id; };

    public:
        Heartbeat_t Heartbeat;
        EStop_t EStop;
        GetMotorError_t GetMotorError;
        GetEncoderError_t GetEncoderError;
        GetSensorlessError_t GetSensorlessError;
        // SetAxisNodeID
        SetAxisRequestedState_t SetAxisRequestedState;
        // SetAxisStartupConfig
        GetEncoderEstimates_t GetEncoderEstimates;
        GetEncoderCount_t GetEncoderCount;
        SetControllerModes_t SetControllerModes;
        SetInputPos_t SetInputPos;
        SetInputVel_t SetInputVel;
        SetInputTorque_t SetInputTorque;
        SetLimits_t SetLimits;
        StartAnticogging_t StartAnticogging;
        // SetTrajVelLimit
        // SetTrajAccelLimits
        // SetTrajIntertia
        GetIQ_t GetIQ;
        GetSensorlessEstimates_t GetSensorlessEstimates;
        RebootOdrive_t RebootOdrive;
        GetVbusVoltage_t GetVbusVoltage;
        ClearErrors_t ClearErrors;
        // SetLinearCount
    };
    ODriveNode_t *_nodes[ODRIVE_AXIS_COUNT];

public:
    ODriveFlexCAN(uint32_t *node_ids)
    {
        for (uint32_t i = 0; i < ODRIVE_AXIS_COUNT; i++)
        {
            _nodes[i] = new ODriveNode_t(node_ids[i]);
        }
    }
    const ODriveNode_t &operator()(uint32_t node_id) const
    {
        for (uint32_t i = 0; i < ODRIVE_AXIS_COUNT; i++)
            if (_nodes[i]->getNodeId() == node_id)
                return *_nodes[i];
        
        return *_nodes[0];
    }
    void filter(const CAN_message_t &msg)
    {
        uint32_t node_id = GET_NODE_ID(msg.id);
        uint32_t msg_id = GET_MSG_ID(msg.id);

        for (uint32_t i = 0; i < ODRIVE_AXIS_COUNT; i++)
        {
            if (_nodes[i]->getNodeId() == node_id)
            {
                //Serial.print("decode ");
                //Serial.println(msg_id);
                if (msg_id == MessageID_t::ODrive_Heartbeat)
                {
                    //_nodes[i]->Heartbeat.error = (AxisError)can_getSignal<uint32_t>(msg, 0, 4, true);
                    //_nodes[i]->Heartbeat.state = (AxisState)can_getSignal<uint32_t>(msg, 4, 4, true);

                    _nodes[i]->Heartbeat.error = (ODrive::AxisError)CanbusConverters::buffer_to_uint32(msg.buf);
                    _nodes[i]->Heartbeat.state = (ODrive::AxisState)CanbusConverters::buffer_to_uint32(msg.buf + 4);
                }
                if (msg_id == MessageID_t::GetMotorError)
                {
                    _nodes[i]->GetMotorError.error = (ODrive::MotorError)CanbusConverters::buffer_to_uint32(msg.buf);
                }
                if (msg_id == MessageID_t::GetEncoderError)
                {
                    _nodes[i]->GetEncoderError.error = (ODrive::EncoderError)CanbusConverters::buffer_to_uint32(msg.buf);
                }
                if (msg_id == MessageID_t::GetSensorlessError)
                {
                    _nodes[i]->GetSensorlessError.error = (ODrive::SensorlessEstimatorError)CanbusConverters::buffer_to_uint32(msg.buf);
                }
                if (msg_id == MessageID_t::GetEncoderEstimates)
                {
                    //_nodes[i]->GetEncoderEstimates.pos = can_getSignal<float>(msg, 0, 4, true, 1, 0);
                    //_nodes[i]->GetEncoderEstimates.vel = can_getSignal<float>(msg, 4, 4, true, 1, 0);
                    
                    _nodes[i]->GetEncoderEstimates.pos = CanbusConverters::buffer_to_float(msg.buf);
                    _nodes[i]->GetEncoderEstimates.vel = CanbusConverters::buffer_to_float(msg.buf + 4);
                }
                if (msg_id == MessageID_t::GetEncoderCount)
                {
                    _nodes[i]->GetEncoderCount.shadow_count = (int)CanbusConverters::buffer_to_uint32(msg.buf);
                    _nodes[i]->GetEncoderCount.count_cpr = (int)CanbusConverters::buffer_to_uint32(msg.buf + 4);
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