#ifndef ODriveCanbus_h
#define ODriveCanbus_h

#include "Arduino.h"

#ifndef ODRIVE_AXIS_COUNT
#define ODRIVE_AXIS_COUNT 2
#endif

#include <FlexCAN_T4.h>

namespace ODrive
{
#include "ODriveEnums.h"
};

//#include "CAN-Helpers\can_helpers.hpp"
#include "can_helpers.hpp"

// Upper 6 bits - Node ID (axis id)
// Lower 5 bits - Command ID
// can_id = axis_id << 5 | cmd_id
// node_id = can_id >> 5
// cmd_id = can_id & 0b00011111 (0x1F)
#define GET_CANBUS_ID(node_id, msg_id) ((node_id << 5) + msg_id)
#define GET_NODE_ID(canbus_id) (canbus_id >> 5)
#define GET_MSG_ID(canbus_id) (canbus_id & 0x1F)


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
               message_id == MessageID_t::GetSensorlessEstimates ||
               message_id == MessageID_t::GetVbusVoltage;
    };
    
    class MessageBase_t
    {
    public:
        MessageBase_t(uint32_t node_id, uint32_t message_id)
            : _node_id(node_id),
              _message_id(message_id){};

        CAN_message_t operator()() const
        {
            return MessageBase_t::pack();
        }

    protected:
        const uint32_t _node_id;
        const uint32_t _message_id;

        CAN_message_t pack() const
        {
            uint8_t zero[8] = {0};
            return pack(zero);
        }
        CAN_message_t pack(const uint8_t *buf) const
        {
            CAN_message_t msg;
            msg.id = GET_CANBUS_ID(_node_id, _message_id);
            msg.len = 8;
            std::memcpy(msg.buf, buf, msg.len);
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
    };
    class GetMotorError_t : public MessageBase_t
    {
    public:
        ODrive::MotorError error;
        GetMotorError_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetMotorError),
              error(ODrive::MotorError::MOTOR_ERROR_NONE){};
    };
    class GetEncoderError_t : public MessageBase_t
    {
    public:
        ODrive::EncoderError error;
        GetEncoderError_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetEncoderError),
              error(ODrive::EncoderError::ENCODER_ERROR_NONE){};
    };
    class GetSensorlessError_t : public MessageBase_t
    {
    public:
        ODrive::SensorlessEstimatorError error;
        GetSensorlessError_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetSensorlessError),
              error(ODrive::SensorlessEstimatorError::SENSORLESS_ESTIMATOR_ERROR_NONE){};
    };
    // class SetAxisNodeID_t {};
    class SetAxisRequestedState_t : public MessageBase_t
    {
    public:
        SetAxisRequestedState_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetAxisRequestedState){};

        CAN_message_t operator()(ODrive::AxisState requested_state) const
        {
            can_Message_t msg;
            can_setSignal<uint32_t>(msg, (uint32_t)requested_state, 0, 32, true);
            return MessageBase_t::pack(msg.buf);
        }
    };
    // class SetAxisStartupConfig_t {};
    class GetEncoderEstimates_t : public MessageBase_t
    {
    public:
        float pos, vel;
        GetEncoderEstimates_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetEncoderEstimates),
              pos(0),
              vel(0){};
    };
    class GetEncoderCount_t : public MessageBase_t
    {
    public:
        int32_t shadow_count, count_cpr;
        GetEncoderCount_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetEncoderCount),
              shadow_count(0),
              count_cpr(0){};
    };
    class SetControllerModes_t : public MessageBase_t
    {
    public:
        SetControllerModes_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetControllerModes){};

        CAN_message_t operator()(ODrive::ControlMode control_mode, ODrive::InputMode input_mode) const
        {
            can_Message_t msg;
            can_setSignal<uint32_t>(msg, (uint32_t)control_mode, 0, 32, true);
            can_setSignal<uint32_t>(msg, (uint32_t)input_mode, 32, 32, true);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class SetInputPos_t : public MessageBase_t
    {
    public:
        SetInputPos_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputPos){};

        CAN_message_t operator()(float input_pos, int16_t vel_ff = 0, int16_t torque_ff = 0) const
        {
            can_Message_t msg;
            can_setSignal<float>(msg, input_pos, 0, 32, true, 1, 0);
            can_setSignal<int16_t>(msg, vel_ff, 32, 16, true, 0.001, 0);
            can_setSignal<int16_t>(msg, torque_ff, 48, 16, true, 0.001, 0);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class SetInputVel_t : public MessageBase_t
    {
    public:
        SetInputVel_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputVel){};

        CAN_message_t operator()(float input_vel, float torque_ff = 0) const
        {
            can_Message_t msg;
            can_setSignal<float>(msg, input_vel, 0, 32, true, 1, 0);
            can_setSignal<float>(msg, torque_ff, 32, 32, true, 1, 0);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class SetInputTorque_t : public MessageBase_t
    {
    public:
        SetInputTorque_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputTorque){};

        CAN_message_t operator()(float input_torque) const
        {
            can_Message_t msg;
            can_setSignal<float>(msg, input_torque, 0, 32, true, 1, 0);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class SetLimits_t : public MessageBase_t
    {
    public:
        SetLimits_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetLimits){};

        CAN_message_t operator()(float vel_lim, float cur_lim) const
        {
            can_Message_t msg;
            can_setSignal<float>(msg, vel_lim, 0, 32, true, 1, 0);
            can_setSignal<float>(msg, cur_lim, 32, 32, true, 1, 0);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class StartAnticogging_t : public MessageBase_t
    {
    public:
        StartAnticogging_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::StartAnticogging){};
    };
    class SetTrajVelLimit_t : public MessageBase_t
    {
    public:
        SetTrajVelLimit_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetTrajVelLimit){};

        CAN_message_t operator()(float traj_vel_lim) const
        {
            can_Message_t msg;
            can_setSignal<float>(msg, traj_vel_lim, 0, 32, true, 1, 0);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class SetTrajAccelLimits_t : public MessageBase_t
    {
    public:
        SetTrajAccelLimits_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetTrajAccelLimits){};

        CAN_message_t operator()(float traj_accel_lim, float traj_decel_lim) const
        {
            can_Message_t msg;
            can_setSignal<float>(msg, traj_accel_lim, 0, 32, true, 1, 0);
            can_setSignal<float>(msg, traj_decel_lim, 32, 32, true, 1, 0);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class SetTrajInertia_t : public MessageBase_t
    {
    public:
        SetTrajInertia_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetTrajInertia){};

        CAN_message_t operator()(float traj_inertia) const
        {
            can_Message_t msg;
            can_setSignal<float>(msg, traj_inertia, 0, 32, true, 1, 0);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class GetIQ_t : public MessageBase_t
    {
    public:
        float iq_setpoint, iq_measured;
        GetIQ_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetIQ),
            iq_setpoint(0),
            iq_measured(0) {};
    };
    class GetSensorlessEstimates_t : public MessageBase_t
    {
    public:
        float pos, vel;
        GetSensorlessEstimates_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetSensorlessEstimates),
                pos(0),
                vel(0){};
    };
    class RebootOdrive_t : public MessageBase_t
    {
    public:
        RebootOdrive_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::RebootODrive){};
    };
    class GetVbusVoltage_t : public MessageBase_t
    {
    public:
        float vbus;
        GetVbusVoltage_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetVbusVoltage),
                vbus(0){};
    };
    class ClearErrors_t : public MessageBase_t
    {
    public:
        ClearErrors_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::ClearErrors){};
    };
    class SetLinearCount_t : public MessageBase_t
    {
    public:
        SetLinearCount_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetLinearCount){};

        CAN_message_t operator()(int32_t pos) const
        {
            can_Message_t msg;
            can_setSignal<int32_t>(msg, pos, 0, 32, true);
            return MessageBase_t::pack(msg.buf);
        }
    };

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
              SetTrajVelLimit(node_id),
              SetTrajAccelLimits(node_id),
              SetTrajInertia(node_id),
              GetIQ(node_id),
              GetSensorlessEstimates(node_id),
              RebootOdrive(node_id),
              GetVbusVoltage(node_id),
              ClearErrors(node_id),
              SetLinearCount(node_id)

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
        SetTrajVelLimit_t SetTrajVelLimit;
        SetTrajAccelLimits_t SetTrajAccelLimits;
        SetTrajInertia_t SetTrajInertia;
        GetIQ_t GetIQ;
        GetSensorlessEstimates_t GetSensorlessEstimates;
        RebootOdrive_t RebootOdrive;
        GetVbusVoltage_t GetVbusVoltage;
        ClearErrors_t ClearErrors;
        SetLinearCount_t SetLinearCount;
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
                if (false && (msg_id != MessageID_t::ODrive_Heartbeat))
                {
                    Serial.print("decode ");
                    Serial.println(msg_id);
                }
                if (msg_id == MessageID_t::ODrive_Heartbeat)
                {
                    _nodes[i]->Heartbeat.error = (ODrive::AxisError)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 0, 32, true);
                    _nodes[i]->Heartbeat.state = (ODrive::AxisState)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 32, 32, true);
                }
                if (msg_id == MessageID_t::GetMotorError)
                {
                    _nodes[i]->GetMotorError.error = (ODrive::MotorError)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 0, 32, true); // or 64?
                }
                if (msg_id == MessageID_t::GetEncoderError)
                {
                    _nodes[i]->GetEncoderError.error = (ODrive::EncoderError)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 0, 32, true);
                }
                if (msg_id == MessageID_t::GetSensorlessError)
                {
                    _nodes[i]->GetSensorlessError.error = (ODrive::SensorlessEstimatorError)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 0, 32, true);
                }
                if (msg_id == MessageID_t::GetEncoderEstimates)
                {
                    _nodes[i]->GetEncoderEstimates.pos = can_getSignal<float>(flexcan_to_odrive(msg), 0, 32, true, 1, 0);
                    _nodes[i]->GetEncoderEstimates.vel = can_getSignal<float>(flexcan_to_odrive(msg), 32, 32, true, 1, 0);
                }
                if (msg_id == MessageID_t::GetEncoderCount)
                {
                    _nodes[i]->GetEncoderCount.shadow_count = can_getSignal<int32_t>(flexcan_to_odrive(msg), 0, 32, true, 1, 0);
                    _nodes[i]->GetEncoderCount.count_cpr = can_getSignal<int32_t>(flexcan_to_odrive(msg), 32, 32, true, 1, 0);
                }
                if (msg_id == MessageID_t::GetIQ)
                {
                    _nodes[i]->GetIQ.iq_setpoint = can_getSignal<float>(flexcan_to_odrive(msg), 0, 32, true, 1, 0);
                    _nodes[i]->GetIQ.iq_measured = can_getSignal<float>(flexcan_to_odrive(msg), 32, 32, true, 1, 0);
                }
                if (msg_id == MessageID_t::GetSensorlessEstimates)
                {
                    _nodes[i]->GetSensorlessEstimates.pos = can_getSignal<float>(flexcan_to_odrive(msg), 0, 32, true, 1, 0);
                    _nodes[i]->GetSensorlessEstimates.vel = can_getSignal<float>(flexcan_to_odrive(msg), 32, 32, true, 1, 0);
                }
                if (msg_id == MessageID_t::GetVbusVoltage)
                {
                    _nodes[i]->GetVbusVoltage.vbus = can_getSignal<float>(flexcan_to_odrive(msg), 0, 32, true, 1, 0);
                }
            }
        }
    }
private:    
    static can_Message_t flexcan_to_odrive(const CAN_message_t &flexcan_msg)
    {
        can_Message_t odrive_message;
        odrive_message.id = flexcan_msg.id;
        odrive_message.len = flexcan_msg.len;
        odrive_message.rtr = flexcan_msg.flags.remote;
        std::memcpy(odrive_message.buf, flexcan_msg.buf, flexcan_msg.len);
        return odrive_message;
    }

};


#undef GET_CANBUS_ID
#undef GET_NODE_ID
#undef GET_MSG_ID
#endif