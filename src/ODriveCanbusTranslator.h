#ifndef ODriveCanbusTranslator_h
#define ODriveCanbusTranslator_h

#include "Arduino.h"

#include "ODriveEnums.h"
#include "CAN-Helpers/can_helpers.hpp"

#define ODRIVE_MAX_NODES 100

// Upper 6 bits - Node ID (axis id)
// Lower 5 bits - Command ID
// can_id = axis_id << 5 | cmd_id
// node_id = can_id >> 5
// cmd_id = can_id & 0b00011111 (0x1F)
#define GET_CANBUS_ID(node_id, msg_id) ((node_id << 5) + msg_id)
#define GET_NODE_ID(canbus_id) (canbus_id >> 5)
#define GET_MSG_ID(canbus_id) (canbus_id & 0x1F)

#define CONSTRUCTORS(user_class)                                          \
    user_class(uint32_t node_id = 0) : ODriveCanbusTranslator(node_id) {} \
    user_class(const uint32_t *node_ids, uint32_t node_count) : ODriveCanbusTranslator(node_ids, node_count) {}

template <typename T>
class ODriveCanbusTranslator
{
public:
    ODriveCanbusTranslator(uint32_t node_id)
        : _node_count(1)
    {
        _nodes[0] = new ODriveNode_t(node_id, *this);
    }
    ODriveCanbusTranslator(const uint32_t *node_ids, uint32_t node_count)
        : _node_count(node_count)
    {
        for (uint32_t i = 0; i < _node_count; i++)
        {
            _nodes[i] = new ODriveNode_t(node_ids[i], *this);
        }
    }
    void filter(uint32_t id, uint8_t len, const uint8_t *buf)
    {
        uint32_t node_id = GET_NODE_ID(id);
        uint32_t msg_id = GET_MSG_ID(id);

        can_Message_t msg;
        msg.id = id;
        msg.len = len;
        std::memcpy(msg.buf, buf, len);

        for (uint32_t i = 0; i < _node_count; i++)
        {
            if (_nodes[i]->getNodeId() == node_id)
            {
                if (msg_id == MessageID_t::ODrive_Heartbeat)
                {
                    _nodes[i]->Heartbeat.error = (AxisError)can_getSignal<uint32_t>(msg, 0, 32, true);
                    _nodes[i]->Heartbeat.state = (AxisState)can_getSignal<uint32_t>(msg, 32, 32, true);
                }
                if (msg_id == MessageID_t::GetMotorError)
                {
                    _nodes[i]->GetMotorError.error = (MotorError)can_getSignal<uint32_t>(msg, 0, 32, true); // or 64?
                }
                if (msg_id == MessageID_t::GetEncoderError)
                {
                    _nodes[i]->GetEncoderError.error = (EncoderError)can_getSignal<uint32_t>(msg, 0, 32, true);
                }
                if (msg_id == MessageID_t::GetSensorlessError)
                {
                    _nodes[i]->GetSensorlessError.error = (SensorlessEstimatorError)can_getSignal<uint32_t>(msg, 0, 32, true);
                }
                if (msg_id == MessageID_t::GetEncoderEstimates)
                {
                    _nodes[i]->GetEncoderEstimates.pos = can_getSignal<float>(msg, 0, 32, true, 1, 0);
                    _nodes[i]->GetEncoderEstimates.vel = can_getSignal<float>(msg, 32, 32, true, 1, 0);
                }
                if (msg_id == MessageID_t::GetEncoderCount)
                {
                    _nodes[i]->GetEncoderCount.shadow_count = can_getSignal<int32_t>(msg, 0, 32, true, 1, 0);
                    _nodes[i]->GetEncoderCount.count_cpr = can_getSignal<int32_t>(msg, 32, 32, true, 1, 0);
                }
                if (msg_id == MessageID_t::GetIQ)
                {
                    _nodes[i]->GetIQ.iq_setpoint = can_getSignal<float>(msg, 0, 32, true, 1, 0);
                    _nodes[i]->GetIQ.iq_measured = can_getSignal<float>(msg, 32, 32, true, 1, 0);
                }
                if (msg_id == MessageID_t::GetSensorlessEstimates)
                {
                    _nodes[i]->GetSensorlessEstimates.pos = can_getSignal<float>(msg, 0, 32, true, 1, 0);
                    _nodes[i]->GetSensorlessEstimates.vel = can_getSignal<float>(msg, 32, 32, true, 1, 0);
                }
                if (msg_id == MessageID_t::GetVbusVoltage)
                {
                    _nodes[i]->GetVbusVoltage.vbus = can_getSignal<float>(msg, 0, 32, true, 1, 0);
                }
            }
        }
    }

protected:
    virtual T user_pack(uint32_t id, uint8_t len, const uint8_t *buf, bool rtr) = 0;

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
        return  message_id == MessageID_t::GetMotorError ||
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
    protected:
        MessageBase_t(uint32_t node_id, uint32_t message_id, ODriveCanbusTranslator &owner)
            : _node_id(node_id),
              _message_id(message_id),
              _owner(&owner) {};

    public:
        T operator()() const { return MessageBase_t::pack(); }

    protected:
        const uint32_t _node_id;
        const uint32_t _message_id;

        T pack() const
        {
            uint8_t zero[8] = {0};
            return pack(zero);
        }
        T pack(const uint8_t *buf) const
        {
            return _owner->user_pack(GET_CANBUS_ID(_node_id, _message_id), 8, buf, is_rtr_message(_message_id));
        }
    private:
        ODriveCanbusTranslator<T> *const _owner;
    };
    class Heartbeat_t : public MessageBase_t
    {
    public:
        AxisError error;
        AxisState state;

        Heartbeat_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::ODrive_Heartbeat, owner),
              error(AxisError::AXIS_ERROR_NONE),
              state(AxisState::AXIS_STATE_UNDEFINED){};
    };
    class EStop_t : public MessageBase_t
    {
    public:
        EStop_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::ODrive_EStop, owner){};
    };
    class GetMotorError_t : public MessageBase_t
    {
    public:
        MotorError error;
        GetMotorError_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::GetMotorError, owner),
              error(MotorError::MOTOR_ERROR_NONE){};
    };
    class GetEncoderError_t : public MessageBase_t
    {
    public:
        EncoderError error;
        GetEncoderError_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::GetEncoderError, owner),
              error(EncoderError::ENCODER_ERROR_NONE){};
    };
    class GetSensorlessError_t : public MessageBase_t
    {
    public:
        SensorlessEstimatorError error;
        GetSensorlessError_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::GetSensorlessError, owner),
              error(SensorlessEstimatorError::SENSORLESS_ESTIMATOR_ERROR_NONE){};
    };
    // class SetAxisNodeID_t {};
    class SetAxisRequestedState_t : public MessageBase_t
    {
    public:
        SetAxisRequestedState_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetAxisRequestedState, owner){};

        T operator()(AxisState requested_state) const
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
        GetEncoderEstimates_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::GetEncoderEstimates, owner),
              pos(0),
              vel(0){};
    };
    class GetEncoderCount_t : public MessageBase_t
    {
    public:
        int32_t shadow_count, count_cpr;
        GetEncoderCount_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::GetEncoderCount, owner),
              shadow_count(0),
              count_cpr(0){};
    };
    class SetControllerModes_t : public MessageBase_t
    {
    public:
        SetControllerModes_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetControllerModes, owner){};

        T operator()(ControlMode control_mode, InputMode input_mode) const
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
        SetInputPos_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetInputPos, owner){};

        T operator()(float input_pos, int16_t vel_ff = 0, int16_t torque_ff = 0) const
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
        SetInputVel_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetInputVel, owner){};

        T operator()(float input_vel, float torque_ff = 0) const
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
        SetInputTorque_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetInputTorque, owner){};

        T operator()(float input_torque) const
        {
            can_Message_t msg;
            can_setSignal<float>(msg, input_torque, 0, 32, true, 1, 0);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class SetLimits_t : public MessageBase_t
    {
    public:
        SetLimits_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetLimits, owner){};

        T operator()(float vel_lim, float cur_lim) const
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
        StartAnticogging_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::StartAnticogging, owner){};
    };
    class SetTrajVelLimit_t : public MessageBase_t
    {
    public:
        SetTrajVelLimit_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetTrajVelLimit, owner){};

        T operator()(float traj_vel_lim) const
        {
            can_Message_t msg;
            can_setSignal<float>(msg, traj_vel_lim, 0, 32, true, 1, 0);
            return MessageBase_t::pack(msg.buf);
        }
    };
    class SetTrajAccelLimits_t : public MessageBase_t
    {
    public:
        SetTrajAccelLimits_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetTrajAccelLimits, owner){};

        T operator()(float traj_accel_lim, float traj_decel_lim) const
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
        SetTrajInertia_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetTrajInertia, owner){};

        T operator()(float traj_inertia) const
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
        GetIQ_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::GetIQ, owner),
              iq_setpoint(0),
              iq_measured(0){};
    };
    class GetSensorlessEstimates_t : public MessageBase_t
    {
    public:
        float pos, vel;
        GetSensorlessEstimates_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::GetSensorlessEstimates, owner),
              pos(0),
              vel(0){};
    };
    class RebootOdrive_t : public MessageBase_t
    {
    public:
        RebootOdrive_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::RebootODrive, owner){};
    };
    class GetVbusVoltage_t : public MessageBase_t
    {
    public:
        float vbus;
        GetVbusVoltage_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::GetVbusVoltage, owner),
              vbus(0){};
    };
    class ClearErrors_t : public MessageBase_t
    {
    public:
        ClearErrors_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::ClearErrors, owner){};
    };
    class SetLinearCount_t : public MessageBase_t
    {
    public:
        SetLinearCount_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : MessageBase_t(node_id, MessageID_t::SetLinearCount, owner){};

        T operator()(int32_t pos) const
        {
            can_Message_t msg;
            can_setSignal<int32_t>(msg, pos, 0, 32, true);
            return MessageBase_t::pack(msg.buf);
        }
    };

private:
    class ODriveNode_t
    {
    private:
        uint32_t _node_id;
    public:
        uint32_t getNodeId() { return _node_id; };

    public:
        ODriveNode_t(uint32_t node_id, ODriveCanbusTranslator &owner)
            : _node_id(node_id),
              Heartbeat(node_id, owner),
              EStop(node_id, owner),
              GetMotorError(node_id, owner),
              GetEncoderError(node_id, owner),
              GetSensorlessError(node_id, owner),
              //SetAxisNodeID(node_id, owner),
              SetAxisRequestedState(node_id, owner),
              //SetAxisStartupConfig(node_id, owner),
              GetEncoderEstimates(node_id, owner),
              GetEncoderCount(node_id, owner),
              SetControllerModes(node_id, owner),
              SetInputPos(node_id, owner),
              SetInputVel(node_id, owner),
              SetInputTorque(node_id, owner),
              SetLimits(node_id, owner),
              StartAnticogging(node_id, owner),
              SetTrajVelLimit(node_id, owner),
              SetTrajAccelLimits(node_id, owner),
              SetTrajInertia(node_id, owner),
              GetIQ(node_id, owner),
              GetSensorlessEstimates(node_id, owner),
              RebootOdrive(node_id, owner),
              GetVbusVoltage(node_id, owner),
              ClearErrors(node_id, owner),
              SetLinearCount(node_id, owner){};

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

private:
    ODriveNode_t *_nodes[ODRIVE_MAX_NODES];
    const uint32_t _node_count;

public:
    const ODriveNode_t &operator()(uint32_t node_id) const
    {
        for (uint32_t i = 0; i < _node_count; i++)
            if (_nodes[i]->getNodeId() == node_id)
                return *_nodes[i];

        return *_nodes[0]; // for compiler warning
    }
};

#endif
