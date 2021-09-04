#ifndef ODriveFlexCAN_h
#define ODriveFlexCAN_h

#include "Arduino.h"

#include "FlexCAN_T4.h" // CAN_message_t
#include "ODriveEnums.h"

#define ODRIVE_MAX_NODES 100

class ODriveFlexCAN
{
public:
    ODriveFlexCAN(uint32_t *node_ids);
    void filter(const CAN_message_t &msg);

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
        MessageBase_t(uint32_t node_id, uint32_t message_id)
            : _node_id(node_id),
              _message_id(message_id){};
    public:
        CAN_message_t operator()() const { return MessageBase_t::pack(); }

    protected:
        const uint32_t _node_id;
        const uint32_t _message_id;

        CAN_message_t pack() const;
        CAN_message_t pack(const uint8_t *buf) const;
    };
    class Heartbeat_t : public MessageBase_t
    {
    public:
        AxisError error;
        AxisState state;

        Heartbeat_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::ODrive_Heartbeat),
              error(AxisError::AXIS_ERROR_NONE),
              state(AxisState::AXIS_STATE_UNDEFINED){};
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
        MotorError error;
        GetMotorError_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetMotorError),
              error(MotorError::MOTOR_ERROR_NONE){};
    };
    class GetEncoderError_t : public MessageBase_t
    {
    public:
        EncoderError error;
        GetEncoderError_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetEncoderError),
              error(EncoderError::ENCODER_ERROR_NONE){};
    };
    class GetSensorlessError_t : public MessageBase_t
    {
    public:
        SensorlessEstimatorError error;
        GetSensorlessError_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::GetSensorlessError),
              error(SensorlessEstimatorError::SENSORLESS_ESTIMATOR_ERROR_NONE){};
    };
    // class SetAxisNodeID_t {};
    class SetAxisRequestedState_t : public MessageBase_t
    {
    public:
        SetAxisRequestedState_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetAxisRequestedState){};

        CAN_message_t operator()(AxisState requested_state) const;
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

        CAN_message_t operator()(ControlMode control_mode, InputMode input_mode) const;
    };
    class SetInputPos_t : public MessageBase_t
    {
    public:
        SetInputPos_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputPos){};

        CAN_message_t operator()(float input_pos, int16_t vel_ff = 0, int16_t torque_ff = 0) const;
    };
    class SetInputVel_t : public MessageBase_t
    {
    public:
        SetInputVel_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputVel){};

        CAN_message_t operator()(float input_vel, float torque_ff = 0) const;
    };
    class SetInputTorque_t : public MessageBase_t
    {
    public:
        SetInputTorque_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetInputTorque){};

        CAN_message_t operator()(float input_torque) const;
    };
    class SetLimits_t : public MessageBase_t
    {
    public:
        SetLimits_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetLimits){};

        CAN_message_t operator()(float vel_lim, float cur_lim) const;
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

        CAN_message_t operator()(float traj_vel_lim) const;
    };
    class SetTrajAccelLimits_t : public MessageBase_t
    {
    public:
        SetTrajAccelLimits_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetTrajAccelLimits){};

        CAN_message_t operator()(float traj_accel_lim, float traj_decel_lim) const;
    };
    class SetTrajInertia_t : public MessageBase_t
    {
    public:
        SetTrajInertia_t(uint32_t node_id)
            : MessageBase_t(node_id, MessageID_t::SetTrajInertia){};

        CAN_message_t operator()(float traj_inertia) const;
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

        CAN_message_t operator()(int32_t pos) const;
    };

private:
    class ODriveNode_t
    {
    private:
        uint32_t _node_id;
    public:
        uint32_t getNodeId() { return _node_id; };

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
              SetLinearCount(node_id) {};

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
    const ODriveNode_t &operator()(uint32_t node_id) const;
};

#endif
