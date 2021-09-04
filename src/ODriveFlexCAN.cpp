#include "ODriveFlexCAN.h"

#include "can_helpers.hpp" // can_Message_t, can_getSignal, can_setSignal (not the same can_Message_t as FlexCAN!)

// Upper 6 bits - Node ID (axis id)
// Lower 5 bits - Command ID
// can_id = axis_id << 5 | cmd_id
// node_id = can_id >> 5
// cmd_id = can_id & 0b00011111 (0x1F)
#define GET_CANBUS_ID(node_id, msg_id) ((node_id << 5) + msg_id)
#define GET_NODE_ID(canbus_id) (canbus_id >> 5)
#define GET_MSG_ID(canbus_id) (canbus_id & 0x1F)

ODriveFlexCAN::ODriveFlexCAN(const uint32_t *node_ids, uint32_t node_count)
    : _node_count(node_count)
{
    for (uint32_t i = 0; i < _node_count; i++)
    {
        _nodes[i] = new ODriveNode_t(node_ids[i]);
    }
}

const ODriveFlexCAN::ODriveNode_t &ODriveFlexCAN::operator()(uint32_t node_id) const
{
    Serial.print("flexcan() _node_count: ");
    Serial.println(_node_count);
    for (uint32_t i = 0; i < _node_count; i++)
        if (_nodes[i]->getNodeId() == node_id)
            return *_nodes[i];

    return *_nodes[0]; // for compiler warning
}

can_Message_t flexcan_to_odrive(const CAN_message_t &flexcan_msg)
{
    can_Message_t odrive_message;
    odrive_message.id = flexcan_msg.id;
    odrive_message.len = flexcan_msg.len;
    odrive_message.rtr = flexcan_msg.flags.remote;
    std::memcpy(odrive_message.buf, flexcan_msg.buf, flexcan_msg.len);
    return odrive_message;
}

void ODriveFlexCAN::filter(const CAN_message_t &msg)
{
    uint32_t node_id = GET_NODE_ID(msg.id);
    uint32_t msg_id = GET_MSG_ID(msg.id);

    for (uint32_t i = 0; i < _node_count; i++)
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
                _nodes[i]->Heartbeat.error = (AxisError)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 0, 32, true);
                _nodes[i]->Heartbeat.state = (AxisState)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 32, 32, true);
            }
            if (msg_id == MessageID_t::GetMotorError)
            {
                _nodes[i]->GetMotorError.error = (MotorError)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 0, 32, true); // or 64?
            }
            if (msg_id == MessageID_t::GetEncoderError)
            {
                _nodes[i]->GetEncoderError.error = (EncoderError)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 0, 32, true);
            }
            if (msg_id == MessageID_t::GetSensorlessError)
            {
                _nodes[i]->GetSensorlessError.error = (SensorlessEstimatorError)can_getSignal<uint32_t>(flexcan_to_odrive(msg), 0, 32, true);
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

CAN_message_t ODriveFlexCAN::MessageBase_t::pack() const
{
    uint8_t zero[8] = {0};
    return pack(zero);
}
CAN_message_t ODriveFlexCAN::MessageBase_t::pack(const uint8_t *buf) const
{
    CAN_message_t msg;
    msg.id = GET_CANBUS_ID(_node_id, _message_id);
    msg.len = 8;
    std::memcpy(msg.buf, buf, msg.len);
    msg.flags.remote = is_rtr_message(_message_id);
    return msg;
}

CAN_message_t ODriveFlexCAN::SetAxisRequestedState_t::operator()(AxisState requested_state) const
{
    can_Message_t msg;
    can_setSignal<uint32_t>(msg, (uint32_t)requested_state, 0, 32, true);
    return MessageBase_t::pack(msg.buf);
}
CAN_message_t ODriveFlexCAN::SetControllerModes_t::operator()(ControlMode control_mode, InputMode input_mode) const
{
    can_Message_t msg;
    can_setSignal<uint32_t>(msg, (uint32_t)control_mode, 0, 32, true);
    can_setSignal<uint32_t>(msg, (uint32_t)input_mode, 32, 32, true);
    return MessageBase_t::pack(msg.buf);
}
CAN_message_t ODriveFlexCAN::SetInputPos_t::operator()(float input_pos, int16_t vel_ff, int16_t torque_ff) const
{
    can_Message_t msg;
    can_setSignal<float>(msg, input_pos, 0, 32, true, 1, 0);
    can_setSignal<int16_t>(msg, vel_ff, 32, 16, true, 0.001, 0);
    can_setSignal<int16_t>(msg, torque_ff, 48, 16, true, 0.001, 0);
    return MessageBase_t::pack(msg.buf);
}
CAN_message_t ODriveFlexCAN::SetInputVel_t::operator()(float input_vel, float torque_ff) const
{
    can_Message_t msg;
    can_setSignal<float>(msg, input_vel, 0, 32, true, 1, 0);
    can_setSignal<float>(msg, torque_ff, 32, 32, true, 1, 0);
    return MessageBase_t::pack(msg.buf);
}
CAN_message_t ODriveFlexCAN::SetInputTorque_t::operator()(float input_torque) const
{
    can_Message_t msg;
    can_setSignal<float>(msg, input_torque, 0, 32, true, 1, 0);
    return MessageBase_t::pack(msg.buf);
}
CAN_message_t ODriveFlexCAN::SetLimits_t::operator()(float vel_lim, float cur_lim) const
{
    can_Message_t msg;
    can_setSignal<float>(msg, vel_lim, 0, 32, true, 1, 0);
    can_setSignal<float>(msg, cur_lim, 32, 32, true, 1, 0);
    return MessageBase_t::pack(msg.buf);
}
CAN_message_t ODriveFlexCAN::SetTrajVelLimit_t::operator()(float traj_vel_lim) const
{
    can_Message_t msg;
    can_setSignal<float>(msg, traj_vel_lim, 0, 32, true, 1, 0);
    return MessageBase_t::pack(msg.buf);
}
CAN_message_t ODriveFlexCAN::SetTrajAccelLimits_t::operator()(float traj_accel_lim, float traj_decel_lim) const
{
    can_Message_t msg;
    can_setSignal<float>(msg, traj_accel_lim, 0, 32, true, 1, 0);
    can_setSignal<float>(msg, traj_decel_lim, 32, 32, true, 1, 0);
    return MessageBase_t::pack(msg.buf);
}
CAN_message_t ODriveFlexCAN::SetTrajInertia_t::operator()(float traj_inertia) const
{
    can_Message_t msg;
    can_setSignal<float>(msg, traj_inertia, 0, 32, true, 1, 0);
    return MessageBase_t::pack(msg.buf);
}
CAN_message_t ODriveFlexCAN::SetLinearCount_t::operator()(int32_t pos) const
{
    can_Message_t msg;
    can_setSignal<int32_t>(msg, pos, 0, 32, true);
    return MessageBase_t::pack(msg.buf);
}
