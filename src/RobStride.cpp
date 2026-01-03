#include "RobStride.h"

RobStrideMotor::RobStrideMotor(uint8_t id, uint8_t master) : motor_id(id), master_id(master) {}

int RobStrideMotor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void RobStrideMotor::enable()
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x03 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;
    memset(frame.data, 0, 8);
    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::stop()
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x04 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;
    memset(frame.data, 0, 8);
    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::clear_fault()
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x11 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;
    memset(frame.data, 0, 8);
    frame.data[0] = 1; // Clear Fault Command
    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::set_zero_position()
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x06 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 0;
    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::write_parameter(uint16_t index, float value)
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x12 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;

    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    frame.data[2] = 0;
    frame.data[3] = 0;
    memcpy(&frame.data[4], &value, 4);

    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::write_parameter(uint16_t index, uint8_t value)
{
    CanFrame frame;
    frame.identifier = ((uint32_t)0x12 << 24) | ((uint32_t)master_id << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;

    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = value;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::send_motion_command(float position, float velocity, float kp, float kd, float torque)
{
    CanFrame frame;
    uint16_t torque_int = float_to_uint(torque, RS_T_MIN, RS_T_MAX, 16);

    frame.identifier = ((uint32_t)0x01 << 24) | ((uint32_t)torque_int << 8) | motor_id;
    frame.extd = true;
    frame.data_length_code = 8;

    uint16_t p_int = float_to_uint(position, RS_P_MIN, RS_P_MAX, 16);
    uint16_t v_int = float_to_uint(velocity, RS_V_MIN, RS_V_MAX, 16);
    uint16_t kp_int = float_to_uint(kp, RS_KP_MIN, RS_KP_MAX, 16);
    uint16_t kd_int = float_to_uint(kd, RS_KD_MIN, RS_KD_MAX, 16);

    frame.data[0] = p_int >> 8;
    frame.data[1] = p_int & 0xFF;
    frame.data[2] = v_int >> 8;
    frame.data[3] = v_int & 0xFF;
    frame.data[4] = kp_int >> 8;
    frame.data[5] = kp_int & 0xFF;
    frame.data[6] = kd_int >> 8;
    frame.data[7] = kd_int & 0xFF;

    ESP32Can.writeFrame(&frame);
}

void RobStrideMotor::set_current_reference(float current)
{
    write_parameter(RS_IDX_IQ_REF, current);
}

void RobStrideMotor::set_speed_reference(float speed)
{
    write_parameter(RS_IDX_SPD_REF, speed);
}

void RobStrideMotor::set_position_reference(float position)
{
    write_parameter(RS_IDX_LOC_REF, position);
}

void RobStrideMotor::set_run_mode(uint8_t mode)
{
    write_parameter(RS_IDX_RUN_MODE, mode);
}

void RobStrideMotor::set_limit_current(float current)
{
    write_parameter(RS_IDX_LIMIT_CUR, current);
}

void RobStrideMotor::set_limit_speed(float speed)
{
    write_parameter(RS_IDX_LIMIT_SPD, speed);
}

void RobStrideMotor::set_limit_accel_rad(float accel)
{
    write_parameter(RS_IDX_ACC_RAD, accel);
}

void RobStrideMotor::set_pp_limits(float velocity, float accel)
{
    write_parameter(RS_IDX_VEL_MAX, velocity);
    write_parameter(RS_IDX_ACC_SET, accel);
}
