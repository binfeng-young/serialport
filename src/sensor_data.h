//
// Created by binfeng.yang on 2021/4/14.
//
#include <cstdint>

#ifndef SERIALPORT_SENSOR_DATA_H
#define SERIALPORT_SENSOR_DATA_H
struct SensorData {
    uint32_t encoder_t;
    uint32_t encoder_l;
    uint32_t encoder_r;
    uint32_t imu_t;
    int16_t  a_x;
    int16_t  a_y;
    int16_t  a_z;
    int16_t  w_x;
    int16_t  w_y;
    int16_t  w_z;
    int16_t  Roll;
    int16_t  Pitch;
    int16_t  Yaw;
    uint8_t  right_wall;
    uint8_t  cliff_and_bump;
    uint8_t  led_status;
    uint8_t  charge_state;
    uint16_t current_fan;
    uint8_t  off_ground;
    uint8_t  left_piles;
    uint8_t  right_piles;
    uint16_t current_left_wheel_avg;
    uint16_t current_right_wheel_avg;
    uint16_t current_roll;
    uint8_t  current_left_side;
    uint8_t  current_right_side;
    uint16_t current_left_wheel;
    uint16_t current_right_wheel;
    uint8_t  ir_state;
    uint8_t  current_lidar;
    uint8_t  left_wall;
    uint8_t  carpet;
    uint8_t  lidar_collide;
    uint16_t ultrasoinc_value;
    uint8_t  front_left_piles;
    uint8_t  front_right_piles;
    uint8_t  left_front_collide;
    uint8_t  mid_front_collide;
    uint8_t  right_front_collide;
};
#endif // SERIALPORT_SENSOR_DATA_H
