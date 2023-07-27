#include "main.h"
#ifndef DEVICES_H
#define DEVICES_H

Controller master(E_CONTROLLER_MASTER);

// Left side motors
Motor left_front(11, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS);
Motor left_top(12, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor left_bottom(13, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS);

MotorGroup left({left_front, left_top, left_bottom});

// Right side motors
Motor right_front(20, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS);
Motor right_top(19, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor right_bottom(18, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS);

MotorGroup right({right_front, right_top, right_bottom});

Imu imu_sensor1(10);
Imu imu_sensor2(16);

#endif