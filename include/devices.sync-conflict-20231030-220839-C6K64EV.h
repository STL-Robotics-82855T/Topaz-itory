#include "main.h"
#ifndef DEVICES_H
#define DEVICES_H

Controller master(E_CONTROLLER_MASTER);

// Left side motors
Motor left_front_bottom(5, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS);
Motor left_front_top(6, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor left_back(4, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS);

MotorGroup left({left_front_bottom, left_front_top, left_back});

// Right side motors
Motor right_front_bottom(1, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS);
Motor right_front_top(3, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor right_back(2, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS);

MotorGroup right({right_front_bottom, right_front_top, right_back});

// Cata motor
Motor cata_motor(11, E_MOTOR_GEAR_RED, false, E_MOTOR_ENCODER_ROTATIONS);

Imu imu_sensor1(12);
Imu imu_sensor2(13);

// Rotation horizontal_tracker(2);
// Rotation right_tracker(3);
// Rotation cata_tracker(4, true);

#endif