#include "main.h"
#ifndef DEVICES_H
#define DEVICES_H

Controller master(E_CONTROLLER_MASTER);

// MOTORS					P#	Cartridge			Invert	Encoder units
// Cata motor
Motor cata_motor(			10,	E_MOTOR_GEAR_RED,	false,	E_MOTOR_ENCODER_ROTATIONS);

// Right side motors
Motor right_front_top(		11,	E_MOTOR_GEAR_BLUE,	true,	E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor right_front_bottom(	12,	E_MOTOR_GEAR_BLUE,	false,	E_MOTOR_ENCODER_ROTATIONS);
Motor right_back(			13,	E_MOTOR_GEAR_BLUE,	false,	E_MOTOR_ENCODER_ROTATIONS);

// Left side motors
Motor left_front_top(		14,	E_MOTOR_GEAR_BLUE,	false,	E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor left_front_bottom(	15,	E_MOTOR_GEAR_BLUE,	true,	E_MOTOR_ENCODER_ROTATIONS);
Motor left_back(			16,	E_MOTOR_GEAR_BLUE,	true,	E_MOTOR_ENCODER_ROTATIONS);

// Intake motor
Motor intake_motor(			20,	E_MOTOR_GEAR_GREEN,	false,	E_MOTOR_ENCODER_ROTATIONS);

// Motor groups
MotorGroup left({left_front_bottom, left_front_top, left_back});
MotorGroup right({right_front_bottom, right_front_top, right_back});

bool state = false;
ADIDigitalOut intake_cylinders('a', state);

// IMUs			P#
Imu imu_sensor1(17);
Imu imu_sensor2(18);

// ROTATION SENSORS				P#	Invert
Rotation horizontal_tracker(	2);
Rotation right_tracker(			3);
Rotation cata_tracker(			4,	true);

#endif
