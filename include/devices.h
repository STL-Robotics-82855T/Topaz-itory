#pragma once

Controller master(E_CONTROLLER_MASTER);

// Left side motors
Motor left_front_top(14, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor left_front_bottom(15, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS);
Motor left_back(16, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS);

MotorGroup left({ left_front_bottom, left_front_top, left_back });

// Right side motors
Motor right_front_top(11, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor right_front_bottom(12, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS);
Motor right_back(13, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS);

MotorGroup right({ right_front_bottom, right_front_top, right_back });

// Cata motor
Motor cata_motor(1, E_MOTOR_GEAR_RED, false, E_MOTOR_ENCODER_ROTATIONS);


// Intake motor
Motor intake_motor(20, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS);

bool wing_state_1 = false;
bool wing_state_2 = false;
bool intake_state = false;
ADIDigitalOut intake_cylinders('a', intake_state);
ADIDigitalOut wing_cylinder_1('b', wing_state_1);
ADIDigitalOut wing_cylinder_2('h', wing_state_2);


Imu imu_sensor1(17);
// Imu imu_sensor2(18); // Traded to Y Team for pneumatics

// Rotation horizontal_tracker(2);
// Rotation right_tracker(3);
Rotation cata_tracker(10, true);
