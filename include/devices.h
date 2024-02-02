#pragma once

Controller master(E_CONTROLLER_MASTER);

// Left side motors
Motor left_back(11, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor left_middle(12, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS);
Motor left_front(17, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS);

MotorGroup left({left_middle, left_front, left_back});

// Right side motors
Motor right_back(18, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS); // Top is stacked on top of bottom
Motor right_middle(19, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS);
Motor right_front(20, E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_ROTATIONS);

MotorGroup right({ right_middle, right_front, right_back });

// Cata motor
Motor cata_motor(16, E_MOTOR_GEAR_RED, false, E_MOTOR_ENCODER_ROTATIONS);


// Intake motor
Motor intake_motor(10, E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_ROTATIONS);

bool wing_global_state = false; // to ensure both wings are in the same state when driving

bool wing_state_1 = false;
bool wing_state_2 = false;
// bool intake_state = false;
// ADIDigitalOut intake_cylinders('a', intake_state);
ADIDigitalOut wing_cylinder_1('a', wing_state_1);
ADIDigitalOut wing_cylinder_2('b', wing_state_2);


bool blocker_state = false;
ADIDigitalOut blocker_cylinders('d', blocker_state);

bool hang_state = false;
ADIDigitalOut hang_cylinder('c', hang_state);

bool side_hang_state = false;
ADIDigitalOut side_hang_cylinder('e', side_hang_state);

Imu imu_sensor1(14);
// Imu imu_sensor2(18); // Traded to Y Team for pneumatics

// Rotation horizontal_tracker(2);
Rotation odom_tracker(5, true);
// Rotation cata_tracker(10, true);

Distance vertical_distance(15);


