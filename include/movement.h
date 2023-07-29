#include "main.h"
#ifndef DEVICES_H
#include "devices.h" // Defines motors, controller and sensors
#endif


/// @brief Points the robot to a specified angle
/// @param degrees 0 - 360
/// @param direction True: clockwise, False: counterclockwise
void turn_to_angle(float degrees, bool direction) {
    return;
}


// void drive_forward(int desired_rotations) {
	
//     float P = 1;
//     float D = 0.5;

// 	int correct_cycles = 0;

// 	float left_error = 0;
// 	float right_error = 0;
// 	float previous_left_error = 0;
// 	float previous_right_error = 0;
	
// 	float left_derivative = 0;
// 	float right_derivative = 0;

// 	while (correct_cycles < 100) {

// 		vector<double> left_positions = left.get_positions();
// 		float left_position = (left_positions[0] + left_positions[1] + left_positions[2]) / 3;
// 		vector<double> right_positions = right.get_positions();
// 		float right_position = (right_positions[0] + right_positions[1] + right_positions[2]) / 3;


// 		lcd::print(0, "Left Position: %f", left_position);
// 		lcd::print(1, "Right Position: %f", right_position);

// 		left_error = desired_rotations - left_position;
// 		right_error = desired_rotations - right_position;

// 		left_derivative = left_error - previous_left_error;
// 		right_derivative = right_error - previous_right_error;

// 		previous_left_error = left_error;
// 		previous_right_error = right_error;

// 		lcd::print(2, "Left Error: %f", left_error);
// 		lcd::print(3, "Right Error: %f", right_error);

// 		if (left_error < 0.08 && right_error < 0.08) {
// 			correct_cycles++;
// 		} else {
// 			correct_cycles = 0;
// 		}

// 		float left_power = left_error * P * 100 + left_derivative * D * 100;
// 		float right_power = right_error * P * 100 + right_derivative * D * 100;

// 		left.move(left_power);
// 		right.move(right_power);

// 		delay(5);

// 	}
// }