#ifndef MOVEMENT_H
#define MOVEMENT_H
#include "main.h"
#include "./movement/odometry.h" // Defines odometry class
#include "devices.h"			 // Defines motors, controller and sensors

odometry odom(6.02, 6.02, 0.73, 3.25);

/// @brief Turns n degrees clockwise (accepts negative) (Blocking)
/// @param target_heading 0.0 - 360.0 degrees (0 is forward) (All degrees relative to the starting position of the robot)
/// @param timeout (Optional) Time in sec to kill the function if it doesn't exit
void turn_to_angle_auton(float target_heading, float timeout = -1) {

	// PID constants
	float P = 4.2;
	float I = 0.03;
	float D = 1;
	float current_error = 0;
	float previous_error = 0;
	float build_up_error = 0;
	float allowed_error = 1; // degrees of error allowed
	float power;

	while (abs(current_error) > allowed_error || abs(left[1].get_actual_velocity()) > 15 || abs(right[0].get_actual_velocity()) > 15) {
		current_error = abs(target_heading - odom.current_heading_deg);
		if (current_error > 180) { // If the error is greater than 180 degrees, the robot should turn the other way
			current_error = 360 - current_error;
		}

		// Check which way the robot should turn
		bool turn_right = true;
		if (odom.current_heading_deg < target_heading) { // If the robot is to the left of the target
			turn_right = (target_heading - odom.current_heading_deg < 180); // If the target is less than 180 degrees away going clockwise is faster
		} else {
			turn_right = (odom.current_heading_deg - target_heading > 180); // If the target is more than 180 degrees away going counter-clockwise is faster
		}

		if (turn_right) {
			current_error *= -1;
		}

		if (abs(current_error) < 10) {
			build_up_error += current_error;
		}
		power = (current_error * P) + (build_up_error * I) + ((current_error - previous_error) * D);
		previous_error = current_error;

		left.move(-power);
		right.move(power);

		delay(5);

	}

	left.move(0);
	right.move(0);
}

/// @brief Drives in a straight line for a specified distance (Blocking)
/// @param target_inches Distance to travel in inches
/// @param speed Speed to travel at (0 - 100)
/// @param timeout (Optional) Time in sec to kill the function if it doesn't exit
void drive_line_auton(float target_inches, float speed, float timeout = -1) {


	float wheel_distance_per_rotation = 3.25 * PI * (36.0 / 60.0); // inches per rotation

	// PID constants
	float P = 4.2;
	float I = 0.025;
	float D = 1.25;
	float allowed_error = 0.5; // degrees of error allowed


	float start_left_position = left[1].get_position();
	float start_right_position = right[0].get_position();

	float current_error_left;
	float previous_error_left = 0;
	float build_up_error_left = 0;
	float current_error_right;
	float previous_error_right = 0;
	float build_up_error_right = 0;

	float power_right;
	float power_left;


	while (abs(current_error_left) > allowed_error || abs(start_right_position) > allowed_error || abs(left[1].get_actual_velocity()) > 20 || abs(right[0].get_actual_velocity()) > 20) {
		current_error_right = target_inches - ((right[0].get_position() - start_right_position) * wheel_distance_per_rotation);
		if (abs(current_error_right) < 5) { // If the error is less than 5 inches, start building up the error (avoids windup)
			build_up_error_right += current_error_right;
		}
		power_right = (current_error_right * P) + (build_up_error_right * I) + ((current_error_right - previous_error_right) * D);
		previous_error_right = current_error_right;

		current_error_left = target_inches - ((left[1].get_position() - start_left_position) * wheel_distance_per_rotation);
		if (abs(current_error_left) < 5) { // If the error is less than 5 inches, start building up the error (avoids windup)
			build_up_error_left += current_error_left;
		}
		power_left = (current_error_left * P) + (build_up_error_left * I) + ((current_error_left - previous_error_left) * D);
		previous_error_left = current_error_left;

		right.move(power_right);
		left.move(power_left);

		delay(5);

	}

	left.move(0);
	right.move(0);

}
#endif
