#ifndef MOVEMENT_H
#define MOVEMENT_H
#include "main.h"
#include "./movement/odometry.h" // Defines odometry class
#ifndef DEVICES_H
#include "devices.h" // Defines motors, controller and sensors
#endif

odometry odom(6.02, 6.02, 0.73, 3.25);

/// @brief Turns n degrees clockwise (accepts negative) (Blocking)
/// @param degrees 0.0 - 360.0 degrees (0 is forward) (All degrees relative to the starting position of the robot)
/// @param timeout (Optional) Time in sec to kill the function if it doesn't exit
void turn_to_angle_auton(float degrees, float timeout = -1)
{

	float P = 5;
	float I = 0.6;
	float D = 0.6;

	// How long the robot has to be within the error range to exit the function in cycles
	int wait_time = 100; // 100 * 5ms = 500ms
	int correct_cycles = 0;

	float previous_angle_error = 0;
	float derivative = 0;
	float power = 0;

	// Calculate error
	float error = degrees - odom.current_angle_deg;
	float integral = (error * 0.005);

	time_t start_time = std::time(nullptr);

	while (correct_cycles < wait_time)
	{

		// Calculate derivative
		derivative = error - previous_angle_error;
		previous_angle_error = error;

		// Calculate integral
		integral += (error * 0.5);
		if (integral > 100)
		{
			integral = 100;
		}
		else if (integral < -100)
		{
			integral = -100;
		}

		// Calculate power
		power = error * P + derivative * D * 100 + integral * I;



		// Set motors
		left.move(power);
		right.move(-power);

		// Calculate error
		error = degrees - odom.current_angle_deg;

		// Check if the function should exit

		if (timeout != -1)
		{
			if (std::time(nullptr) < (start_time + timeout))
			{
				break;
			}
		}

		if (error < 1.5 && error > -1.5)
		{ // Within -0.5 and 0.5 degrees of the target
			correct_cycles++;
		}
		else
		{
			correct_cycles = 0;
		}

		// master.print(0,0, "cycles %d", correct_cycles);
		cout << "error " << error << endl;
		// cout << "power " << power << endl;


		delay(5);
	}

	// left.move(0);
	// right.move(0);

	return;
}

/// @brief Drives in a straight line for a specified distance (Blocking)
/// @param inches Distance to travel in inches
/// @param speed Speed to travel at (0 - 100)
/// @param timeout (Optional) Time in sec to kill the function if it doesn't exit
void drive_line_auton(float inches, float speed, float timeout = -1)
{

	int index = 0;

	float P = 1;
	float D = 0.5;

	float previous_error = 0;
	float derivative = 0;
	float power = 0;

	// Calculate error
	float wheel_distance_per_rotation = 3.25 * PI * (36.0 / 60.0); // inches per rotation

	// How long the robot has to be within the error range to exit the function in cycles
	int wait_time = 100; // 100 * 5ms = 500ms
	int correct_cycles = 0;

	float start_left_position = left.get_positions()[1];
	float start_right_position = right.get_positions()[0];

	// Error is measured in inches
	float error = inches - (((left.get_positions()[1] - start_left_position) * wheel_distance_per_rotation + (right.get_positions()[0] - start_right_position) * wheel_distance_per_rotation) / 2);

	time_t start_time = std::time(nullptr);

	while (correct_cycles < wait_time)
	{

		// Calculate derivative
		derivative = error - previous_error;
		previous_error = error;

		// Calculate power
		power = error * P + derivative * D;

		// Set motors
		left.move(power);
		right.move(power);

		// Calculate error
		error = inches - (((left.get_positions()[1] - start_left_position) * wheel_distance_per_rotation + (right.get_positions()[0] - start_right_position) * wheel_distance_per_rotation) / 2);

		// Check if the function should exit

		if (timeout != -1)
		{
			if (std::time(nullptr) < (start_time + timeout))
			{
				break;
			}
		}

		if (error < 0.25 && error > -0.25)
		{ // Within -0.25 and 0.25 degrees of the target
			correct_cycles++;
		}
		else
		{
			correct_cycles = 0;
		}

		if (index == 50) {
			lcd::print(1, "cycles %.2f", correct_cycles);
			lcd::print(2, "Error %.2f", error);
			delay(500);

			index = 0;
		}
		index++;

		delay(5);
	}

	left.move(0);
	right.move(0);

	return;
}
#endif
