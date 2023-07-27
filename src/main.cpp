#include "main.h"
#include "devices.h" // Defines motors, controller, sensors and helper functions

#include "odometry.h" // Defines odometry class


float P = 1;
float D = 0.5;

// Left offset: 7.25"
// Right offset: 7.25"
// Wheel diameter: 3.25"
// Wheel RPM: 360
// Motor RPM: 600
// Tracking center: (7.25", 7")
odometry odom(7.25, 7.25, 3.25, 360, 600, pair<float, float>(7.25, 7));

// float current_angle = 0;
// float previous_angle = 0;


void initialize() {
	lcd::initialize();
	left.tare_position();
	right.tare_position();


	master.print(0, 0, "Calibrating IMUs...");
	imu_sensor1.reset(true);
	delay(50);
	imu_sensor2.reset(true);
	master.clear();
	
	Task odom_angle_task([] { odom.get_current_angle(); });

}


void disabled() {}

void competition_initialize() {}


void turn_right_to_angle(float degrees) {

	// TODO: Implement Odom (or use IMU)

}

void drive_forward(int desired_rotations) {
	
	int correct_cycles = 0;

	float left_error = 0;
	float right_error = 0;
	float previous_left_error = 0;
	float previous_right_error = 0;
	
	float left_derivative = 0;
	float right_derivative = 0;

	while (correct_cycles < 100) {

		vector<double> left_positions = left.get_positions();
		float left_position = (left_positions[0] + left_positions[1] + left_positions[2]) / 3;
		vector<double> right_positions = right.get_positions();
		float right_position = (right_positions[0] + right_positions[1] + right_positions[2]) / 3;


		lcd::print(0, "Left Position: %f", left_position);
		lcd::print(1, "Right Position: %f", right_position);

		left_error = desired_rotations - left_position;
		right_error = desired_rotations - right_position;

		left_derivative = left_error - previous_left_error;
		right_derivative = right_error - previous_right_error;

		previous_left_error = left_error;
		previous_right_error = right_error;

		lcd::print(2, "Left Error: %f", left_error);
		lcd::print(3, "Right Error: %f", right_error);

		if (left_error < 0.08 && right_error < 0.08) {
			correct_cycles++;
		} else {
			correct_cycles = 0;
		}

		float left_power = left_error * P * 100 + left_derivative * D * 100;
		float right_power = right_error * P * 100 + right_derivative * D * 100;

		left.move(left_power);
		right.move(right_power);

		delay(5);

	}
}

void autonomous() {

	// drive_forward(5);
	// delay(400);
	// turn_right_to_angle(90);
	// delay(400);
	// drive_forward(5);
	// delay(400);
	// turn_right_to_angle(90);
	// delay(400);
	// drive_forward(5);
	// delay(400);
	// turn_right_to_angle(90);
	// delay(400);
	// drive_forward(5);
	// delay(400);
	// turn_right_to_angle(90);
	// delay(400);
	// left.move(0);
	// right.move(0);


}

void opcontrol() {



	// drive_forward(5);

	int index = 0;

	while (true) {

		int power = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

		int left_power = power + turn;
		int right_power = power - turn;

		left.move(left_power);
		right.move(right_power);

		if (index == 50) {
			master.print(0, 0, "Degs: %f", odom.current_angle_deg);
			delay(50);
			master.print(1, 0, "Rads: %f", odom.current_angle_rad);
			index = 0;
		}

		index++;

		delay(5);
	}

}
