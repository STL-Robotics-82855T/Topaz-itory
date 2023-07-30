#include "main.h"
#include "devices.h" // Defines motors, controller, sensors and helper functions
#include "movement.h" // Defines movement class
#include "odometry.h" // Defines odometry class

// Left offset: 5.8"
// Right offset: 5.8"
// Back offset: 0.5"
// Wheel diameter: 3.25"
odometry odom(6.02, 6.02, 0.73, 3.25);


void initialize() {
	lcd::initialize();
	left.tare_position();
	right.tare_position();
	horizontal_tracker.reset_position();
	right_tracker.reset_position();

	master.print(0, 0, "Calibrating IMUs...");
	imu_sensor1.reset(true);
	delay(50);
	imu_sensor2.reset(true);
	delay(50);
	master.clear();
	
	Task odom_angle_task([] { odom.get_current_angle(); });
	Task odom_position_task([] { odom.get_current_position(); });

	// turn_to_angle(90, true);
}


void disabled() {}

void competition_initialize() {}

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

	int index = 0;

	while (true) {

		int power = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

		int left_power = power + turn;
		int right_power = power - turn;

		left.move(left_power);
		right.move(right_power);

		if (index == 50) {
			master.print(0, 0, "X: %f", odom.absolute_position.first);
			delay(50);
			master.print(1, 0, "Y: %f", odom.absolute_position.second);
			delay(50);
			// float test = (left_front.get_position() * (36.0/60.0))*odom.inches_per_rotation;
			// float test = (float)right_tracker.get_position()/36000.0;
			master.print(2, 0, "Dist: %f", odom.current_angle_deg);
			index = 0;
		}

		index++;

		delay(5);
	}
	 

}
