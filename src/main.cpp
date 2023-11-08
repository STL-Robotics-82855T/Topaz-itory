#include "main.h"
#include "devices.h" // Defines motors, controller, sensors and helper functions
#include "./movement/movement.h" // Defines movement class
#include "./movement/odometry.h" // Defines odometry class
#include "catapult.h" // The catapult functions

#include "squiggles/squiggles.hpp" // The squiggles library for path planning

// Left offset: 5.8"
// Right offset: 5.8"
// Back offset: 0.5"
// Wheel diameter: 3.25"

// Constants
//odometry odom(6.02, 6.02, 0.73, 3.25);
catapult cata(master, cata_motor, cata_tracker);

/*
   --- Calculating Speed ---
   pi * 3.25 in = 10.210176124 in // Distance travelled per full rotation
   600 rpm geared with (36:60) = 360 rpm
   360 rpm = 6 rps
   6 rps * 10.210176124 in = 61.261056744 in/s
   61.261056744 in/s / 12 in/ft = 5.105088062 ft/s
   5.105088062 ft/s * 0.3048 m/ft = 1.556932505 m/s

   --- Calculating Torque ---
   V5 motor 600rpm stall torque: 0.33 N*m
   Gear ratio advantage: 60/36 = 1.666666667
   0.33 N*m * 1.666666667 = 0.55 N*m stall torque

   Using only 0.5 N*m for safety

   --- Calculating Force ---
   Force = torque / wheel radius
   0.5 N*m / 0.0413 m(1.625 in) = 12.1 N

   6 motors * 12.1 N = 72.6 N

   Sum of forces = mass * acceleration

   72.6 N = 4.24 kg * acceleration

   --- Calculating Acceleration ---
   72.6 N / 4.24 kg = 17.12 m/s^2

   17.12 m/s^2 * 3.28084 ft/m = 56.2 ft/s^2

   --- Calculating Jerk ---
   Assuming 0.5 seconds to reach max acceleration
   17.12 m/s^2 / 0.5 s = 34.24 m/s^3

 */

const double MAX_VELOCITY = 1.556932505; // m/s
const double MAX_ACCELERATION = 17.12; // m/s^2
const double MAX_JERK = 34.24; // m/s^3
const double WHEELBASE = 11.9; // in

const double inch_to_meter = 0.0254; // convert all inches to meters for squiggles
const double feet_to_meter = 0.3048; // convert all feet to meters for squiggles

squiggles::Constraints motion_constraints = squiggles::Constraints(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);

squiggles::SplineGenerator spline_generator = squiggles::SplineGenerator(motion_constraints, std::make_shared<squiggles::TankModel>(WHEELBASE*inch_to_meter, motion_constraints));

std::vector<squiggles::ProfilePoint> path = spline_generator.generate({squiggles::Pose(0.0, 0.0, 1.0),squiggles::Pose(4.0, 4.0, 1.0)});

extern const lv_img_dsc_t funniimage;

void reset_sensors() {
	// set posititon of sensors to 0
	lcd::initialize();
	left.tare_position();
	right.tare_position();


	// horizontal_tracker.reset_position();
	// right_tracker.reset_position();
	// cata_tracker.reset_position();


	imu_sensor1.tare();
	imu_sensor2.tare();

	master.print(0, 0, "Calibrating IMUs...");
	imu_sensor1.reset(false);
	delay(50);
	imu_sensor2.reset(true);
	delay(50);
	master.clear();
}

void initialize() {



	reset_sensors();

	Task odom_angle_task([] { odom.get_current_angle(); });
	Task odom_position_task([] { odom.get_current_position(); });
	// Task catapult_rewind([] { cata.rewind_cata(); });
	Task catapult_monitor([] { cata.start(); });
	
	lv_obj_t * img1 = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(img1, &funniimage);
	lv_obj_align(img1, NULL, LV_ALIGN_CENTER, 0, 0);
}


void disabled() {}

void competition_initialize() {}

void autonomous() {

	// time_t start_time = std::time(nullptr); // Get the current time in seconds

	int path_size = path.size();

	for (int index = 0; index < path_size; index++) {
		squiggles::ProfilePoint point = path[index];
		vector<double> wheel_velocities = point.wheel_velocities;
		double left_velocity = wheel_velocities[0];
		double right_velocity = wheel_velocities[1];
		double last_time = point.time;
		double next_time = path[std::min(index + 1, path_size)].time;
		double wait_time = next_time - last_time;

		// Hopefully this works :O
		double left_power = left_velocity / MAX_VELOCITY * 127;
		double right_power = right_velocity / MAX_VELOCITY * 127;

		left.move(left_power);
		right.move(right_power);

		delay(wait_time * 1000);

	}

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

		delay(5);
	}


}

