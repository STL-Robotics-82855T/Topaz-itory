#include "main.h"
#include "devices.h" // Defines motors, controller, sensors and helper functions
#include "./movement/odometry.h" // Defines odometry class
#include "./movement/movement.h" // Defines movement class
#include "catapult.h" // The catapult functions

#include "squiggles/squiggles.hpp" // The squiggles library for path planning

// Left offset: 5.8"
// Right offset: 5.8"
// Back offset: 0.5"
// Wheel diameter: 3.25"

// Constants
//odometry odom(6.02, 6.02, 0.73, 3.25);
catapult cata;

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

// squiggles::Constraints motion_constraints = squiggles::Constraints(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);

// squiggles::SplineGenerator spline_generator = squiggles::SplineGenerator(motion_constraints, std::make_shared<squiggles::TankModel>(WHEELBASE * inch_to_meter, motion_constraints));

// std::vector<squiggles::ProfilePoint> path = spline_generator.generate({ squiggles::Pose(0.0, 0.0, 0.0),squiggles::Pose(1.0, 1.0, 1.0) });

// Pre-calculated values for the controller to save time (https://www.desmos.com/calculator/zdyrwup2xa)
std::map<int, int> power_curve = {{-127, -127},{-126, -125},{-125, -124},{-124, -122},{-123, -121},{-122, -119},{-121, -118},{-120, -116},{-119, -115},{-118, -113},{-117, -112},{-116, -110},{-115, -109},{-114, -108},{-113, -106},{-112, -105},{-111, -103},{-110, -102},{-109, -100},{-108, -99},{-107, -98},{-106, -96},{-105, -95},{-104, -94},{-103, -92},{-102, -91},{-101, -90},{-100, -88},{-99, -87},{-98, -86},{-97, -84},{-96, -83},{-95, -82},{-94, -80},{-93, -79},{-92, -78},{-91, -77},{-90, -75},{-89, -74},{-88, -73},{-87, -72},{-86, -70},{-85, -69},{-84, -68},{-83, -67},{-82, -65},{-81, -64},{-80, -63},{-79, -62},{-78, -61},{-77, -59},{-76, -58},{-75, -57},{-74, -56},{-73, -55},{-72, -54},{-71, -53},{-70, -51},{-69, -50},{-68, -49},{-67, -48},{-66, -47},{-65, -46},{-64, -45},{-63, -44},{-62, -43},{-61, -42},{-60, -41},{-59, -40},{-58, -39},{-57, -38},{-56, -37},{-55, -36},{-54, -35},{-53, -34},{-52, -33},{-51, -32},{-50, -31},{-49, -30},{-48, -29},{-47, -28},{-46, -27},{-45, -26},{-44, -25},{-43, -25},{-42, -24},{-41, -23},{-40, -22},{-39, -21},{-38, -20},{-37, -19},{-36, -19},{-35, -18},{-34, -17},{-33, -16},{-32, -16},{-31, -15},{-30, -14},{-29, -13},{-28, -13},{-27, -12},{-26, -11},{-25, -11},{-24, -10},{-23, -9},{-22, -9},{-21, -8},{-20, -7},{-19, -7},{-18, -6},{-17, -6},{-16, -5},{-15, -5},{-14, -4},{-13, -4},{-12, -3},{-11, -3},{-10, -2},{-9, -2},{-8, -2},{-7, -1},{-6, -1},{-5, 0},{-4, 0},{-3, 0},{-2, 0},{-1, 0},{0, 0},{1, 0},{2, 0},{3, 0},{4, 0},{5, 0},{6, 1},{7, 1},{8, 2},{9, 2},{10, 2},{11, 3},{12, 3},{13, 4},{14, 4},{15, 5},{16, 5},{17, 6},{18, 6},{19, 7},{20, 7},{21, 8},{22, 9},{23, 9},{24, 10},{25, 11},{26, 11},{27, 12},{28, 13},{29, 13},{30, 14},{31, 15},{32, 16},{33, 16},{34, 17},{35, 18},{36, 19},{37, 19},{38, 20},{39, 21},{40, 22},{41, 23},{42, 24},{43, 25},{44, 25},{45, 26},{46, 27},{47, 28},{48, 29},{49, 30},{50, 31},{51, 32},{52, 33},{53, 34},{54, 35},{55, 36},{56, 37},{57, 38},{58, 39},{59, 40},{60, 41},{61, 42},{62, 43},{63, 44},{64, 45},{65, 46},{66, 47},{67, 48},{68, 49},{69, 50},{70, 51},{71, 53},{72, 54},{73, 55},{74, 56},{75, 57},{76, 58},{77, 59},{78, 61},{79, 62},{80, 63},{81, 64},{82, 65},{83, 67},{84, 68},{85, 69},{86, 70},{87, 72},{88, 73},{89, 74},{90, 75},{91, 77},{92, 78},{93, 79},{94, 80},{95, 82},{96, 83},{97, 84},{98, 86},{99, 87},{100, 88},{101, 90},{102, 91},{103, 92},{104, 94},{105, 95},{106, 96},{107, 98},{108, 99},{109, 100},{110, 102},{111, 103},{112, 105},{113, 106},{114, 108},{115, 109},{116, 110},{117, 112},{118, 113},{119, 115},{120, 116},{121, 118},{122, 119},{123, 121},{124, 122},{125, 124},{126, 125},{127, 127}};

void reset_sensors() {
	// set posititon of sensors to 0
	lcd::initialize();
	left.tare_position();
	right.tare_position();


	// horizontal_tracker.reset_position();
	// right_tracker.reset_position();
	cata_tracker.reset();
	cata_tracker.reset_position();


	master.print(0, 0, "Calibrating IMU...");
	imu_sensor1.reset(true);
	delay(50);
	master.clear();
	imu_sensor1.tare();
}

void initialize() {

	reset_sensors();

	Task odom_angle_task([] { odom.get_current_angle(); });
	// Task odom_position_task([] { odom.get_current_position(); });
	Task catapult_monitor([] { cata.start(); });

	cout << "Initialized" << endl;
}


void disabled() {}

void competition_initialize() {}

void autonomous() {

	cout << "Autonomous started" << endl;


	// Left side auton

	// drive_line_auton(5);
	// delay(500);
	// turn_to_angle_auton(-60);
	// delay(1000);
	// drive_line_auton(20);
	// delay(500);
	// intake_motor.move(-100);
	// delay(1000);
	// intake_motor.move(0);
	// drive_line_auton(10);
	// delay(50);
	// turn_to_angle_auton(0);
	// delay(50);
	// drive_line_auton(50);
	// delay(50);
	// drive_line_auton(-60);


	// Right side auton

	// drive_line_auton(5);
	// delay(500);
	// turn_to_angle_auton(60);
	// delay(1000);
	// drive_line_auton(20);
	// delay(500);
	// intake_motor.move(-100);
	// delay(1000);
	// intake_motor.move(0);
	// drive_line_auton(10);
	// delay(50);
	// turn_to_angle_auton(0);
	// delay(50);
	// drive_line_auton(50);
	// delay(50);
	// drive_line_auton(-60);


	// Auton skills
	intake_state = !intake_state;
	intake_cylinders.set_value(intake_state);
	
	intake_motor.move(127);
	delay(50000);
	intake_motor.move(0);
	drive_line_auton(75);


	cout << "Autonomous ended" << endl;

}



void opcontrol() {

	int index = 0;
	bool reverse_drive = false;

	while (true) {

		int power = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

		// https://www.desmos.com/calculator/zdyrwup2xa (Graph of the power curve)
		power = power_curve[power];
		
		if (master.get_digital(E_CONTROLLER_DIGITAL_LEFT) && master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
			reverse_drive = !reverse_drive;
		}

		int left_power = (reverse_drive ? -1 : 1) * power + turn;
		int right_power = (reverse_drive ? -1 : 1) * power - turn;

		left.move(left_power);
		right.move(right_power);

		if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intake_motor.move(127);
		} else if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			intake_motor.move(-127);
		} else {
			intake_motor.move(0);
		}

		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			intake_state = !intake_state;
			intake_cylinders.set_value(intake_state);

		}

		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
			wing_state = !wing_state;
			wing_cylinders.set_value(wing_state);
		}

		delay(5);
	}


}

