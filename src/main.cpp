#include "main.h"
#include "devices.h" // Defines motors, controller, sensors and helper functions
#include "lemlib/api.hpp" // LemLib :)
// #include "./movement/odometry.h" // Defines odometry class


// Needs to be declared before movement class
// odometry odom(6.00, 3.265);
// odometry odom(-0.5, 2.3);


// #include "./movement/movement.h" // Defines movement class
#include "catapult.h" // The catapult functions

// Left offset: 5.8"
// Right offset: 5.8"
// Back offset: 0.5"
// Wheel diameter: 3.25"

// Constants
catapult cata;

lemlib::Drivetrain drivetrain {
	&left,
	&right,
	12,
	2.3,
	360,
	8 // Tune this value (8 is for w/ traction wheels, 2 for without traction wheels)
};

lemlib::TrackingWheel tracking_wheel(&odom_tracker, 2.3, -0.5);

lemlib::OdomSensors sensors {
	&tracking_wheel,
	nullptr,
	nullptr,
	nullptr,
	&imu_sensor1
};

// forward/backward PID
lemlib::ControllerSettings lateralController {
    21, // kP
	0, // kI
    50, // kD
	0, // windupRange
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    16 // slew rate
};
 
// turning PID
lemlib::ControllerSettings angularController {
    5, // kP
	0, // kI
    40, // kD
	10, // windupRange
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

/// @brief Toggles the left wing (facing toward the back of the robot)
void toggle_wing_left() {
	wing_state_1 = !wing_state_1;
	wing_cylinder_1.set_value(wing_state_1);
}

/// @brief Toggles the right wing (facing toward the front of the robot)
void toggle_wing_right() {
	wing_state_2 = !wing_state_2;
	wing_cylinder_2.set_value(wing_state_2);
}

void toggle_wings() {
	wing_global_state = !wing_global_state;
	wing_cylinder_1.set_value(wing_global_state);
	wing_cylinder_2.set_value(wing_global_state);
}

void toggle_blocker() {
	blocker_state = !blocker_state;
	blocker_cylinders.set_value(blocker_state);
}

void toggle_passive_hang() {
	hang_state = !hang_state;
	hang_cylinder.set_value(hang_state);
}

void toggle_side_hang() {
	side_hang_state = !side_hang_state;
	side_hang_cylinder.set_value(side_hang_state);
}

// void toggle_wing_2() {
// 	wing_state_2 = !wing_state_2;
// 	wing_cylinder_2.set_value(wing_state_2);
// }


// void toggle_intake() {
// 	intake_state = !intake_state;
// 	intake_cylinders.set_value(intake_state);
// }

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

// no longer needed with lemlib
// void reset_sensors() {
// 	// set posititon of sensors to 0
// 	// lcd::initialize();
// 	left.tare_position();
// 	right.tare_position();


// 	// horizontal_tracker.reset_position();
// 	// right_tracker.reset_position();
// 	// cata_tracker.reset();
// 	// cata_tracker.reset_position();

// 	odom_tracker.reset_position();
// 	odom_tracker.reset();


// 	master.print(0, 0, "Calibrating IMU...");
// 	imu_sensor1.reset(true);
// 	delay(50);
// 	master.clear();
// 	imu_sensor1.tare();
// }

static lv_res_t btn_click_action(lv_obj_t * btn) {
	uint8_t id = lv_obj_get_free_num(btn);

	printf("Button %d is released\n", id);

	/* The button is released.
	 * Make something here */

	return LV_RES_OK; /*Return OK if the button is not deleted*/
}

void auton_selector() {
	/*Create a title label*/
    lv_obj_t* label = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_text(label, "Default buttons");
    lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

    /*Create a normal button*/
    lv_obj_t* btn1 = lv_btn_create(lv_scr_act(), NULL);
    lv_cont_set_fit(btn1, true, true); /*Enable resizing horizontally and vertically*/
    lv_obj_align(btn1, label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_set_free_num(btn1, 1);   /*Set a unique number for the button*/
    lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_click_action);

    /*Add a label to the button*/
    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Normal");

    /*Copy the button and set toggled state. (The release action is copied too)*/
    lv_obj_t* btn2 = lv_btn_create(lv_scr_act(), btn1);
    lv_obj_align(btn2, btn1, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_btn_set_state(btn2, LV_BTN_STATE_TGL_REL);  /*Set toggled state*/
    lv_obj_set_free_num(btn2, 2);               /*Set a unique number for the button*/

    /*Add a label to the toggled button*/
    label = lv_label_create(btn2, NULL);
    lv_label_set_text(label, "Toggled");

    /*Copy the button and set inactive state.*/
    lv_obj_t* btn3 = lv_btn_create(lv_scr_act(), btn1);
    lv_obj_align(btn3, btn2, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_btn_set_state(btn3, LV_BTN_STATE_INA);   /*Set inactive state*/
    lv_obj_set_free_num(btn3, 3);               /*Set a unique number for the button*/

    /*Add a label to the inactive button*/
    label = lv_label_create(btn3, NULL);
    lv_label_set_text(label, "Inactive");
}

void update_screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}

void initialize() {
    // auton_selector();

	lcd::initialize();
	chassis.calibrate();
	Task screenTask(update_screen);
	// chassis.setPose(35.307, -58.859, 0);
	// chassis.setPose(-35.582, -58.283, 0);


	// Task odom_position_task([] { odom.get_current_position(); });

	cout << "Initialized" << endl;
}




void disabled() {}

void competition_initialize() {}



void autonomous() {

	cout << "Autonomous started" << endl;


	// Shooter side safe AWP (quals)
	// chassis.setPose(-47.124, -57.259, -45);
	// intake_motor.move(-127);
	// chassis.moveToPoint(-51.5, -52.3, 1000, true, 50);
	// delay(1000);
	// intake_motor.move(0);
	// chassis.waitUntilDone();
	// toggle_wing_left();
	// chassis.moveToPose(-33, -59, -90, 1500, {.forwards=false, .maxSpeed=90});
	// chassis.waitUntil(10);
	// toggle_wing_left();
	// // chassis.turnTo(0, -57, 1000, false);
	// chassis.moveToPoint(-8.3, -57, 2000, false, 127);
	// chassis.moveToPoint(-38, -58, 25000, true, 127);
	// chassis.turnTo(0, -57, 1000);
	// intake_motor.move(-127);
	// chassis.moveToPoint(-8.3, -57, 2000);
	// delay(2000);
	// intake_motor.move(0);
	// END Shooter side safe AWP (quals)

	// Shooter side auton steal (elim)
	// chassis.setPose(-36, -54, 0);
	// toggle_wing_right();
	// intake_motor.move(-127);
	// chassis.moveToPoint(-25.5, -7.5, 2000, true, 127);
	// delay(250);
	// toggle_wing_right();
	// delay(750);
	// intake_motor.move(0);
	// chassis.waitUntil(25);
	// intake_motor.move(127);
	// chassis.waitUntilDone();
	// delay(250);
	// chassis.moveToPoint(-48, -52, 2500, false);
	// chassis.waitUntil(90);
	// toggle_wing_left();
	// chassis.turnTo(90, -60, 1000, false, 90);
	// chassis.waitUntilDone();
	// toggle_wing_left();
	// chassis.moveToPoint(-8.5, -57, 2000, false, 127);
	// chassis.moveToPoint(-38, -58, 25000, true, 127);
	// chassis.turnTo(0, -57, 1000); // Adjust this on actual field (pull out ball)
	// chassis.moveToPoint(-8.5, -57, 2000);
	// chassis.waitUntilDone();
	// intake_motor.move(-127);
	// delay(1000);
	// intake_motor.move(0);
	// chassis.moveToPoint(-50, -57, 3000, false);
	// END Shooter side auton steal (elim)


	// Push side AWP (quals)
	// chassis.setPose(14.8, -58.4, -90);
	// intake_motor.move(-127);
	// delay(500);
	// intake_motor.move(127);
	// chassis.moveToPoint(7, -58.4, 1000);
	// chassis.waitUntilDone();
	// delay(250);
	// chassis.moveToPoint(38.3, -58.4, 1300, false);
	// toggle_wing_left();
	// chassis.moveToPose(60, -37.7, -180, 1500, {.forwards=false, .chasePower=15, .lead=0.4});
	// delay(600);
	// toggle_wing_right();
	// chassis.waitUntilDone();
	// chassis.turnTo(58, 0, 700, false);
	// toggle_wing_left();
	// chassis.moveToPoint(58, -32.6, 1500, false);
	// chassis.waitUntilDone();
	// toggle_wing_right();
	// chassis.moveToPoint(58, -40, 1500);
	// chassis.turnTo(58, 0, 700);
	// chassis.waitUntilDone();
	// intake_motor.move(-127);
	// chassis.moveToPoint(58, -32.6, 1500);
	// chassis.waitUntilDone();
	// intake_motor.move(0);
	// chassis.moveToPoint(45.1, -41.7, 1200, false);
	// // chassis.turnTo(12.75, -27.75, 1000);
	// chassis.moveToPoint(12.75, -27.75, 2000);
	// delay(100);
	// intake_motor.move(127);
	// chassis.waitUntilDone();
	// delay(250);
	// intake_motor.move(0);
	// chassis.moveToPose(40, -13.6, 90, 2000, {.chasePower=15, .lead=0.4});
	// chassis.waitUntilDone();
	// intake_motor.move(-127);
	// delay(250);
	// intake_motor.move(0);
	// chassis.moveToPose(10, -36.68, 45, 3000, {.forwards=false, .chasePower=15, .lead=0.4});
	// chassis.waitUntilDone();
	// toggle_blocker();
	// END Push side AWP (quals)

	// Push side 6 ball (elims) (risky)
	// chassis.setPose(34.91, -56.56, 0);
	// intake_motor.move(-127);
	// chassis.moveToPose(34.835, -42.852, 0, 500, {.chasePower=15, .minSpeed=120}); // Get out of way of bar
	// chassis.moveToPose(23.645, -6.963, 0, 1600, {.chasePower=15, .lead=0.2, .minSpeed=120}); // Go toward middle ball
	// delay(500);
	// intake_motor.move(0);
	// chassis.waitUntil(40);
	// intake_motor.move(127);
	// chassis.waitUntilDone();
	// chassis.moveToPose(34.918, -55, 60, 2000, {.forwards=false, .chasePower=7, .lead=0.3, .minSpeed=120}); // Move back to start
	// chassis.waitUntil(85);
	// intake_motor.move(-127);
	// delay(350);
	// intake_motor.move(0);
	// chassis.moveToPose(6.531, -59.514, -90, 1200, {.chasePower=10, .lead=0.4, .minSpeed=120}); // Grab ball under bar
	// delay(250);
	// intake_motor.move(127);
	// chassis.moveToPose(38.3, -58.4, 90, 1300, {.forwards=false, .chasePower=15, .lead=0.4, .minSpeed=120}); // Move back to start-ish
	// delay(400);
	// toggle_wing_left();
	// chassis.waitUntilDone();
	// intake_motor.move(0);
	// chassis.moveToPose(58, -37.7, -180, 1500, {.forwards=false, .chasePower=15, .lead=0.4, .minSpeed=90}); // Descore matchload
	// chassis.waitUntilDone();
	// toggle_wing_left();
	// chassis.moveToPose(58, -32.6, 180, 1500, {.forwards=false, .chasePower=15, .lead=0.4, .minSpeed=120}); // Move to goal
	// chassis.moveToPose(58, -40, 180, 500, {.chasePower=15, .lead=0.2, .minSpeed=120}); // Move away from goal to allow turning
	// chassis.turnTo(58, 0, 750); // Turn to face goal
	// intake_motor.move(-127);
	// chassis.moveToPose(58, -32.6, 0, 1500, {.chasePower=15, .lead=0.4, .minSpeed=120}); // Push into goal
	// chassis.waitUntilDone();
	// intake_motor.move(0); // temp

	// END Push side 6 ball (elims) (risky)

	// Push side 6 ball (elims) (safe)
	chassis.setPose(33.5, -53.841, 0);
	intake_motor.move(-127);
	chassis.moveToPose(33.5, -42.852, 0, 500, {.chasePower=15, .minSpeed=120}); // Get out of way of bar
	chassis.waitUntilDone();
	intake_motor.move(0);
	chassis.moveToPose(9.855, -4.611, -60, 1500, {.chasePower=12, .lead=0.35}); // Go toward middle ball
	chassis.waitUntil(40);
	intake_motor.move(127);
	chassis.turnTo(50, -4.611, 590); // Point to net
	chassis.waitUntilDone();
	intake_motor.move(0);
	chassis.moveToPose(41.286, -4.611, 90, 1200, {.chasePower=15}); // Score 2 in net (Not ramming into the net, adjust on the actual field)
	delay(300);
	intake_motor.move(-127);
	// TEMP
	// chassis.tank(-127, -127);
	delay(250);
	//
	// chassis.moveToPose(44.576, -3.321, 90, 1700, {.chasePower=15, .minSpeed=120}); // Score 2 in net (Not ramming into the net, adjust on the actual field) (For actual ramming)
	chassis.moveToPose(31.654, -4.611, -120, 1200, {.forwards=false, .minSpeed=50}); // Back up from net
	// chassis.waitUntilDone();
	// chassis.turnTo(11, -20, 500);
	// chassis.waitUntilDone();
	delay(300);
	intake_motor.move(127);
	chassis.moveToPose(11, -19.445, -120, 1350); // Grab third triball (safe one)
	// chassis.turnTo(29.849, -35.462, 1200);
	// TEMP
	chassis.waitUntilDone();
	chassis.moveToPose(35, -15.253, -100, 1200, {.forwards=false, .minSpeed=50}); // Back up from third triball
	// chassis.moveToPose(29.849, -35.462, 130, 1700, {.chasePower=9, .lead=0.5, .minSpeed=120}); // Move back to alley ish
	chassis.moveToPose(35, -51.573, 180, 1500, {.chasePower=10, .lead=0.6, .minSpeed=100});
	chassis.waitUntilDone();
	intake_motor.move(0);
	chassis.turnTo(60, -51.573, 450);
	delay(200);
	intake_motor.move(-127);
	chassis.turnTo(0, -51.573, 450);
	chassis.waitUntilDone();
	intake_motor.move(0);
	chassis.moveToPose(8.0, -51.681, -90, 1500, {.chasePower=10, .lead=0.4}); // , .minSpeed=100
	delay(500);
	intake_motor.move(127);
	chassis.moveToPose(38.3, -51.681, 90, 1300, {.forwards=false, .chasePower=15, .lead=0.4, .minSpeed=120}); // Move back to start-ish
	delay(400);
	toggle_wing_left();
	chassis.waitUntilDone();
	intake_motor.move(0);
	chassis.moveToPose(59, -32.7, -180, 2000, {.forwards=false, .chasePower=13, .lead=0.4, .minSpeed=70}); // Descore matchload
	chassis.waitUntilDone();
	toggle_wing_left();
	chassis.moveToPose(59, -30, 180, 1500, {.forwards=false, .chasePower=15, .lead=0.4, .minSpeed=120}); // Move to goal
	chassis.moveToPose(59, -38, 180, 500, {.chasePower=15, .lead=0.2, .minSpeed=120}); // Move away from goal to allow turning
	chassis.turnTo(59, 0, 750); // Turn to face goal
	chassis.waitUntilDone();
	intake_motor.move(-127);
	chassis.moveToPose(59, -30, 0, 1500, {.chasePower=15, .lead=0.4, .minSpeed=120}); // Push into goal

	// PID test code
	// chassis.setPose(0,0,0);
	// chassis.moveToPose(0, 10, 0, 5000);

	cout << "Autonomous ended" << endl;

}



void opcontrol() {

	// If blocker is up, put it down
	if (blocker_state) {
		toggle_blocker();
	}

	// autonomous();

	Task catapult_monitor([] { cata.start(); });
	
	bool reverse_drive = false;
	char is_intake = 0;
	bool prev_intake_state = false;
	bool intake_released = false;

	bool motor_overtorque = false;
	int overtorque_count = 0;

	while (true) {

		int power = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

		// https://www.desmos.com/calculator/zdyrwup2xa (Graph of the power curve)
		// power = power_curve[power]; // Temp disable power curve cause of some turning issues

		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
			reverse_drive = !reverse_drive;
		}

		// int left_power = (reverse_drive ? -1 : 1) * power + turn;
		// int right_power = (reverse_drive ? -1 : 1) * power - turn;

		chassis.arcade(power*(reverse_drive ? -1 : 1), turn, 3.0);

		// chassis.curvature(power*(reverse_drive ? -1 : 1), turn); // Curvature drive if want to test



		// left.move(left_power);
		// right.move(right_power);

		// Parsa Controls

		if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			intake_motor.move(127);
		} else if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			intake_motor.move(-127);
		} else {
			intake_motor.move(0);
		}

		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			toggle_blocker();
		}

		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
			blocker_state = true;
			blocker_cylinders.set_value(blocker_state);
			toggle_passive_hang();
		}

		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			toggle_side_hang();
		}

		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
			toggle_wings();
		}


		// Ryan Controls
		// if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
		// 	intake_motor.move(127);
		// } else if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
		// 	intake_motor.move(-127);
		// } else {
		// 	intake_motor.move(0);
		// }

		// if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
		// 	toggle_intake();

		// }

		// if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
		// 	toggle_wing_1();
		// 	toggle_wing_2();
		// }

		if (is_intake == 1) {
			intake_released = false;
			prev_intake_state = true;
		} else if (is_intake == 0 && prev_intake_state) {
			intake_released = true;	
			prev_intake_state = false;
		} else {
			intake_released = false;
			prev_intake_state = false;
		}

		delay(5);
	}


}

