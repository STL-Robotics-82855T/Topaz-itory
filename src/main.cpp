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
	lemlib::Omniwheel::NEW_325,
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
    10, // kP
	0, // kI
    25, // kD
	0, // windupRange
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    10 // slew rate
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
	chassis.setPose(-35.582, -58.283, 0);


	// Task odom_position_task([] { odom.get_current_position(); });

	cout << "Initialized" << endl;
}




void disabled() {}

void competition_initialize() {}


ASSET(path_txt);

void autonomous() {

	cout << "Autonomous started" << endl;

	intake_motor.move(-127);
	chassis.follow(path_txt, 4, 2500);
	delay(1000);
	intake_motor.move(127);
	delay(1000);
	chassis.moveToPoint(-35.582, -58.283, 2000, false);
	delay(500);
	intake_motor.move(0);
	// cata_motor.move(127);
	// delay(700);
	// cata_motor.move(0);

	// chassis.turnTo(50,0, 1000);
	// chassis.moveToPoint(-23.8, -7.6, 1000, 90);
	// delay(500);
	// intake_motor.move(127);
	// delay(500);
	// intake_motor.move(0);
	// // chassis.moveToPoint(-24.389, -18, 270, 1000, 127, false);
	// cata_motor.move(127);
	// delay(1500);
	// cata_motor.move(0);

	// chassis.turnTo(0,0,1000);
	// chassis.moveTo(0,0,1000);


	cout << "Autonomous ended" << endl;

}



void opcontrol() {

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

		if (master.get_digital(E_CONTROLLER_DIGITAL_LEFT) && master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
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
			is_intake = 1;
		} else if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			motor_overtorque = false;
			is_intake = 2;
		} else {
			is_intake = 0;
		}

		if (intake_motor.get_power() > 12.0) {
			if (overtorque_count == 0) {
				overtorque_count = 1;
			} else {
				overtorque_count++;
			}
			if (overtorque_count > 20) {
				motor_overtorque = true;
			}
		} 

		if (intake_released) {
			motor_overtorque = false;
			overtorque_count = 0;
		}

		if (is_intake == 1 && !motor_overtorque) {
			intake_motor.move(127);
		} else if (is_intake == 2) {
			intake_motor.move(-127);
		} else {
			intake_motor.move(0);
		}


		// if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
		// 	toggle_intake();

		// }

		if (master.get_digital(E_CONTROLLER_DIGITAL_UP) && master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			toggle_blocker();
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

