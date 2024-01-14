#include "main.h"
#include "devices.h" // Defines motors, controller, sensors and helper functions
#include "./movement/odometry.h" // Defines odometry class


// Needs to be declared before movement class
// odometry odom(6.00, 3.265);
odometry odom(-0.5, 2.3);


#include "./movement/movement.h" // Defines movement class
#include "catapult.h" // The catapult functions

// Left offset: 5.8"
// Right offset: 5.8"
// Back offset: 0.5"
// Wheel diameter: 3.25"

// Constants
catapult cata;


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

void reset_sensors() {
	// set posititon of sensors to 0
	// lcd::initialize();
	left.tare_position();
	right.tare_position();


	// horizontal_tracker.reset_position();
	// right_tracker.reset_position();
	// cata_tracker.reset();
	// cata_tracker.reset_position();

	odom_tracker.reset_position();
	odom_tracker.reset();


	master.print(0, 0, "Calibrating IMU...");
	imu_sensor1.reset(true);
	delay(50);
	master.clear();
	imu_sensor1.tare();
}

static lv_res_t btn_click_action(lv_obj_t * btn) {
	uint8_t id = lv_obj_get_free_num(btn);

	printf("Button %d is released\n", id);

	/* The button is released.
	 * Make something here */

	return LV_RES_OK; /*Return OK if the button is not deleted*/
}

void initialize() {
	/*Create a title label*/
	lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(label, "Default buttons");
	lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

	/*Create a normal button*/
	lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
	lv_cont_set_fit(btn1, true, true); /*Enable resizing horizontally and vertically*/
	lv_obj_align(btn1, label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
	lv_obj_set_free_num(btn1, 1);   /*Set a unique number for the button*/
	lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_click_action);

	/*Add a label to the button*/
	label = lv_label_create(btn1, NULL);
	lv_label_set_text(label, "Normal");

	/*Copy the button and set toggled state. (The release action is copied too)*/
	lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), btn1);
	lv_obj_align(btn2, btn1, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
	lv_btn_set_state(btn2, LV_BTN_STATE_TGL_REL);  /*Set toggled state*/
	lv_obj_set_free_num(btn2, 2);               /*Set a unique number for the button*/

	/*Add a label to the toggled button*/
	label = lv_label_create(btn2, NULL);
	lv_label_set_text(label, "Toggled");

	/*Copy the button and set inactive state.*/
	lv_obj_t * btn3 = lv_btn_create(lv_scr_act(), btn1);
	lv_obj_align(btn3, btn2, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
	lv_btn_set_state(btn3, LV_BTN_STATE_INA);   /*Set inactive state*/
	lv_obj_set_free_num(btn3, 3);               /*Set a unique number for the button*/

	/*Add a label to the inactive button*/
	label = lv_label_create(btn3, NULL);
	lv_label_set_text(label, "Inactive");


	reset_sensors();

	Task odom_angle_task([] { odom.get_current_angle(); });
	Task odom_position_task([] { odom.get_current_position(); });
	delay(500);

	cout << "Initialized" << endl;
}


void disabled() {}

void competition_initialize() {}

void autonomous() {

	cout << "Autonomous started" << endl;

	float target_inches = 12.0;

	int start_time = millis();

	float wheel_distance_per_encoder_rotation = 3.26 * PI * (36.0 / 60.0); // inches per rotation

	// PID constants
	float P = 1;
	float I = 0.025;
	float D = 2.25;
	float allowed_error = 0.01; // inches of error allowed


	float start_left_position = left[1].get_position();
	float start_right_position = right[0].get_position();

	float current_error_left = target_inches;
	float previous_error_left = 0;
	float build_up_error_left = 0;
	float current_error_right = target_inches;
	float previous_error_right = 0;
	float build_up_error_right = 0;

	float power_right;

	cout << "Driving straight for: " << target_inches << endl;

	while (abs(current_error_right) > allowed_error || abs(left[1].get_actual_velocity()) > 10 || abs(right[0].get_actual_velocity()) > 10) {



		current_error_right = target_inches - sqrt(odom.absolute_position.second*odom.absolute_position.second + odom.absolute_position.first*odom.absolute_position.first);
		// if (abs(current_error_right) < 3) { // If the error is less than 3 inches, start building up the error (avoids windup)
		// 	build_up_error_right += current_error_right;
		// }
		power_right = (current_error_right * P) + ((current_error_right - previous_error_right) * D);
		previous_error_right = current_error_right;

		power_right *= 8; // Tune this scaling factor


		// power_left = map(power_left, 0, 2, 0, 127);
		// power_right = map(power_right, 0, 2, 0, 127);

		// cout << "Error: " << current_error_left << " " << current_error_right << endl;
		// cout << power_left << " " << power_right << endl;



		// power_left = 127 * abs(power_left) / power_left;
		// power_right = 127 * abs(power_right) / power_right;

		right.move(power_right);
		left.move(power_right);


		delay(10);

	}

	cout << "Done driving" << endl;

	left.move(0);
	right.move(0);


	// close to net side auton

	// intake_motor.move(127); // to keep ball in

	// drive_line_auton(5);
	// drive_line_auton(-25);

	// left.move(-105);
	// right.move(-45);
	// delay(300);
	// toggle_wing_1();
	// delay(300);
	// left.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	// right.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	// left.move(0);
	// right.move(0);
	// left.brake();
	// right.brake();
	// delay(100);
	// left.set_brake_modes(E_MOTOR_BRAKE_COAST);
	// right.set_brake_modes(E_MOTOR_BRAKE_COAST);

	// drive_line_auton(-4);
	// turn_to_angle_auton(-85, 1000, 3);
	// toggle_wing_1();
	// drive_line_auton(-25,false, 1000);
	// drive_line_auton(10, false, 1000);
	// turn_to_angle_auton(-250, 2000, 1);
	// intake_motor.move(-127);
	// drive_line_auton(20, false, 1000);
	// drive_line_auton(-15);
	// turn_to_angle_auton(100, 1000, 3);
	// intake_motor.move(0);
	// drive_line_auton(20, false, 1000);
	// intake_motor.move(-127);
	// drive_line_auton(-10, false, 1000);
	// intake_motor.move(0);



	// turn_to_angle_auton(90, 800);
	// intake_motor.move(-127);
	// drive_line_auton(6);
	// intake_motor.move(0);

	// drive_line_auton(-7);
	// turn_to_angle_auton(180, 1500);

	// drive_line_auton(44);
	// turn_to_angle_auton(-90, 800);
	// drive_line_auton(20);
	// toggle_wing_2();
	// wing_state_1 = !wing_state_1; // to prevent see-saw


	// Far side (shooter) auton

	// intake_motor.move(70); // to keep ball in
	// toggle_wing_1();
	// turn_to_angle_auton(-90, 800, 5);
	// toggle_wing_1();
	// drive_line_auton(10);
	// turn_to_angle_auton(45, 800, 4);
	// drive_line_auton(5);
	// intake_motor.move(-127);
	// drive_line_auton(8);
	// drive_line_auton(-10);
	// turn_to_angle_auton(225, 2500, 5);
	// intake_motor.move(0);
	// drive_line_auton(-10);
	// drive_line_auton(15);

	// backup far side (point toward net and drive straight)
	// intake_motor.move(70); // to keep ball in
	// drive_line_auton(25);
	// intake_motor.move(-127);
	// delay(500);
	// intake_motor.move(0);
	// drive_line_auton(-10);
	// drive_line_auton(30);



	// Skills auton
	// turn_to_angle_auton(-35, 500, 2);
	// toggle_intake();
	// left.move(6);
	// right.move(5);
	// int cata_launches = 0;
	// bool has_counted = false;
	// while (true) {
	// 	cata_motor.move(110);
	// 	if (cata_tracker.get_position() < 2000) {
	// 		if (!has_counted) {
	// 			cata_launches++; 
	// 			has_counted = true;
	// 		}
	// 	} else if (cata_tracker.get_position() > 3000) {
	// 		has_counted = false;
	// 	}
	// 	if (cata_launches == 48) {
	// 		break;
	// 	}
	// 	master.print(0, 0, "Cata launches: %d", cata_launches-1);
	// 	delay(50);

	// 	if (odom.current_angle_deg > -18) {
	// 		right.move(12);
	// 		left.move(5);
	// 	} else if (odom.current_angle_deg < -22) {
	// 		right.move(5);
	// 		left.move(12);
	// 	}
	// }
	// cata_motor.move(0);
	// left.move(0);
	// right.move(0);
	// turn_to_angle_auton(0, 15, 2);
	// drive_line_auton(-5);
	// turn_to_angle_auton(195, 3000, 3);

	// cata_motor.move(127);
	// delay(250);
	// cata_motor.move(0);
	// toggle_intake();
	// intake_motor.move(127);
	// drive_line_auton(80);


	// turn_to_angle_auton(130, 2000, 3);
	// drive_line_auton(30);
	// drive_line_auton(-15);
	// turn_to_angle_auton(60);
	// drive_line_auton(50);
	// turn_to_angle_auton(180);
	// toggle_wing_1();
	// toggle_wing_2();
	// intake_motor.move(-127);
	// drive_line_auton(30);
	// drive_line_auton(-15);
	// drive_line_auton(20);
	// intake_motor.move(0);





	cout << "Autonomous ended" << endl;

}



void opcontrol() {

	// autonomous();

	Task catapult_monitor([] { cata.start(); });

	int index = 0;
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


		int left_power = (reverse_drive ? -1 : 1) * power + turn;
		int right_power = (reverse_drive ? -1 : 1) * power - turn;

		left.move(left_power);
		right.move(right_power);



		if (index == 10) {
			// lcd::print(1, "X: %.2f", odom.absolute_position.first);
			delay(25);
			// lcd::print(2, "Y: %.2f", odom.absolute_position.second);
			delay(25);

			index = 0;
		}
		index++;

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

