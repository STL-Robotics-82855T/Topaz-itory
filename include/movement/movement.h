#pragma once

odometry odom(6.02, 6.02, 0.73, 3.25);

float map(float val, float in_min, float in_max, float out_min, float out_max) {
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/// @brief Turns n degrees clockwise (accepts negative) (Blocking)
/// @param target_heading 0.0 - 360.0 degrees (0 is forward) (All degrees relative to the starting position of the robot)
/// @param timeout Time in milliseconds to stop the turn
void turn_to_angle_auton(float target_heading, float timeout = -1, float scaling = 1, bool chainable = false) {

	int start_time = millis();

	// PID constants
	float P = 1.4;
	float I = 0.03;
	float D = 3.4;
	float current_error = abs(target_heading - odom.current_heading_deg);
	float previous_error = 0;
	float build_up_error = 0;
	float allowed_error = 1; // degrees of error allowed
	float power;

	cout << "Turning to angle: " << target_heading << endl;

	while (abs(current_error) > allowed_error || ((abs(left[1].get_actual_velocity()) > 15 || abs(right[0].get_actual_velocity()) > 15) && !chainable)) {
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

		left.move(-power*scaling);
		right.move(power*scaling);

		if (timeout != -1) {
			if (millis() > (start_time + timeout)) {
				break;
			}
		}

		delay(10);

	}

	left.move(0);
	right.move(0);

	cout << "Done turning" << endl;
}

/// @brief Drives in a straight line for a specified distance (Blocking)
/// @param target_inches Distance to travel in inches
/// @param timeout Time in milliseconds to stop the turn
void drive_line_auton(float target_inches, bool chainable = false, float timeout = -1, float scaling = 0) {
	
	int start_time = millis();

	float wheel_distance_per_encoder_rotation = 3.25 * PI * (36.0 / 60.0); // inches per rotation

	// PID constants
	float P = 2.5;
	float I = 0.025;
	float D = 2.25;
	float allowed_error = 0.75; // inches of error allowed


	float start_left_position = left[1].get_position();
	float start_right_position = right[0].get_position();

	float current_error_left = target_inches;
	float previous_error_left = 0;
	float build_up_error_left = 0;
	float current_error_right = target_inches;
	float previous_error_right = 0;
	float build_up_error_right = 0;

	float power_right;
	float power_left;

	cout << "Driving straight for: " << target_inches << endl;

	while (abs(current_error_left) > allowed_error || abs(current_error_right) > allowed_error || ((abs(left[1].get_actual_velocity()) > 15 || abs(right[0].get_actual_velocity()) > 15) && !chainable)) {
				
				
		current_error_right = target_inches - ((right[0].get_position() - start_right_position) * wheel_distance_per_encoder_rotation);
		if (abs(current_error_right) < 3) { // If the error is less than 3 inches, start building up the error (avoids windup)
			build_up_error_right += current_error_right;
		}
		power_right = (current_error_right * P) + (build_up_error_right * I) + ((current_error_right - previous_error_right) * D);
		previous_error_right = current_error_right;

		current_error_left = target_inches - ((left[1].get_position() - start_left_position) * wheel_distance_per_encoder_rotation);
		if (abs(current_error_left) < 3) { // If the error is less than 3 inches, start building up the error (avoids windup)
			build_up_error_left += current_error_left;
		}

		power_left = (current_error_left * P) + (build_up_error_left * I) + ((current_error_left - previous_error_left) * D);
		previous_error_left = current_error_left;

		power_right *= 20+scaling; // Tune this scaling factor
		power_left *= 20+scaling;


		// power_left = map(power_left, 0, 2, 0, 127);
		// power_right = map(power_right, 0, 2, 0, 127);

		// cout << "Error: " << current_error_left << " " << current_error_right << endl;
		// cout << power_left << " " << power_right << endl;



		// power_left = 127 * abs(power_left) / power_left;
		// power_right = 127 * abs(power_right) / power_right;

		right.move(power_right);
		left.move(power_left);

		if (timeout != -1) {
			if (millis() > (start_time + timeout)) {
				break;
			}
		}

		delay(10);

	}

	cout << "Done driving" << endl;

	left.move(0);
	right.move(0);

}

/// @brief Drives for specified degrees on a curved circle
/// @param radius Radius of imaginary circle
/// @param degrees Degrees along the imaginary circle to travel
/// @param timeout Time in milliseconds to stop the turn
void move_circle_auton(float radius, float degrees, bool turn_left, float timeout = -1, float scaling = 12) {
	
	int start_time = millis();

	float starting_angle = odom.current_heading_deg;

	//Circle stuff below

	// calculate the distance each wheel needs to travel
	// left side offset = 5.93 in
	// right side offset = 5.93 in
	// subtract the offset from the radius
	float left_radius = radius + 6.0 * (!turn_left) - 6.0 * turn_left;
	float right_radius = radius + 6.0 * turn_left - 6.0 * (!turn_left);

	float left_distance = left_radius * (degrees / 360.0) * 2 * PI;
	float right_distance = right_radius * (degrees / 360.0) * 2 * PI;

	// PID Stuff below

	float wheel_distance_per_encoder_rotation = 3.25 * PI * (36.0 / 60.0); // inches per rotation

	// PID constants
	float P = 2.5;
	float I = 0.025;
	float D = 2.25;
	float allowed_error = 1.00; // inches of error allowed


	float start_left_position = left[1].get_position();
	float start_right_position = right[0].get_position();

	float current_error_left = left_distance;
	float previous_error_left = 0;
	float build_up_error_left = 0;
	float current_error_right = right_distance;
	float previous_error_right = 0;
	float build_up_error_right = 0;

	float power_right;
	float power_left;


	while (abs(current_error_left) > allowed_error || abs(current_error_right) > allowed_error || abs(left[1].get_actual_velocity()) > 30 || abs(right[0].get_actual_velocity()) > 30) {
		current_error_right = right_distance - ((right[0].get_position() - start_right_position) * wheel_distance_per_encoder_rotation);
		if (abs(current_error_right) < 3) { // If the error is less than 3 inches, start building up the error (avoids windup)
			build_up_error_right += current_error_right;
		}
		power_right = (current_error_right * P) + (build_up_error_right * I) + ((current_error_right - previous_error_right) * D);
		previous_error_right = current_error_right;

		current_error_left = left_distance - ((left[1].get_position() - start_left_position) * wheel_distance_per_encoder_rotation);
		if (abs(current_error_left) < 3) { // If the error is less than 3 inches, start building up the error (avoids windup)
			build_up_error_left += current_error_left;
		}

		power_left = (current_error_left * P) + (build_up_error_left * I) + ((current_error_left - previous_error_left) * D);
		previous_error_left = current_error_left;

		power_right *= scaling; // Tune this scaling factor
		power_left *= scaling;


		// power_left = map(power_left, 0, 2, 0, 127);
		// power_right = map(power_right, 0, 2, 0, 127);

		// cout << "Error: " << current_error_left << " " << current_error_right << endl;
		// cout << power_left << " " << power_right << endl;



		// power_left = 127 * abs(power_left) / power_left;
		// power_right = 127 * abs(power_right) / power_right;

		right.move(power_right);
		left.move(power_left);

		if (timeout != -1) {
			if (millis() > (start_time + timeout)) {
				break;
			}
		}

		delay(10);

	}

	cout << "Done driving" << endl;

	left.move(0);
	right.move(0);

	// if (turn_left) {
	// 	turn_to_angle_auton(starting_angle - degrees);
	// } else {
	// 	turn_to_angle_auton(starting_angle + degrees);
	// }

}



/// @brief Drives for specified degrees on a curved circle (Does not come to a stop, chainable)
/// @param radius Radius of imaginary circle
/// @param degrees Degrees along the imaginary circle to travel
/// @param timeout Time in milliseconds to stop the turn
void move_circle_multi_auton(float radius, float degrees, bool turn_left, float timeout = -1, float scaling = 13) {
	
	int start_time = millis();

	float starting_angle = odom.current_heading_deg;

	//Circle stuff below

	// calculate the distance each wheel needs to travel
	// left side offset = 5.93 in
	// right side offset = 5.93 in
	// subtract the offset from the radius
	float left_radius = radius + 6.0 * (!turn_left) - 6.0 * turn_left;
	float right_radius = radius + 6.0 * turn_left - 6.0 * (!turn_left);

	float left_distance = left_radius * (degrees / 360.0) * 2 * PI;
	float right_distance = right_radius * (degrees / 360.0) * 2 * PI;

	// PID Stuff below

	float wheel_distance_per_encoder_rotation = 3.25 * PI * (36.0 / 60.0); // inches per rotation

	// PID constants
	float P = 2.5;
	float I = 0.025;
	float D = 2.25;
	float allowed_error = 1.00; // inches of error allowed


	float start_left_position = left[1].get_position();
	float start_right_position = right[0].get_position();

	float current_error_left = left_distance;
	float previous_error_left = 0;
	float build_up_error_left = 0;
	float current_error_right = right_distance;
	float previous_error_right = 0;
	float build_up_error_right = 0;

	float power_right;
	float power_left;


	while (abs(current_error_left) > allowed_error || abs(current_error_right) > allowed_error) {
		current_error_right = right_distance - ((right[0].get_position() - start_right_position) * wheel_distance_per_encoder_rotation);
		if (abs(current_error_right) < 3) { // If the error is less than 3 inches, start building up the error (avoids windup)
			build_up_error_right += current_error_right;
		}
		power_right = (current_error_right * P) + (build_up_error_right * I) + ((current_error_right - previous_error_right) * D);
		previous_error_right = current_error_right;

		current_error_left = left_distance - ((left[1].get_position() - start_left_position) * wheel_distance_per_encoder_rotation);
		if (abs(current_error_left) < 3) { // If the error is less than 3 inches, start building up the error (avoids windup)
			build_up_error_left += current_error_left;
		}

		power_left = (current_error_left * P) + (build_up_error_left * I) + ((current_error_left - previous_error_left) * D);
		previous_error_left = current_error_left;

		power_right *= scaling; // Tune this scaling factor
		power_left *= scaling;


		// power_left = map(power_left, 0, 2, 0, 127);
		// power_right = map(power_right, 0, 2, 0, 127);

		// cout << "Error: " << current_error_left << " " << current_error_right << endl;
		// cout << power_left << " " << power_right << endl;



		// power_left = 127 * abs(power_left) / power_left;
		// power_right = 127 * abs(power_right) / power_right;

		right.move(power_right);
		left.move(power_left);

		if (timeout != -1) {
			if (millis() > (start_time + timeout)) {
				break;
			}
		}

		delay(10);

	}

	cout << "Done driving" << endl;

	left.move(0);
	right.move(0);

	// if (turn_left) {
	// 	turn_to_angle_auton(starting_angle - degrees);
	// } else {
	// 	turn_to_angle_auton(starting_angle + degrees);
	// }

}