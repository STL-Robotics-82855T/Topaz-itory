#include "odometry.h"

// Constructor
odometry::odometry(Controller &controller, Imu &imu1, Imu &imu2, Motor &lfb, Motor &rfb, float left_offset, float right_offset, float back_offset, float wheel_size): 
	master(controller), 
	imu_sensor1(imu1), 
	imu_sensor2(imu2), 
	left_front_bottom(lfb), 
	right_front_bottom(rfb) {
		left_offset = left_offset;
		right_offset = right_offset;
		back_offset = back_offset;
		wheel_size = wheel_size;
		inches_per_rotation = wheel_size * PI;
	}

/// @brief Gets the current angle of the robot, updates every 5ms
void odometry::get_current_angle() {
	// odom rotation thread

	while (true) {
		float imu1 = imu_sensor1.get_rotation() * (365.0 / 360.0); // Offset cause imu 1 just sucks ig
		float imu2 = imu_sensor2.get_rotation();

		current_angle_deg = (imu1 + imu2) / 2;
		// cut off to 3 decimal places
		current_angle_deg = floorf(current_angle_deg * 1000) / 1000;
		current_angle_rad = current_angle_deg * (PI / 180.0);

		// master.print(0, 0, "IMU 1: %.2f\n", imu_sensor1.get_rotation()*(365.0/360.0)); 
		// Offset factor due to some weirdness with the IMU
		// Task::delay(50);
		// master.print(1, 0, "IMU 2: %.2f", imu_sensor2.get_rotation());
		// Task::delay(50);
		delay(5);
	}
}

/// @brief Gets the current direction of the robot, updates every 10ms
void odometry::get_current_position() {
	// Positions are measured in number of motor rotations (not degrees)
	float prev_left_pos = 0;
	float prev_right_pos = 0;
	float prev_horizontal_pos = 0;
	float left_pos = 0;
	float right_pos = 0;
	float horizontal_pos = 0;

	// Measured in rad
	float previous_angle = 0;
	float angle_delta;
	float offset_theta = 0;
	float offset_radius = 0;

	// Distances are measured in inches
	float left_side_distance_delta = 0;
	float right_side_distance_delta = 0;
	float horizontal_distance_delta = 0;

	int index = 0;

	while (true) {
		// 36:60 gearing
		left_pos = left_front_bottom.get_position() * (36.0/60.0);
		right_pos = right_front_bottom.get_position() * (36.0/60.0);

		// horizontal_pos = (float)horizontal_tracker.get_position()/36000.0;
		// right_pos = (float)right_tracker.get_position() / 36000.0;

		// ΔL and ΔR
		// left_side_distance_delta = (left_pos - prev_left_pos) * inches_per_rotation;
		right_side_distance_delta = (right_pos - prev_right_pos) * inches_per_rotation;

		// ΔS
		horizontal_distance_delta = (horizontal_pos - prev_horizontal_pos) * inches_per_rotation;
		horizontal_distance_delta = horizontal_distance_delta-back_offset*angle_delta;

		// Δθ
		angle_delta = smart_radian_diff(current_angle_rad, previous_angle);

		// Update position
		if (angle_delta == 0) { // Straight line or no movement
			local_offset.first = horizontal_distance_delta;
			local_offset.second = right_side_distance_delta;
		} else { // Arc
			local_offset.first = (horizontal_distance_delta/angle_delta + back_offset) * (2 * sin(angle_delta / 2));
			local_offset.second = (right_side_distance_delta/angle_delta + right_offset) * (2 * sin(angle_delta / 2));
		}

		// Rotate offset vector to match robot's current heading
		offset_theta = atan2f(local_offset.second, local_offset.first);
		offset_radius = sqrt(pow(local_offset.first, 2) + pow(local_offset.second, 2));

		if (index == 50) {
			cout << local_offset.first << " "  << local_offset.second << " " << offset_theta << endl;

			lcd::print(0, "theta (deg): %.2f", offset_theta*180/PI);
			delay(50);
			lcd::print(1, "X: %.2f", local_offset.first);
			delay(50);
			lcd::print(2, "Y: %.2f", local_offset.second);
			delay(500);

			index = 0;
		}
		index++;

		offset_theta -= current_angle_rad + angle_delta / 2;
		local_offset.first = offset_radius * cos(offset_theta);
		local_offset.second = offset_radius * sin(offset_theta);

		// Update absolute position
		absolute_position.first += local_offset.first;
		absolute_position.second += local_offset.second;

		previous_angle = current_angle_rad;
		// prev_left_pos = left_pos;
		prev_right_pos = right_pos;
		prev_horizontal_pos = horizontal_pos;

		delay(10);
	}
}

/// @brief Calculates the difference between two angles in radians, always returns the smallest difference
/// @param rad1
/// @param rad2
/// @return Difference between the two angles in radians
float odometry::smart_radian_diff(float rad1, float rad2) {
	float diff = rad1 - rad2;
	if (diff >= PI) {
		diff -= 2 * PI;
	}
	if (diff < -PI) {
		diff += 2 * PI;
	}
	return diff;
}