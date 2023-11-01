#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "main.h"

class odometry {
	public:
		float current_angle_deg;
		float current_angle_rad;
		float current_heading_deg;
		float inches_per_second;
		float inches_per_rotation;
		Controller master;
		Imu &imu_sensor1;
		Imu &imu_sensor2;
		Motor &left_front_bottom;
		Motor &right_front_bottom;
		pair<float, float> absolute_position = {0.0, 0.0};
		odometry(Controller &controller, Imu &imu1, Imu &imu2, Motor &lfb, Motor &rfb, float left_offset, float right_offset, float back_offset, float wheel_size);
		void get_current_angle();
		void get_current_position();

	private:
		// Offsets are measured in inches
		float left_offset;  // Inches measured from the center of the robot
		float right_offset; // Inches measured from the center of the robot
		float back_offset;  // Inches measured from the center of the robot
		float wheel_size;   // Diameter measured in inches
		float wheel_rpm;
		float motor_rpm;
		pair<float, float> local_offset = {0.0, 0.0}; // Measured in inches
		float smart_radian_diff(float rad1, float rad2);
};
#endif
