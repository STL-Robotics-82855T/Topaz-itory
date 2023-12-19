#pragma once

class odometry {
public:
	float current_angle_deg; // angle in degrees
	float current_angle_rad;
	float current_heading_deg; // absolute heading in degrees (0-360)
	float inches_per_second;
	pair<float, float> absolute_position = { 0.0, 0.0 }; // x is left-right, y is forward-backward, measured in inches

	// Constructor
	odometry(float _wheel_offset, float _wheel_diameter) {
		wheel_offset = _wheel_offset;
		wheel_diameter = _wheel_diameter;
		
	}

	/// @brief Gets the current angle of the robot, updates every 5ms
	void get_current_angle() { // odom rotation thread

		while (true) {
			// float imu1 = imu_sensor1.get_rotation() * (365.0 / 360.0); // Offset cause imu 1 just sucks ig

			current_angle_deg = imu_sensor1.get_rotation();
			current_heading_deg = imu_sensor1.get_heading();
			// // cut off to 3 decimal places
			// current_angle_deg = floorf(current_angle_deg * 1000) / 1000;
			current_angle_rad = current_angle_deg * (PI / 180.0);

			// master.print(0, 0, "IMU 1: %.2f\n", imu_sensor1.get_rotation()*(365.0/360.0)); // Offset factor due to some weirdness with the IMU
			// Task::delay(50);
			// master.print(1, 0, "IMU 2: %.2f", imu_sensor2.get_rotation());
			// Task::delay(50);
			delay(5);

		}
	}

	/// @brief Gets the current position of the robot, updates every 10ms
	void get_current_position() {

		const float inches_per_rotation = wheel_diameter * PI;

		float previous_forward_encoder = right[0].get_position(); // using encoders for now (units in rotations)
		

		float forward_encoder;
		float forward_encoder_delta;
		float forward_distance_delta;

		float previous_angle_deg = odom.current_angle_deg; // check if this should be degrees or heading
		float angle_delta_deg;

		float theta_rad;
		float thetaM_rad;
		float radius;

		while (true) {

			forward_encoder = right[0].get_position();
			forward_encoder_delta = forward_encoder - previous_forward_encoder;
			forward_distance_delta = forward_encoder_delta * inches_per_rotation;

			previous_forward_encoder = forward_encoder;

			angle_delta_deg = odom.current_angle_deg - previous_angle_deg;


			// Since we have traction wheels, we can assume the robot doesn't slide sideways

			if (angle_delta_deg == 0) {
				// Straight line movement
				local_offset.second += forward_distance_delta;

				local_offset.first = 0.0; // No sideways movement since we have traction wheels
			} else {
				// Arc movement
				local_offset.second += (2*sin_degrees(angle_delta_deg/2) * (forward_distance_delta/angle_delta_deg + wheel_offset));
				local_offset.first = 0.0; // No sideways movement since we have traction wheels
			}

			// Update absolute position
			
			theta_rad = atan2f(local_offset.second, local_offset.first);
			radius = local_offset.second; // Simplified from pythagorean theorem, since x is always 0
			thetaM_rad = previous_angle_deg + angle_delta_deg/2;
			theta_rad -= thetaM_rad;
			
			local_offset.first = radius * cos(theta_rad);
			local_offset.second = radius * sin(theta_rad);

			// Update previous angle
			previous_angle_deg = odom.current_angle_deg;

			// Update absolute position
			absolute_position.first += local_offset.first;
			absolute_position.second += local_offset.second;

			delay(10);
		}
	}

private:
	// Offsets are measured in inches

	float wheel_offset; // Distance between the wheel and the COR of the robot
	float wheel_diameter;   // Diameter measured in inches
	pair<float, float> local_offset = { 0.0, 0.0 }; // x is left-right, y is forward-backward, measured in inches

	float sin_degrees(float degrees) {
		return sin(degrees * (PI / 180.0));
	}

};