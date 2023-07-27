#include "main.h"
#ifndef DEVICES_H
#include "devices.h" // Defines motors, controller and sensors
#endif

class odometry {
    public:
        float current_angle_deg;
        float current_angle_rad;
        float inches_per_second;
        float inches_per_rotation;
        pair<float, float> absolute_position;

        // Constructor
        odometry(float left_offset, float right_offset, float wheel_size, float wheel_rpm, float motor_rpm, pair<float, float> tracking_center) {
            left_offset = left_offset;
            right_offset = right_offset;
            wheel_size = wheel_size;
            wheel_rpm = wheel_rpm;
            motor_rpm = motor_rpm;
            inches_per_second = wheel_size * PI * wheel_rpm / 60;
            inches_per_rotation = wheel_size * PI;
        }

        // Threads
        void get_current_angle() { // odom rotation thread

            while (true) {
                float imu1 = imu_sensor1.get_rotation()*(365.0/360.0); // Offset cause imu 1 just sucks ig
                float imu2 = imu_sensor2.get_rotation();

                current_angle_deg = (imu1 + imu2) / 2;
                current_angle_rad = current_angle_deg * (PI / 180.0);

                // master.print(0, 0, "IMU 1: %.2f\n", imu_sensor1.get_rotation()*(365.0/360.0)); // Offset factor due to some weirdness with the IMU
                // Task::delay(50);
                // master.print(1, 0, "IMU 2: %.2f", imu_sensor2.get_rotation());
                // Task::delay(50);
                Task::delay(5);
            }
        }
        void get_current_position() {
            // Positions are measured in number of motor rotations (not degrees)
            float prev_left_pos = 0;
            float prev_right_pos = 0;
            float left_pos = 0;
            float right_pos = 0;
            
            // Measured in rad
            float previous_angle;
            float angle_delta;

            // Distances are measured in inches
            float left_side_distance_delta = 0;
            float right_side_distance_delta = 0;

            while (true) {
                vector<double> left_positions = left.get_positions();
                vector<double> right_positions = right.get_positions();
                left_pos = accumulate(left_positions.begin(), left_positions.end(), 0.0) / left_positions.size();
                right_pos = accumulate(right_positions.begin(), right_positions.end(), 0.0) / right_positions.size();

                // ΔL and ΔR
                left_side_distance_delta = (left_pos - prev_left_pos) * inches_per_rotation;
                right_side_distance_delta = (right_pos - prev_right_pos) * inches_per_rotation;

                // Δθ
                angle_delta = smart_radian_diff(current_angle_rad, previous_angle);

                // Change in position
                float distance_delta = (left_side_distance_delta + right_side_distance_delta) / 2;

                // Update position
                if (angle_delta == 0 || (left_side_distance_delta == right_side_distance_delta)) { // Straight line or no movement
                    local_position.first += distance_delta * cos(current_angle_rad);
                    local_position.second += distance_delta * sin(current_angle_rad);
                } else { // Arc
                    float radius = distance_delta / angle_delta;
                    float x = radius * sin(angle_delta);
                    float y = radius * (1 - cos(angle_delta));
                    local_position.first += x;
                    local_position.second += y;
                }

                previous_angle = current_angle_rad;
                prev_left_pos = left_pos;
                prev_right_pos = right_pos;



                Task::delay(5);
            }
        }


    private:
        // Offsets are measured in inches
        float left_offset;
        float right_offset;
        float wheel_size; // Diameter measured in inches
        float wheel_rpm;
        float motor_rpm;
        pair<float, float> local_position; // Measured in inches

        float smart_radian_diff(float rad1, float rad2) {
            float diff = rad1 - rad2;
            if (diff > PI) {
                diff -= 2 * PI;
            } else if (diff < -PI) {
                diff += 2 * PI;
            }
            return diff;
        }

};
