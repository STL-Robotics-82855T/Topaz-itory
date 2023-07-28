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
        pair<float, float> absolute_position = {0.0, 0.0};

        // Constructor
        odometry(float left_offset, float right_offset, float back_offset, float wheel_size, float wheel_rpm, float motor_rpm, pair<float, float> tracking_center) {
            left_offset = left_offset;
            right_offset = right_offset;
            back_offset = back_offset;
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
            float prev_horizontal_pos = 0;
            float left_pos = 0;
            float right_pos = 0;
            float horizontal_pos = 0;
            
            // Measured in rad
            float previous_angle;
            float angle_delta;

            // Distances are measured in inches
            float left_side_distance_delta = 0;
            float right_side_distance_delta = 0;
            float horizontal_distance_delta = 0;

            while (true) {
                // 36:60 gearing
                left_pos = left_front.get_position() * (60.0/36.0);
                right_pos = right_front.get_position() * (60.0/36.0);
                horizontal_pos = horizontal_tracker.get_position();

                // ΔL and ΔR
                left_side_distance_delta = (left_pos - prev_left_pos) * inches_per_rotation;
                right_side_distance_delta = (right_pos - prev_right_pos) * inches_per_rotation;

                // ΔS
                horizontal_distance_delta = (horizontal_pos - prev_horizontal_pos) * inches_per_rotation;   

                // Δθ
                angle_delta = smart_radian_diff(current_angle_rad, previous_angle);

                // Update position
                if (angle_delta == 0 || (left_side_distance_delta == right_side_distance_delta)) { // Straight line or no movement
                    local_offset.first = horizontal_distance_delta;
                    local_offset.second = right_side_distance_delta;
                } else { // Arc
                    float x_step_1 = horizontal_distance_delta/angle_delta + back_offset;
                    float y_step_1 = right_side_distance_delta/angle_delta + right_offset;
                    local_offset.first = x_step_1 * (2 * sin(angle_delta / 2));
                    local_offset.second = y_step_1 * (2 * sin(angle_delta / 2));
                }

                float offset_theta = atan2f(local_offset.second, local_offset.first);
                float offset_radius = sqrt(pow(local_offset.first, 2) + pow(local_offset.second, 2));
                offset_theta = offset_theta - current_angle_rad + angle_delta / 2;
                local_offset.first = offset_radius * cos(offset_theta);
                local_offset.second = offset_radius * sin(offset_theta);

                absolute_position.first += local_offset.first;
                absolute_position.second += local_offset.second;

                previous_angle = current_angle_rad;
                prev_left_pos = left_pos;
                prev_right_pos = right_pos;
                prev_horizontal_pos = horizontal_pos;

                Task::delay(5);
            }
        }


    private:
        // Offsets are measured in inches
        float left_offset;
        float right_offset;
        float back_offset;
        float wheel_size; // Diameter measured in inches
        float wheel_rpm;
        float motor_rpm;
        pair<float, float> local_offset = {0.0, 0.0}; // Measured in inches

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