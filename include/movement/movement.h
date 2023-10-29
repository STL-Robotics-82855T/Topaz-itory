#include "main.h"
#ifndef DEVICES_H
#include "devices.h" // Defines motors, controller and sensors
#endif

/// @brief Points the robot to a specified angle using the minimum angle (Blocking)
/// @param degrees 0.0 - 360.0 degrees (0 is forward) (All degrees relative to the starting position of the robot)
/// @param timeout (Optional) Time in sec to kill the function if it doesn't exit
void turn_to_angle_auton(float degrees, float timeout = -1)
{

    float P = 1;
    float D = 0.5;

    // How long the robot has to be within the error range to exit the function in cycles
    float wait_time = 100; // 100 * 5ms = 500ms
    float correct_cycles = 0;

    float previous_angle_error = 0;
    float derivative = 0;
    float power = 0;

    // Calculate error
    float error = degrees - odom.current_angle_deg;

    time_t start_time = std::time(nullptr);

    while (correct_cycles < wait_time)
    {

        // Calculate derivative
        derivative = error - previous_angle_error;
        previous_angle_error = error;

        // Calculate power
        power = error * P + derivative * D;

        // Set motors
        left.move(power);
        right.move(-power);

        // Calculate error
        error = degrees - odom.current_angle_deg;

        // Check if the function should exit

        if (timeout != -1)
        {
            if (std::time(nullptr) < (start_time + timeout))
            {
                break;
            }
        }

        if (error < 0.5 && error > -0.5)
        { // Within -0.5 and 0.5 degrees of the target
            correct_cycles++;
        }
        else
        {
            correct_cycles = 0;
        }

        delay(5);
    }

    left.move(0);
    right.move(0);

    return;
}

/// @brief Drives in a straight line for a specified distance (Blocking)
/// @param inches Distance to travel in inches
/// @param speed Speed to travel at (0 - 100)
/// @param timeout (Optional) Time in sec to kill the function if it doesn't exit
void drive_line_auton(float inches, float speed, float timeout = -1)
{

    float P = 1;
    float D = 0.5;

    float previous_error = 0;
    float derivative = 0;
    float power = 0;

    // Calculate error
    float wheel_distance = 3.25 * PI * (36.0 / 60.0); // inches per rotation

    // How long the robot has to be within the error range to exit the function in cycles
    float wait_time = 100; // 100 * 5ms = 500ms
    float correct_cycles = 0;

    // Error is measured in inches
    float error = inches - ((left.get_positions()[1] * wheel_distance + right.get_positions()[0] * wheel_distance) / 2);

    time_t start_time = std::time(nullptr);

    while (correct_cycles < wait_time)
    {

        // Calculate derivative
        derivative = error - previous_error;
        previous_error = error;

        // Calculate power
        power = error * P + derivative * D;

        // Set motors
        left.move(power);
        right.move(power);

        // Calculate error
        error = inches - ((left.get_positions()[1] * wheel_distance + right.get_positions()[0] * wheel_distance) / 2);

        // Check if the function should exit

        if (timeout != -1)
        {
            if (std::time(nullptr) < (start_time + timeout))
            {
                break;
            }
        }

        if (error < 0.25 && error > -0.25)
        { // Within -0.25 and 0.25 degrees of the target
            correct_cycles++;
        }
        else
        {
            correct_cycles = 0;
        }

        delay(5);
    }

    left.move(0);
    right.move(0);

    return;
}

