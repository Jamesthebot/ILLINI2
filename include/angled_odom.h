#pragma once
#include "api.h"
#include "lemlib/api.hpp"


namespace angledOdom {

/**
 * @brief Configure the angled odom system
 *
 * Offsets are measured from the tracking center to each wheel's contact point, in inches,
 * with +x = right and +y = forward. Each encoder must count positive when the bot drives forward.
 *
 * @param enc1 First encoder (rolls toward forward-right, +45 deg from forward)
 * @param enc2 Second encoder (rolls toward forward-left, -45 deg from forward)
 * @param imu  IMU sensor for heading
 * @param chassis Reference to the LemLib chassis (for setPose)
 * @param wheelDiameter Diameter of tracking wheels in inches
 * @param enc1X Sideways offset of enc1 from tracking center (inches, + = right)
 * @param enc1Y Forward offset of enc1 from tracking center (inches, + = forward)
 * @param enc2X Sideways offset of enc2 from tracking center (inches, + = right)
 * @param enc2Y Forward offset of enc2 from tracking center (inches, + = forward)
 * @param enc1Scale Scale factor for enc1 distance (1.0 = no correction)
 * @param enc2Scale Scale factor for enc2 distance (1.0 = no correction)
 * @param ticksPerRotation Encoder ticks per full rotation (8192 for a 2048 CPR encoder such as the AMT102, 360 for the VEX optical shaft encoder)
 */
void configure(pros::adi::Encoder* enc1, pros::adi::Encoder* enc2,
               pros::IMU* imu, lemlib::Chassis* chassis,
               float wheelDiameter, float enc1X, float enc1Y, float enc2X, float enc2Y,
               float enc1Scale = 1.0f, float enc2Scale = 1.0f,
               float ticksPerRotation = 8192.0f);

/**
 * @brief Start the odom update task (10ms period)
 * @param x Starting x position (inches)
 * @param y Starting y position (inches)
 * @param theta Starting heading (degrees)
 */
void start(float x = 0, float y = 0, float theta = 0);

/**
 * @brief Stop the odom update task
 */
void stop();

/**
 * @brief Set the current pose
 */
void setPose(float x, float y, float theta);

/**
 * @brief Shift the current pose by (dx, dy, dtheta) without losing odom updates
 */
void offsetPose(float dx, float dy, float dtheta);

/**
 * @brief Get the current pose (x, y in inches, theta in degrees)
 */
lemlib::Pose getPose();

} // namespace angledOdom
