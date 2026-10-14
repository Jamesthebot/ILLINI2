#pragma once
#include "main.h"
#include "api.h"


namespace angledOdom {

/**
 * @brief Configure the angled odom system
 *
 * @param enc1 First encoder (mounted at +45 deg from forward)
 * @param enc2 Second encoder (mounted at -45 deg from forward)
 * @param imu  IMU sensor for heading
 * @param chassis Reference to the LemLib chassis (for setPose)
 * @param wheelDiameter Diameter of tracking wheels in inches
 * @param enc1Offset Perpendicular offset of enc1 from tracking center (inches)
 * @param enc2Offset Perpendicular offset of enc2 from tracking center (inches)
 * @param enc1Scale Scale factor for enc1 distance (1.0 = no correction)
 * @param enc2Scale Scale factor for enc2 distance (1.0 = no correction)
 * @param ticksPerRotation Encoder ticks per full rotation (8192 for ADI quadrature)
 */
void configure(pros::adi::Encoder* enc1, pros::adi::Encoder* enc2,
               pros::IMU* imu, lemlib::Chassis* chassis,
               float wheelDiameter, float enc1Offset, float enc2Offset,
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
 * @brief Get the current pose (x, y in inches, theta in degrees)
 */
lemlib::Pose getPose();

} // namespace angledOdom
