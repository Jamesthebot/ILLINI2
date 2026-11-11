#include "robot.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-10, -9, -7, -6}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({14, 11, 4, 1}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&left_mg,
                              &right_mg,
                              12.38, // track width
                              2.5, // wheel size
                              600, // drivetrain rpm
                              2 // horizontal drift
);

pros::Imu imu(21);
pros::adi::Encoder back_left_encoder('A', 'B', false);
pros::adi::Encoder back_right_encoder('C', 'D', true);

pros::Distance distanceFront(2);
pros::Distance distanceLeft(12);
pros::Distance distanceRight(16);
pros::Distance distanceBack(13);

Intake intake(-5);
Cascade cascade(3, -8, 15, 0.709, 3);//3,8
Claw claw('H', false, 'G', false, 'F', false); //whole system - actuator, claw clamp, wrist flipper - left to right

lemlib::OdomSensors sensors(nullptr,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu
);

lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(3.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              37, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
);