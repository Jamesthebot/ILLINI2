#include "main.h"
#include "robot.h"
#include "mcl.h"
#include "angled_odom.h"
#include "telemetry.h"



void initialize() {
    pros::lcd::initialize();
	imu.reset(true);
	angledOdom::configure(&back_left_encoder, &back_right_encoder, &imu, &chassis, 2.75, -1.5945, 1.5945);
	angledOdom::start(0, 0, 0); // 48, 0 , 0
	static telemetry mclLog("/usd/telemetry.csv", 300, 4, MCL::LOOP_MS);
	MCL::setTelemetry(&mclLog);
	//MCL::init(300);

	pros::Task screenTask([] {
		while (true) {
			pros::lcd::print(4, "E1: %d E2: %d", back_left_encoder.get_value(), back_right_encoder.get_value());
			pros::lcd::print(5, "IMU: %.2f", imu.get_rotation());
			lemlib::Pose pose = angledOdom::getPose();
			pros::lcd::print(1, "X: %.2f", pose.x);
			pros::lcd::print(2, "Y: %.2f", pose.y);
			pros::lcd::print(3, "Theta: %.2f", pose.theta);
			pros::delay(50);
		}
	}, "screen");

}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	chassis.setPose(0,0,0);
	pros::delay(100);
	chassis.turnToHeading(180, 10000);
} 

void opcontrol() { 
	while (true) {
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		left_mg.move(dir - turn);                     
		right_mg.move(dir + turn);                    
		pros::delay(20);                             
	}
}