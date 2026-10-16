#include "main.h"
#include "robot.h"
#include "mcl.h"
#include "angled_odom.h"



void initialize() {
    pros::lcd::initialize();
	imu.reset(true);
	angledOdom::configure(&back_left_encoder, &back_right_encoder, &imu, &chassis, 2.75, 0, 0);
	angledOdom::start(48, 0, 0);
	MCL::init(300);

	pros::Task screenTask([] {
		while (true) {
			lemlib::Pose pose = chassis.getPose();
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
	
} 

void opcontrol() { 
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		left_mg.move(dir - turn);                     
		right_mg.move(dir + turn);                    
		pros::delay(20);                             
	}
}