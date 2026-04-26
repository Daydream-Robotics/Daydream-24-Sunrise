#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "autonomous.hpp"
#include "slam.h"
#include "objectHandler.h"
#include <numbers>
#include "arclengthSplining.hpp"
#include "paths.hpp"
#include "sd_card_logging.hpp"
#include "purePursuit.hpp"
#include <chrono>

// ALS_Path als_path1;
// ALS_Path als_path2;
// ALS_Path als_path3;
std::vector<ALS_Path> als_paths;
PurePursuit purePursuit = PurePursuit();
Autonomous auton = Autonomous();

void initialize() {
	// Initialize subsystems
	pros::lcd::initialize();
	pros::lcd::print(0, "Reg: Initialize");
	imu.reset();
	while (imu.is_calibrating()) {
		pros::delay(20);
	}
	pros::Task frame_task(UpdateFrame_task_fn, (void*)"PROS_Task_Param", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Vision Frame Update");

	// Precompute Sample Tables (need to do this for each path)
	// ALS_Path als_path1;
	
	Logger::getInstance().init("/usd/log.txt");
    LOG("Program Start");


	
	// als_path1.buildFromPoints(path, 0.25); //(path id, sample spacing)
	// als_path1.isValid() ? pros::lcd::print(2, "Path build success") : pros::lcd::print(2, "Path build failure");
	
	// als_path2.buildFromPoints(path2, 0.25); //(path id, sample spacing)
	// als_path2.isValid() ? pros::lcd::print(3, "Path build success") : pros::lcd::print(3, "Path build failure");

	// als_path3.buildFromPoints(path3, 0.25); //(path id, sample spacing)
	// als_path3.isValid() ? pros::lcd::print(3, "Path build success") : pros::lcd::print(3, "Path build failure");

	als_paths = buildAllPaths(0.25);
}

void disabled() {}

void competition_initialize() {}
//all after path actions are commented out for path testing purposes
void autonomous() {
	/* 
	Path Name: Matchload
	Path Action: Goes to the matchloader
	After Path Action: Use AI to collect balls from the mathloader
	Path Notes: Has the Overshoot points
	*/
	auto stepStartTime = std::chrono::high_resolution_clock::now(); 
	unloader.set_value(true);
	purePursuit.setPath(als_paths[MATCHLOAD]);
	while (not purePursuit.step(1.0, 1, 16)) {
		auto currentTime = std::chrono::high_resolution_clock::now();
    	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - stepStartTime).count();
		if (elapsed > 2500) {
			break;
		}
		pros::delay(10);
	}
	

	///////////////
	matchload(false);
	///////////////







	/* 
	Path Name: Score
	Path Action: Goes to the goal
	After Path Action: Outakes all balls
	Path Notes: N/A
	*/
	stepStartTime = std::chrono::high_resolution_clock::now(); 
	purePursuit.setPath(als_paths[SCORE]);
    while (not purePursuit.step(-1, 0.6, 16)) {
		auto currentTime = std::chrono::high_resolution_clock::now();
    	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - stepStartTime).count();
		if (elapsed > 1) {
			break;
		}
        pros::delay(10);
    }




	// Jitter; reverse for 100 ms, score for 625 ms
	///////////////
	move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE, 1);
	move_intake(STOP, STOP, STOP);
	///////////////

	unloader.set_value(false);
	descorer.set_value(true);


	stepStartTime = std::chrono::high_resolution_clock::now(); 
	purePursuit.setPath(als_paths[OUT]);
	while (not purePursuit.step(1, 0.8, 16)) {
		auto currentTime = std::chrono::high_resolution_clock::now();
    	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - stepStartTime).count();
		if (elapsed > 1800) {
			break;
		}
		pros::delay(10);
	}

	purePursuit.setPath(als_paths[WING]);
	while (not purePursuit.step(-1, 0.8, 12)) {
		pros::delay(10);
	}

	
}

void opcontrol() {
	// Set chassis brake mode to coast

	
	// Autonomous auton = Autonomous();

	// // Go to matchloader
	// // auton.travel(-36, 70, 0);

	// // Turn to matchloader
	// // unloader.set_value(true);
	// // auton.turnTo(90);

	//  matchload(false);

	// collect(GamePiece::RED_BALL, 4);

	 


	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	frontIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	backIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	while(true){

		drive(DriveType::SPLIT_ARCADE);

		if (controller.get_digital(DIGITAL_L1)) {
			descorer.set_value(true);
		} else {
			descorer.set_value(false);
		}

		if (controller.get_digital(DIGITAL_Y)) {
			unloader.set_value(true);
		} else {
			unloader.set_value(false);
		}

		if (controller.get_digital(DIGITAL_R1)) { // intake
			move_intake(STOP, HIGH_VOLTAGE, HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_R2)){ // full score
			move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_L2)){ // score
			centerScore.set_value(true);
			move_intake(HIGH_VOLTAGE, HIGH_VOLTAGE, HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_RIGHT)) { // full outtake
			move_intake(-HIGH_VOLTAGE, -HIGH_VOLTAGE, -HIGH_VOLTAGE);
		} else {
			centerScore.set_value(false);
			move_intake(STOP, STOP, STOP);
		}

		// /* - - - - - - - - - - - - - - [CHASSIS CONTROLS] - - - - - - - - - - - - - - */

		// drive(DriveType::TANK);

		// /* - - - - - - - - - - - - - - [MATCH UNLOADER] - - - - - - - - - - - - - - */

		// if (controller.get_digital_new_press(DIGITAL_L1)) {
		// 	unloader.toggle();
		// }

		// /* - - - - - - - - - - - - - - [CENTER TOGGLE] - - - - - - - - - - - - - - */

		// if (controller.get_digital_new_press(DIGITAL_A)) {
		// 	centerScore.toggle();
		// }


		// /* - - - - - - - - - - - - - - [DESCORE TOGGLE] - - - - - - - - - - - - - - */

		// if (controller.get_digital_new_press(DIGITAL_L2)) {
		// 	descorer.toggle();
		// }


		// /* - - - - - - - - - - - - - - - - [AI] - - - - - - - - - - - - - - - - - - */
		
		// //Matchloads redballs on the bottom and blue balls on the top
		// if(controller.get_digital_new_press(DIGITAL_LEFT)){
		// 	matchload(false);
		// }

		// //collects the red balls off the wall
		// if(controller.get_digital_new_press(DIGITAL_RIGHT)){
		// 	collect(GamePiece::RED_BALL, 4);
		// }

		// //collects a single red ball
		// if(controller.get_digital_new_press(DIGITAL_DOWN)){
		// 	collect(GamePiece::RED_BALL);
		// }

		// //tracks a red ball by turning the robot but not by approaching it
		// if(controller.get_digital_new_press(DIGITAL_UP)){
		// 	trackingMode(GamePiece::RED_BALL);
		// }

		// //collects red balls until the button is pressed again (hold down the button)
		// if(controller.get_digital_new_press(DIGITAL_Y)){
		// 	while(true){
		// 		collect(GamePiece::RED_BALL);

		// 		if(controller.get_digital_new_press(DIGITAL_Y)){
		// 			break;
		// 		}
		// 	}
		// }


		
		// /* - - - - - - - - - - - - - - [INTAKE] - - - - - - - - - - - - - - */
		// // Main intake
		



		// // Delay added to prevent crashing
		// pros::delay(20);
	}

	
}

void move_intake(int front, int mid, int back, double seconds) {

	// check for stalling later and stop motors if stalling

	frontIntake.move(front);
	midIntake.move(mid);
	backIntake.move(back);

	if (seconds != 0) {
		pros::delay(seconds * 1000);

		frontIntake.move(STOP);
		midIntake.move(STOP);
		backIntake.move(STOP);
	}
}

void drive(DriveType type) {
	if (!controller.get_digital(DIGITAL_UP) && !controller.get_digital(DIGITAL_DOWN)) {
		switch (type) {
			case DriveType::TANK: {
				// Get joystick values
				int leftY = controller.get_analog(ANALOG_LEFT_Y);
				int rightY = controller.get_analog(ANALOG_RIGHT_Y);

				// Dead zone for both motors
				if(abs(leftY) > DEADZONE) {
					leftMotors.move(leftY);
				} else {
					leftMotors.move(STOP);
				}

				if(abs(rightY) > DEADZONE) {
					rightMotors.move(rightY);
				} else { 
					rightMotors.move(STOP);
				}
				break;
			}
			case DriveType::SPLIT_ARCADE: {
				// Get joystick values
				int power = controller.get_analog(ANALOG_LEFT_Y);
				int turn = controller.get_analog(ANALOG_RIGHT_X);

				int left = power + turn;
				int right = power - turn;

				// Dead zone for both motors
				if(abs(left) > DEADZONE) {
					leftMotors.move(left);
				} else {
					leftMotors.move(0);
				}

				if(abs(right) > DEADZONE) {
					rightMotors.move(right);
				} else { 
					rightMotors.move(0);
				}
				break;
			}
		}
	} else {
		if (controller.get_digital(DIGITAL_UP)) { // slow move forward
			leftMotors.move(LOW_VOLTAGE);
			rightMotors.move(LOW_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_DOWN)){ // slow move backward
			leftMotors.move(-LOW_VOLTAGE);
			rightMotors.move(-LOW_VOLTAGE);
		}
	}
}