#include "main.h"
#include "subsystems.h"
#include "constants.h"
#include "odometry.h"

#include <numbers>


void initialize() {
	pros::lcd::initialize();

	// Initialize tracking wheels to zero
	LTWheel.reset_position();
	RTWheel.reset_position();
	BTWheel.reset_position();

	imu.reset();

	while (imu.is_calibrating()) {
		pros::delay(20);
	}
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	// Move away from parking zone
	travelDistanceWithHeading(-15.0, 50, 0, -1);

	// Turn to farside
	turn_pid(90, 0);

	// Outtake any balls encountered
	frontIntake.move(HIGH_VOLTAGE);
	mainIntake.move(HIGH_VOLTAGE);
	backIntake.move(HIGH_VOLTAGE);

	// Travel across field
	travelDistanceWithHeading(90, 50, 90, -1);

	// Go to matchloader
	turn_pid(180, 0);
	travelDistanceWithHeading(18.25, 50, 180, -1);
	backIntake.move(STOP);

	// Get balls from match loader
	// Turn to matchloader
	turn_pid(90, 0);
	piston.set_value(true);
	pros::delay(1000);

	frontIntake.move(HIGH_VOLTAGE);
	mainIntake.move(HIGH_VOLTAGE);

	// Attack matchloader
	travelDistanceWithHeading(8.7, 33, 90, 1500);
	pros::delay(3000);

	// Back away from matchloader
	travelDistanceWithHeading(-7.5, 50, 90, -1);
	piston.set_value(false);

	// Turn to balls
	turn_pid(180, 0);
	travelDistanceWithHeading(8.0, 50, 180, -1);
	// Intake side balls
	travelDistanceWithHeading(5.0, 18, 180, 1200);

	// Reverse back to align with long goal
	frontIntake.move(STOP);
	mainIntake.move(STOP);
	travelDistanceWithHeading(-12, 50, 180, -1);

	// Turn to long goal
	turn_pid(90, 0);
	travelDistanceWithHeading(-20.15, 35, 90, 2700);
	mainIntake.move(-MID_VOLTAGE);
	backIntake.move(-MID_VOLTAGE);
	pros::delay(300);

	backIntake.move(MID_VOLTAGE);
	mainIntake.move(MID_VOLTAGE);
	frontIntake.move(MID_VOLTAGE);
	pros::delay(3000);

	backIntake.move(STOP);
	mainIntake.move(STOP);
	frontIntake.move(STOP);

	// Back away from long goal
	travelDistanceWithHeading(15.15, 35, 90, 2700);
	
	// Travel to next area
	turn_pid(0, 0);

	// Intake side balls
	travelDistanceWithHeading(101.25, 70, 0, -1);
	frontIntake.move(HIGH_VOLTAGE);
	mainIntake.move(HIGH_VOLTAGE);
	travelDistanceWithHeading(5.0, 18, 0, 1200);


	// Reverse to matchloader
	travelDistanceWithHeading(-12, 50, 0, -1);

	// Face matchloader
	turn_pid(90, 0);
	piston.set_value(true);
	pros::delay(1000);

	// Attack matchloader
	travelDistanceWithHeading(12.7, 33, 90, 1500);
	pros::delay(3000);

	// Back away from matchLoader
	travelDistanceWithHeading(-8, 35, 90, -1);
	piston.set_value(false);
	
	travelDistanceWithHeading(-20.15, 35, 90, 2700);
	mainIntake.move(-MID_VOLTAGE);
	backIntake.move(-MID_VOLTAGE);
	pros::delay(300);

	backIntake.move(MID_VOLTAGE);
	mainIntake.move(MID_VOLTAGE);
	frontIntake.move(MID_VOLTAGE);
	pros::delay(3000);

	backIntake.move(STOP);
	mainIntake.move(STOP);
	frontIntake.move(STOP);

}

void opcontrol() {
	// Set chassis brake mode to coast
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

	frontIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	mainIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	backIntake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	bool pistonToggle = false, pistonLatch = false;

	while(true){

		update_position_and_angle();
		// Get joystick values
		int leftY = controller.get_analog(ANALOG_LEFT_Y);
		int rightY = controller.get_analog(ANALOG_RIGHT_Y);

		// Dead zone for both motors
		if (!controller.get_digital(DIGITAL_UP) && !controller.get_digital(DIGITAL_DOWN)) {
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
		} else {
			if (controller.get_digital(DIGITAL_UP)) {
				leftMotors.move(LOW_VOLTAGE);
				rightMotors.move(LOW_VOLTAGE);
			} else if (controller.get_digital(DIGITAL_DOWN)){
				leftMotors.move(-LOW_VOLTAGE);
				rightMotors.move(-LOW_VOLTAGE);
			}
		}

		// Main intake
		if (controller.get_digital(DIGITAL_R1)) {
			frontIntake.move(HIGH_VOLTAGE);
			mainIntake.move(HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_R2)){
			frontIntake.move(-HIGH_VOLTAGE);
			mainIntake.move(-HIGH_VOLTAGE);
	    } else if (controller.get_digital(DIGITAL_L2)) {
			frontIntake.move(-HIGH_VOLTAGE);
			mainIntake.move(-HIGH_VOLTAGE);
			backIntake.move(-HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_L1)){
			frontIntake.move(HIGH_VOLTAGE);
			mainIntake.move(HIGH_VOLTAGE);
			backIntake.move(HIGH_VOLTAGE);
		} else if (controller.get_digital(DIGITAL_A)) {
			mainIntake.move(HIGH_VOLTAGE);
		} else {
			frontIntake.move(STOP);
			mainIntake.move(STOP);
			backIntake.move(STOP);
		}

		// // Top outtake (testing)
		// if(controller.get_digital(DIGITAL_X)) {
		// 	backIntake.move(HIGH_VOLTAGE);
		// } else if (controller.get_digital(DIGITAL_B)) {
		// 	backIntake.move(-HIGH_VOLTAGE);
		// } else {
		// 	backIntake.move(STOP);
		// }

		// Match unloader
		if (pistonToggle) {
			piston.set_value(true); // turns clamp solenoid on
		} else {
			piston.set_value(false); // turns clamp solenoid off
		}

		pros::delay(10);

		if (controller.get_digital_new_press(DIGITAL_LEFT)) {
			if(!pistonLatch){ // if latch is false, flip toggle one time and set latch to true
				pistonToggle = !pistonToggle;
				pistonLatch = true;
			}
		}
		else
			pistonLatch = false; // once button is released then release the latch too

		// Delay added to prevent crashing
		pros::delay(20);
	}

}