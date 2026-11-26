#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "TitanReset/TitanReset.hpp"
#include "TitanReset/TRChassis.hpp"
#include "TitanReset/TRSensor.hpp"
#include "TitanReset/TRTypes.hpp"
#include "TitanReset/TRConstants.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "titanselect/titanselect.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups

pros::MotorGroup leftMotors({-5, 4},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 
pros::MotorGroup rightMotors({6, -9}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 
pros::MotorGroup lift_mg({1, -2}, pros::MotorGearset::blue); // lift motor group - ports 6, 7

// individual motors
pros::Motor intake(3, pros::MotorGearset::blue);
pros::Motor claw(8, pros::MotorGearset::green);
pros::Motor claw_rotator(12, pros::MotorGearset::green);

// DSR Titan Reset Lib Sensors
/**
 * North sensor
 * X is your parrallel offset
 * Y is your perpendicular offset
 * P is the port of your sensor
 */
tr_sensor north({0.949, -6.789}, 10);

/**
 * East sensor
 * X is your parrallel offset
 * Y is your perpendicular offset
 * P is the port of your sensor
 */
tr_sensor east({5.912, -0.394}, 11);

/**
 * South sensor
 * X is your parrallel offset
 * Y is your perpendicular offset
 * P is the port of your sensor
 */
tr_sensor south({-0.978, 6.349}, 12);

/**
 * West sensor
 * X is your parrallel offset
 * Y is your perpendicular offset
 * P is the port of your sensor
 */
tr_sensor west({-7.403, 1.347}, 13);



// Inertial Sensor on port 10
pros::Imu imu(10);

// DSR Titan Reset Chassis
extern lemlib::Chassis chassis;

/**
 * TitanReset Chassis
 * &imu - pointer to IMU
 * &chassis - pointer to LemLib chassis
 * {&north, &east, &south, &west} - Array of pointers to sensors.
 */
tr_chassis dsr_system(&chassis, {&north, &east, &south, &west}, tr_fields::plastic);


// tracking wheels

// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.53);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -1.03);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.94, // 12.94 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Global LVGL label
lv_obj_t* poseLabel = nullptr;

// Function to update coordinates
void updateCoordinates(lv_timer_t* timer) {

    // Get current LemLib position
    auto pose = chassis.getPose();

    // Update the LVGL label
    lv_label_set_text_fmt(
        poseLabel,
        "X: %.2f | Y: %.2f | Theta: %.2f",
        pose.x,
        pose.y,
        pose.theta
    );
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


void initialize() {
    // Calibrate LemLib
    chassis.calibrate();

    // Display TitanSelect
    ts::selector::get()->display();

    // Create coordinate label
    poseLabel = lv_label_create(lv_screen_active());

    lv_obj_align(poseLabel, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_text_color(poseLabel, lv_color_white(), 0);

    lv_label_set_text(
        poseLabel,
        "X: 0.00 | Y: 0.00 | Theta: 0.00"
    );

    // Update coordinates every 50ms
    lv_timer_create(updateCoordinates, 50, nullptr);
}



/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

//auton is in the TitanSelect.cpp file

/**
 * Runs in driver control
 */
void opcontrol() {
    bool claw_rotation_state = false;  // Variable to track the state of the claw rotation
    // controller
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
			//intake functions
		
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {  // If L1 is pressed
			intake.move(127);
			claw.move(127);                 // Intake motor moves forward at full speed
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {  // If L2 is pressed
			intake.move(-127);
			claw.move(-127);                // Intake motor moves backward at full speed
		} else {
			intake.move(0);
			claw.move(0);                   // Intake motor stops moving
		}
		
		//scoring funcitions
		
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {  // If Y is pressed
			claw.move(-127);    
			claw_rotator.move_absolute(45, 127);         // Lift motor group moves forward at full speed    
		} else {
			claw.move(0);  
			claw_rotator.move_absolute(0, 127);             // Lift motor group stops moving
		}
		
		//claw functions
		
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {  // If X is newly pressed
			if(claw_rotation_state==false) {  // If the claw is not rotated
				claw_rotator.move_absolute(90, 127);  // Rotate the claw to 90 degrees at full speed
				claw_rotation_state = true;            // Update the state to indicate the claw is rotated
			} else {  // If the claw is already rotated
				claw_rotator.move_absolute(0, 127);   // Rotate the claw back to 0 degrees at full speed
				claw_rotation_state = false;           // Update the state to indicate the claw is not rotated
			}
		}

		// Lift functions
		
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			lift_mg.move(127);  // Lift motor moves forward at full speed
		} else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			lift_mg.move(-127);  // Lift motor moves backward at full speed
		} else {
			lift_mg.move(0);  // Lift motor stops moving
		}
        // delay to save resources
        pros::delay(10);
    }
}