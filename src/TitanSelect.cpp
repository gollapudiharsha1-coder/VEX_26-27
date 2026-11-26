#include "titanselect/titanselect.hpp"

// Global selector object pointer
ts::selector* selector = ts::selector::get();

void left_auton() {
    // Left-side autonomous routine
}

void right_auton() {
    // Right-side autonomous routine
}

// Register autonomous routines
ts::auton left("Left", left_auton);
ts::auton right("Right", right_auton);

void autonomous() {
    // Run the currently selected auton
	selector->run_selected_auton();
}

void initialize() {
    // Display selector
    selector->display();
}