#include "main.h"
#include "titanselect/titanselect.hpp"

void red_left_auton()
{
	//Some super cool auton code.
}

void blue_right_auton()
{
    //Some super cool auton code.
}

void blue_left_auton()
{
    //Some super cool auton code.
}

void red_right_auton()
{
    //Some super cool auton code.
}

ts::auton rl("Red Left", red_left_auton);
ts::auton br("Blue Right", blue_right_auton);
ts::auton rr("Red Right", red_right_auton);
ts::auton bl("Blue Left", blue_left_auton);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void autonomous() {
	ts::selector::get()->run_selected_auton();
}
