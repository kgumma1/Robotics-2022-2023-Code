#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FlywheelUp = motor(PORT1, ratio18_1, false);
motor FLDrive = motor(PORT2, ratio18_1, false);
motor Intake_Roller = motor(PORT3, ratio18_1, false);
motor BLDrive = motor(PORT4, ratio18_1, false);
motor FRDrive = motor(PORT5, ratio18_1, false);
motor BRDrive = motor(PORT6, ratio18_1, false);
motor FlywheelDown = motor(PORT7, ratio18_1, true);
controller Controller1 = controller(primary);
motor Puncher = motor(PORT8, ratio18_1, false);
inertial inertialSensor = inertial(PORT9);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}