#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FlywheelUp = motor(PORT7, ratio6_1, true);
motor FLDrive = motor(PORT2, ratio18_1, false);
motor Intake_Roller = motor(PORT3, ratio18_1, false);
motor BLDrive = motor(PORT4, ratio18_1, true);
motor FRDrive = motor(PORT5, ratio18_1, true);
motor BRDrive = motor(PORT6, ratio18_1, false);
motor FlywheelDown = motor(PORT1, ratio6_1, false);
controller Controller1 = controller(primary);
motor Puncher = motor(PORT20, ratio18_1, false);
inertial inertialSensor = inertial(PORT12);
digital_out trans = digital_out(Brain.ThreeWirePort.A);
digital_out Indexer = digital_out(Brain.ThreeWirePort.B);

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