#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
triport Expander = triport(PORT15);
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
digital_out StringShooters = digital_out(Expander.A);
digital_out angleChanger = digital_out(Expander.G);
line IntakeSensor = line(Expander.B);
encoder rightEncoder = encoder(Brain.ThreeWirePort.E);
encoder leftEncoder = encoder(Brain.ThreeWirePort.C);
encoder backEncoder = encoder(Brain.ThreeWirePort.G);
rotation rotSensor = rotation(PORT13);
signature Vision__SIG_1 = signature (1, 2321, 8611, 5466, -351, 819, 234, 1.1, 0);
signature Vision__SIG_2 = signature (2, -2299, -321, -1310, 1505, 8977, 5241, 0.7, 0);
signature Vision__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision = vision (PORT20, 50, Vision__SIG_1, Vision__SIG_2, Vision__SIG_3, Vision__SIG_4, Vision__SIG_5, Vision__SIG_6, Vision__SIG_7);

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