#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller = controller(primary);

motor cataMain = motor(PORT12, ratio18_1, false); 
motor intake_roller_cata = motor(PORT20, ratio18_1, false);

// CATA IS FRONT

motor RFDrive = motor(PORT17, ratio6_1, false);
motor RMDrive = motor(PORT10, ratio6_1, false);
motor RBDrive = motor(PORT18, ratio6_1, false);
motor_group rDrive = motor_group(RFDrive, RMDrive, RBDrive);

motor LFDrive = motor (PORT11, ratio6_1, true); 
motor LMDrive = motor(PORT14, ratio6_1, true);
motor LBDrive = motor(PORT19, ratio6_1, true);
motor_group lDrive = motor_group(LFDrive, LMDrive, LBDrive);


triport Expander = triport(PORT1);

line bottomIntakeSensor = line(Expander.D);

encoder leftEncoder = encoder(Expander.A);
encoder backEncoder = encoder(Brain.ThreeWirePort.G);

inertial inertialSensor = inertial(PORT15);
optical leftRollerSensor = optical(PORT2);// NOT ON ROBOT
optical rightRollerSensor = optical(PORT3);// NOT ON ROBOT
distance distanceSensor = distance(PORT2);
rotation cataSensor = rotation(PORT9);

digital_out intakeLift = digital_out(Brain.ThreeWirePort.D);
digital_out expansionLeft = digital_out(Brain.ThreeWirePort.A);
digital_out expansionRight = digital_out(Expander.C);
digital_out pistonBoost = digital_out(Brain.ThreeWirePort.C);
digital_out bandBoost = digital_out(Brain.ThreeWirePort.B);

bool RemoteControlCodeEnabled = true;

// GLOBAL FUNCTIONS

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}