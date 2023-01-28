#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller = controller(primary);



motor flywheel = motor(PORT10, ratio6_1, false);
motor intake_roller = motor(PORT1, ratio6_1, false);

motor RFDrive = motor (PORT16, ratio6_1, false);
motor RMDrive = motor(PORT19, ratio6_1, false);
motor RBDrive = motor(PORT20, ratio6_1, false);
motor_group rDrive = motor_group(RFDrive, RMDrive, RBDrive);

motor LFDrive = motor (PORT13, ratio6_1, true);
motor LMDrive = motor(PORT12, ratio6_1, true);
motor LBDrive = motor(PORT11, ratio6_1, true);
motor_group lDrive = motor_group(LFDrive, LMDrive, LBDrive);


triport Expander = triport(PORT2);
line bottomIntakeSensor = line(Expander.B);
line topIntakeSensor = line(Expander.A);
line flywheelSensor = line(Brain.ThreeWirePort.F);

encoder leftEncoder = encoder(Brain.ThreeWirePort.A);
encoder backEncoder = encoder(Brain.ThreeWirePort.G);

inertial inertialSensor = inertial(PORT14);
optical leftRollerSensor = optical(PORT3);
optical rightRollerSensor = optical(PORT4);

digital_out angleChanger = digital_out(Expander.F);
digital_out basket = digital_out(Expander.G);
digital_out intakeLift = digital_out(Expander.H);
digital_out expansion = digital_out(Brain.ThreeWirePort.D);

bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}