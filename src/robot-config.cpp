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
// define variables used for controlling motors based on controller inputs
bool Controller1RightShoulderControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // check the ButtonR1/ButtonR2 status to control Puncher
      if (Controller1.ButtonR1.pressing()) {
        Puncher.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Puncher.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Puncher.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}