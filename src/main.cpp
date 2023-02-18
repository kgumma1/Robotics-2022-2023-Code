/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "drive.h"
#include "autons.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
double topIntakeSensorInit;
double bottomIntakeSensorInit;
double flywheelSensorInit;
double matchLoadSensorInit;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void calibrateLineSensors() {
  wait(1, sec);
  topIntakeSensorInit = 0;
  bottomIntakeSensorInit = 0;
  flywheelSensorInit = 0;
  matchLoadSensorInit = 0;
  for (int i = 0; i < 10; i++) {
    topIntakeSensorInit += topIntakeSensor.reflectivity();
    bottomIntakeSensorInit += bottomIntakeSensor.reflectivity();
    flywheelSensorInit += flywheelSensor.reflectivity();
    matchLoadSensorInit += matchLoadSensor.reflectivity();
    wait(20, msec);
  }
  topIntakeSensorInit /= 10.0;
  bottomIntakeSensorInit /= 10.0;
  flywheelSensorInit /= 10.0;
  matchLoadSensorInit /= 10.0;
}

void calibrateIntertial()
{
  // COMMENTED OUT BECAUSE WE DON"T HAVE AN INERTIAL SENSOR YET 
  wait(1, sec);
  inertialSensor.calibrate();
  Brain.Screen.clearScreen();
  Brain.Screen.print("...");
  Controller.Screen.print("...");
  while (inertialSensor.isCalibrating())
  {
    wait(10, msec);
  }

  Controller.Screen.print("Done");
  Brain.Screen.print("Done");
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  calibrateLineSensors();
  calibrateIntertial();

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

  //twoRoller();
  //farRoller();
  //roller();
  testing();
  //skills();
  //winPoint6(true);
  //rightSide5(false);
  //halfWP();
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  //matchLoadTest();
  drive();
  //testing();
  // testing without comp switch code
  while(!Controller.ButtonA.pressing()) {
    wait(5, msec);
  }
  //twoRoller();
  //winPoint();
  //drive();
  //winPoint();
  //testing();
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
