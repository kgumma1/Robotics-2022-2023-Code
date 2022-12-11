// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FlywheelUp           motor         7               
// FLDrive              motor         2               
// Intake_Roller        motor         3               
// BLDrive              motor         4               
// FRDrive              motor         5               
// BRDrive              motor         6               
// FlywheelDown         motor         1               
// Controller1          controller                    
// Puncher              motor         20              
// inertialSensor       inertial      12              
// trans                digital_out   A               
// Indexer              digital_out   B               
// Expander             triport       15              
// StringShooters       digital_out   A               
// IntakeSensor         line          H               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FlywheelUp           motor         7               
// FLDrive              motor         2               
// Intake_Roller        motor         3               
// BLDrive              motor         4               
// FRDrive              motor         5               
// BRDrive              motor         6               
// FlywheelDown         motor         1               
// Controller1          controller                    
// Puncher              motor         20              
// inertialSensor       inertial      12              
// trans                digital_out   A               
// Indexer              digital_out   B               
// Expander             triport       15              
// StringShooters       digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FlywheelUp           motor         7               
// FLDrive              motor         2               
// Intake_Roller        motor         3               
// BLDrive              motor         4               
// FRDrive              motor         5               
// BRDrive              motor         6               
// FlywheelDown         motor         1               
// Controller1          controller                    
// Puncher              motor         20              
// inertialSensor       inertial      12              
// trans                digital_out   A               
// Indexer              digital_out   B               
// Expander             triport       15              
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FlywheelUp           motor         7               
// FLDrive              motor         2               
// Intake_Roller        motor         3               
// BLDrive              motor         4               
// FRDrive              motor         5               
// BRDrive              motor         6               
// FlywheelDown         motor         1               
// Controller1          controller                    
// Puncher              motor         20              
// inertialSensor       inertial      12              
// trans                digital_out   A               
// Indexer              digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "drive.h"
#include "autons.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
double intakeSensorInit;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous FunController1ions                         */
/*                                                                           */
/*  You may want to perform some aController1ions before the competition starts.      */
/*  Do them in the following funController1ion.  You must return from this funController1ion   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  funController1ion is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/




void calibrateIntakeSensor() {
  wait(1, sec);
  for (int i = 0; i < 10; i++) {
    intakeSensorInit += IntakeSensor.reflectivity();
    wait(20, msec);
  }
  intakeSensorInit /= 10.0;
}

void calibrateIntertial()
{
  // COMMENTED OUT BECAUSE WE DON"T HAVE AN INERTIAL SENSOR YET 
  wait(1, sec);
  inertialSensor.calibrate();
  Brain.Screen.clearScreen();
  Brain.Screen.print("...");
  Controller1.Screen.print("...");
  while (inertialSensor.isCalibrating())
  {
    vex::task::sleep(20);
  }

  Controller1.Screen.print("Done");
  Brain.Screen.print("Done");
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  rotSensor.resetPosition();
  vexcodeInit();
  calibrateIntakeSensor();
  calibrateIntertial();

  // All aController1ivities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  //matchWP3();
  //matchWP5();
  matchFarRoller();
  //skills();
  ////halfWP();
  //testing();
  //matchFarRoller();


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

// --------------------------------------------------------------- //
//                       FlyWHEEL PID FUNController1ION
// --------------------------------------------------------------- //


void usercontrol(void) {
  // User control code here, inside the loop
  //testing();
  drive();
}

//
// Main will set up the competition funController1ions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous funController1ion.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
