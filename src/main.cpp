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
  int i = 0;
  Brain.Screen.print(i+1);
  // driveBasic(2000, 60);
  // Brain.Screen.print(i+1);
  // Turn(50, 60);
  // Brain.Screen.print(i+1);
  // driveFwdPID(24); // inches
  // inertTurnDegPID(90, 0.25); // target & kp
  /*while ((FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2 < 350)  {
    FlywheelDown.spin(forward, 3400.0 / 7 / 50, volt);
    FlywheelUp.spin(forward, 3400.0 / 7 / 50, volt);
  }
  wait(1500, msec);
  Indexer.set(true);
  wait(300, msec);
  Indexer.set(false);
  wait(1800, msec);
  Indexer.set(true);
  wait(300, msec);
  Indexer.set(false);*/
  //matchWP3();
  //skills();
  halfWP();
  //testing();
  //matchFarRoller();
  /*
  driveFwdPID(25, false);
  BLDrive.spinFor(forward, 110, rotationUnits::deg, 50, velocityUnits::pct, false);
  FLDrive.spinFor(forward, 110, rotationUnits::deg, 50, velocityUnits::pct, false);
  BRDrive.spinFor(reverse, 110, rotationUnits::deg, 50, velocityUnits::pct, false);
  FRDrive.spinFor(reverse, 110, rotationUnits::deg, 50, velocityUnits::pct, true); 
  StringShooters.set(true);
  /*driveFwdPID(22, true);
  spinLeft(5);
  spinRight(5);
  wait(1000, msec);
  Intake_Roller.spinFor(0.5, rotationUnits::rev, 100, velocityUnits::pct);*/


/*
  
  spinLeft(5);
  spinRight(5);
  wait(1000, msec);
  Intake_Roller.spinFor(0.15, rotationUnits::rev, 100, velocityUnits::pct);
  wait(100, msec);
  stopBase();
  driveFwdPID(3, false, 10);
  stopBase();
  /*BLDrive.spinFor(reverse, 30, rotationUnits::deg, 50, velocityUnits::pct, false);
  FLDrive.spinFor(reverse, 30, rotationUnits::deg, 50, velocityUnits::pct, false);
  BRDrive.spinFor(forward, 30, rotationUnits::deg, 50, velocityUnits::pct, false);
  FRDrive.spinFor(forward, 30, rotationUnits::deg, 50, velocityUnits::pct, true);
  Fly(3100, 400, 2, 0.30);*/
  //driveFwdPID(5, false, 7);
  /**//*
  BLDrive.spinFor(reverse, 10, rotationUnits::deg, 50, velocityUnits::pct, false);
  FLDrive.spinFor(reverse, 10, rotationUnits::deg, 50, velocityUnits::pct, false);
  BRDrive.spinFor(forward, 10, rotationUnits::deg, 50, velocityUnits::pct, false);
  FRDrive.spinFor(forward, 10, rotationUnits::deg, 50, velocityUnits::pct, true);
  wait(0.5, sec);
  inertTurnDegPID(233, 0.5, false);
  //vex::task t = vex::task(intakeNum);
  Intake_Roller.spin(forward, 100, pct);
  driveFwdPID(134);
  inertTurnDegPID(270, 0.5, true);
  spinLeft(15);
  spinRight(15);
  wait(2, sec);
  Intake_Roller.spinFor(0.15, rotationUnits::rev, 100, velocityUnits::pct);*/

  /*
  spinLeftBack(10);
  spinRightBack(10);
  wait(500, msec);
  inertTurnDegPID(270, 0.25);
  Intake_Roller.spinFor(10, rotationUnits::rev, 100, velocityUnits::pct, false);
  driveFwdPID(46);
  inertTurnDegPID(45, 0.25);
  Intake_Roller.stop(coast);
  Fly(2000, 380, 3, 0.3);
  inertTurnDegPID(270, 0.25);
  Intake_Roller.spinFor(10, rotationUnits::rev, 100, velocityUnits::pct, false);
  driveFwdPID(30);
  inertTurnDegPID(90, 0.25);
  driveFwdPID(30);
  inertTurnDegPID(180, 0.25);
  Intake_Roller.spinFor(10, rotationUnits::rev, 100, velocityUnits::pct, false);
  driveFwdPID(20);
  Fly(3000, 380, 3, 0.3);*/


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
