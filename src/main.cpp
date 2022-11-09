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
// BumperH              bumper        H               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

vex::motor LeftOut(vex::PORT20, true);
vex::motor LeftIn(vex::PORT19, false);
vex::motor RightOut(vex::PORT11, false);
vex::motor RightIn(vex::PORT18, true);
vex::motor FlywheelUp(vex::PORT17, true);
vex::motor FlywheelDown(vex::PORT17, true);
vex::motor Puncher(vex::PORT17, true);
vex::motor Intake_Roller(vex::PORT17, true);
motor_group Flywheel = motor_group(FlywheelUp, FlywheelDown);



vex::controller ct;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
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

bool maintainSpeed = false;
double kp = 0.2;
double ki = 0.1;
double kd = 0.05;
double prevError = 0.0;
double error = 0.0;
double totalError = 0.0; // += error
double derivative = 0.0; // = error-preverror
double speedAvg;
double Power = 0;
bool ReadyShoot = false;

void FlywheelPID(double targetSpeed) {
  while(maintainSpeed){
    speedAvg = (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm))/2; 
    error = targetSpeed - speedAvg;
    if (error <= 0.5){
      ReadyShoot = true;
    }
    else{
      ReadyShoot = false;
    }
    Power = (error*kp + totalError * ki + (error - prevError) * kd)/12;
    Flywheel.spin(forward, Power, volt);
    prevError = error;
    totalError += error;
    wait(20, msec);

  }
}

void usercontrol(void) {
  // User control code here, inside the loop

  bool aPrev = false;
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    double initSens = 0.8;
    double sensInc = -0.0001;


    double Axis3Adjusted = ct.Axis3.position();
    double Axis1Adjusted = ct.Axis1.position();


    double outputL = (Axis3Adjusted + (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    double outputR = (Axis3Adjusted - (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));

    LeftOut.setStopping(coast);
    RightOut.setStopping(coast);
    LeftIn.setStopping(coast);
    RightIn.setStopping(coast);

    LeftIn.spin(forward, outputL, pct);
    LeftOut.spin(forward, outputL, pct);
    RightIn.spin(forward, outputR, pct);
    RightOut.spin(forward, outputR, pct);

    if (ct.ButtonA.pressing() && !aPrev) {
      trans.set(!trans.value());
    }

    aPrev = ct.ButtonA.pressing();


    // Driver Control Code --------------------------------------------------------------------------------

    // BLDrive.spin(forward, ct.Axis3.value() + (ct.Axis1.value() * 0.5), percent);
    // FLDrive.spin(forward, ct.Axis3.value() + (ct.Axis1.value() * 0.5), percent);
    // BRDrive.spin(forward, ct.Axis3.value() - (ct.Axis1.value() * 0.5), percent);
    // FRDrive.spin(forward, ct.Axis3.value() - (ct.Axis1.value() * 0.5), percent);
    
    double smoothFactor = 0;


    double leftVelocity = (LeftOut.velocity(percent)  + LeftIn.velocity(percent)) / 2;
    double rightVelocity = (RightOut.velocity(percent)  + RightIn.velocity(percent)) / 2;

    outputL = (outputL + leftVelocity * smoothFactor) / (smoothFactor + 1);
    outputR = (outputR + rightVelocity * smoothFactor) / (smoothFactor + 1);

    //printf("OL = %f, OR = %f, F = %f, S = %f\n", outputL, outputR, Axis3Adjusted, Axis1Adjusted);
    //printf("vel=%f, rO=%f\n",outputR, rightOffset);
    ///*

    LeftIn.spin(forward, (outputL / 100)  * 12, volt);
    LeftOut.spin(forward, (outputL / 100)  * 12, volt);
    RightOut.spin(forward, (outputR / 100)  * 12, volt);
    RightIn.spin(forward, (outputR / 100)  * 12, volt);

    //--------------------------------------------------------------------------------------------------------

    while(true) {
      if(ct.ButtonL1.pressing())
      {
        Intake_Roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      }
    }

    if(ct.ButtonL2.pressing())
    {
      Intake_Roller.stop(brakeType::hold);
    }
    
    
    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
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
