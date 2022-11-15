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
// FlywheelUp           motor         1               
// FLDrive              motor         2               
// Intake_Roller        motor         3               
// BLDrive              motor         4               
// FRDrive              motor         5               
// BRDrive              motor         6               
// FlywheelDown         motor         7               
// Controller1          controller                    
// Puncher              motor         8               
// inertialSensor       inertial      9               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

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

// ------------------------------------------------------- //
//                         TURNING
// ------------------------------------------------------- //
vex::timer Timer3 = vex::timer();
// spinbase
void spinLeft(double speed)
{
  BLDrive.spin(directionType::fwd, speed , velocityUnits::pct);
  FLDrive.spin(directionType::fwd, speed , velocityUnits::pct); 
}

void spinRight(double speed)
{
  BRDrive.spin(directionType::fwd, speed , velocityUnits::pct);
  FRDrive.spin(directionType::fwd, speed , velocityUnits::pct);
}

void spinBase(double Rspeed, double Lspeed)
{
  spinLeft(Lspeed);
  spinRight(Rspeed);
}

// stopbase
void stopBase()
{
  FLDrive.stop();
  BLDrive.stop();
  FRDrive.stop();
  BRDrive.stop();
}

// get inertial reading
double getInertialReading()
{
  // COMMENTED OUT BECAUSE WE DON"T HAVE AN INERTIAL SENSOR YET 
  double reading = inertialSensor.heading(degrees);
  reading = reading > 180 ? reading - 360 : reading;
  reading = reading < -180 ? reading + 360 : reading;
  return reading;
  return 0;
}

void calibrateIntertial()
{
  // COMMENTED OUT BECAUSE WE DON"T HAVE AN INERTIAL SENSOR YET 
  wait(3, sec);
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

void resetTrackers()
{
  FRDrive.resetPosition();
  BRDrive.resetPosition();
  FLDrive.resetPosition();
  BLDrive.resetPosition();
  // TrackerS.setRotation(0, degrees);
}

void Turn(double destination, double speed, double timeOut=5) {
  double curAngle, turnAmount, startAngle, output, speedR, speedL, absOutput, passed = 1000, PIDspeed, profile, curAngle2, tAngle, curAngleSgn, prevAngleSgn;
  double error, prevError, derivError, intError;
  bool doProfile = true, check = true;
  double incFrac = 0.3;
  double decFrac = 0.3;
  double Kd = 0.0; //0.01
  double Ki = 0.01; //0.0
  double Kp = 0.67; //0.67, recent 0.8
  double speedConst = 8;
  double threshold = 0.2;

  double loggedTime = 1000;
  double bounceTime = 0.7;

  bool triggered = false;

  Timer3.clear();
  double curTime = Timer3.time(vex::timeUnits::sec);


  startAngle = getInertialReading();
  turnAmount = (destination - startAngle);
  turnAmount = turnAmount > 180 ? turnAmount - 360 : turnAmount;
  turnAmount = turnAmount < -180 ? turnAmount + 360 : turnAmount;

  double prevAngle = startAngle;

  resetTrackers();

  bool switched = false;

  double sgn = turnAmount/fabs(turnAmount);

  do{

    curTime = Timer3.time(vex::timeUnits::sec);
    
    //tAngle = getRobotAngle()*(180.0/PI);
    curAngle = getInertialReading() - startAngle;
    //printf("og curAngle = %f\n",curAngle); //remove later

    curAngle = curAngle > 180 ? curAngle - 360 : curAngle;
    curAngle = curAngle < -180 ? curAngle + 360 : curAngle;

    curAngleSgn = curAngle/fabs(curAngle);
    prevAngleSgn = prevAngle/fabs(prevAngle);
    curAngle = (fabs(curAngle)>90 && fabs(prevAngle)>90) && (curAngleSgn == -1 && prevAngleSgn == 1)? curAngle+360 : curAngle;
    curAngle = (fabs(curAngle)>90 && fabs(prevAngle)>90) && (curAngleSgn == 1 && prevAngleSgn == -1)? curAngle-360 : curAngle;

    //_______________________________________________//
    //                PID Controller                 //
    //_______________________________________________//
    error = turnAmount - curAngle;
    derivError = error - prevError;
    intError = error + prevError;
    PIDspeed = Kp*error + Kd*derivError + Ki*intError;

    prevAngle = curAngle;

    wait(10, msec);
    prevError = error;

    /*

    profile = calculateDistanceVelocityProfile(fabs(curAngle), fabs(turnAmount), speed, incFrac, decFrac, speedConst);
    profile = turnAmount<0?-profile:profile;

    if(fabs(PIDspeed-profile)<5 || fabs(curAngle) > fabs(turnAmount*0.8))
    {switched=true;}

    if(switched)
    {output = PIDspeed;}
    else
    {output = profile;}

    */

    //JUST USE PID SPEED

    output = PIDspeed;

    speedR = -output;
    speedL = output;

    spinBase(speedR, speedL);

    //printf("turnamount=%f curAngle=%f error=%f output=%f profileSpeed=%f PIDcalculated=%f \n\n",turnAmount, curAngle, error, output, profile, PIDspeed);

    
    if(((curAngle>turnAmount-threshold && turnAmount>0) || (curAngle<turnAmount+threshold && turnAmount<0)) && !triggered)
    {
      loggedTime = curTime;
      triggered = true;
    }

    printf("loggedTime = %f, loggedTime+bounceTime = %f, curTime = %f\n", loggedTime, loggedTime+bounceTime, curTime);

  }while(curTime < bounceTime+loggedTime && curTime < timeOut);
  //((curAngle<turnAmount-threshold && turnAmount>0) || (curAngle>turnAmount+threshold && turnAmount<0)) && timeOut > curTime

  stopBase();
}

// --------------------------------------------- //
//                   DRIVE
// --------------------------------------------- //
vex::timer Timer1 = vex::timer();

const double PI = 3.14159265;
double r = 3.25/2; // radius of wheel or radius of robot?
double tWheelCirc = 3.25 * PI;
double magicOffset = 0.1;
double Al, Ar, As, rs, xPos1, yPos1;


float getRightTrackerArc() {return ((FRDrive.position(rev)+BRDrive.position(rev))*tWheelCirc*0.66666666*magicOffset);}
float getLeftTrackerArc() {return ((FLDrive.position(rev)+BLDrive.position(rev))*tWheelCirc*0.66666666*magicOffset);}
// float getSidewaysTrackerArc() {return ((TrackerS.position(rev))*tWheelCircS);}
float getRobotAngle()
{
  float leftArc = getLeftTrackerArc();
  float rightArc = getRightTrackerArc();
  float botAngle = (leftArc-rightArc)/(2*r);
  return botAngle;
}

void LinTrackOld()
{
  double Al_prev, Ar_prev, As_prev, theta, rdx, rdy, dx, dy, theta_prev=0, dTheta=0;
  // record current tracker measurements
  Al_prev = getLeftTrackerArc();
  Ar_prev = getRightTrackerArc();
  // As_prev = getSidewaysTrackerArc();
  theta_prev = getRobotAngle();

  // wait duh
  wait(10, msec);

  // calculate changes in encoder
  Al = getLeftTrackerArc() - Al_prev;
  Ar = getRightTrackerArc() - Ar_prev;
  // As = getSidewaysTrackerArc() - As_prev;
  theta = getRobotAngle();
  dTheta = theta - theta_prev;

  // calculate relative movement components
  rdx = As + dTheta*rs;
  rdy = (Ar + Al)/2;

  // calculate global movement components
  dy = rdy*cos(theta) - rdx*sin(theta);
  dx = rdx*cos(theta) + rdy*sin(theta);

  // update global position
  xPos1 += dx;
  yPos1 += dy; 

  //printf("Al = %f, Ar = %f, As = %f\n",getLeftTrackerArc(), getRightTrackerArc(), getSidewaysTrackerArc());
  //printf("Alp = %f, Arp = %f, Asp = %f\n",Al_prev, Ar_prev, As_prev);
}

void DriveS(double amount, double speed, double timeOut = 10, bool fast = false)
{
  resetTrackers();
  double curDist;
  double output, error, intError, derivError, prevOutput=0, prevError=amount, profile, sgn, Alfull=0, Arfull=0, angleCur, Asfull=0;
  double x=0, y=0, dl, dr, ds, dtheta, theta, dx, dy, dhM, dlPrev = 0, drPrev = 0, dsPrev = 0, thetaPrev = 0;
  double speedR=0, speedL=0;
  double vel=0;

  Timer1.clear();
  double curTime = Timer1.time(vex::timeUnits::sec);

  double inc = !fast?10:500;

  // PID CONSTANTS
  double Ki = 0.0;
  double Kd = 0.0;
  double Kp = 3.0;

  double Ks3 = 200;

  double threshold = 0.001;

  double displacement = 0, targetAngle = 0, angleError = 0;

  xPos1 = 0;
  yPos1 = 0;
  Al=0;
  Ar=0;
  As=0;

  sgn = (amount/fabs(amount));
  amount = fabs(amount);

  double angleoffset = 0.0; //3*(PI/180);


  do
  {
    //_______________________________________________//
    //                LINEAR TRACKING                //
    //_______________________________________________//
    
    LinTrackOld();

    //_______________________________________________//

    // get current time
    curTime = Timer1.time(vex::timeUnits::sec);

    curDist = fabs(yPos1);
    displacement = xPos1; //unused
    vel = fabs((FRDrive.velocity(pct)+BRDrive.velocity(pct)+FLDrive.velocity(pct)+BLDrive.velocity(pct))/4.0);

    //_______________________________________________//
    //                PID Controller                 //
    //_______________________________________________//
    error = amount - curDist;
    derivError = error - prevError;
    intError = error + prevError;
    prevError = error;

    output = !fast?Kp*error + Kd*derivError + Ki*intError:100;
    output = output>vel+inc?vel+inc:output;
    output = output>100.0?100.0:output;
    
    //printf("x=%f e=%f ie=%f de=%f o=%f vel=%f\n",curDist, error, intError, derivError, output, vel);

    //_______________________________________________//
    //                 ANGLE CORRECT                 //
    //_______________________________________________//

    angleCur = getRobotAngle(); // can use inertial reading instead? tag2
    angleError = (targetAngle - angleCur)-angleoffset;

    if((angleCur < targetAngle && sgn>0) || (angleCur > targetAngle && sgn<0))
    {

      speedL = output;
      speedR = output - Ks3*fabs(angleError);
      
    }
    else if((angleCur > targetAngle && sgn>0) || (angleCur < targetAngle && sgn<0))
    {

      speedL = output - Ks3*fabs(angleError);
      speedR = output;
      
    }
    else if(Alfull == Arfull)
    {
      speedR = output;
      speedL = output;
    }

  //_______________________________________________//

  printf("x=%f, disp=%f, targetA=%f, robotA=%f, angleE=%f, output=%f, Ks3=%f, SpeedL=%f, SpeedR=%f\n",curDist, displacement, targetAngle*(180/PI), angleCur*(180/PI), angleError*(180/PI), output, Ks3, speedL, speedR);

  speedL = speedL<0?0:speedL;
  speedR = speedR<0?0:speedR;
  spinBase(speedR*sgn, speedL*sgn);


  //printf("curDist=%f output=%f angleCur=%f\n", curDist, output, angleCur);
  }while(((curDist<amount-threshold && amount>0) || (curDist>amount+threshold && amount<0)) && curTime < timeOut);
  
  stopBase();

}



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

// --------------------------------------------------------------- //
//                       FLYWHEEL PID FUNCTION
// --------------------------------------------------------------- //

double kp = 0.2;
double ki = 0.1;
double kd = 0.05;
double prevError = 0.0;
double error = 0.0;
double totalError = 0.0; // += error
double der = 0.0; // = error-preverror
double speedAvg = 0;
double Power = 0;
bool ReadyShoot = false;
double maxspeed = 4200; //4200 rpm
double puncherTime = 500;

void FlywheelPID(double targetSpeed) {
  while(true){
    speedAvg = (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm))/2; 
    error = targetSpeed - speedAvg;
    if (error <= 0.5){ //less that 0.5 RPM from target RPM
      ReadyShoot = true;
    }
    else{
      ReadyShoot = false;
    }
    Power = (error*kp + totalError * ki + (error - prevError) * kd)/12; ///12 for voltage
    FlywheelUp.spin(forward, Power, volt); //final output in volts
    FlywheelDown.spin(forward, Power, volt);
    prevError = error; //derivative
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


    double Axis3Adjusted = Controller1.Axis3.position();
    double Axis1Adjusted = Controller1.Axis1.position();


    double outputL = (Axis3Adjusted + (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    double outputR = (Axis3Adjusted - (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));


    // --------------------------------------------------------------- //
    // These motor commands need to be renamed or assigned to new, 
    // initialized motors in the motor config section. 
    // Error Tag: tag1
    // --------------------------------------------------------------- //

    // LeftOut.setStopping(coast);
    // RightOut.setStopping(coast);
    // LeftIn.setStopping(coast);
    // RightIn.setStopping(coast);

    // LeftIn.spin(forward, outputL, pct);
    // LeftOut.spin(forward, outputL, pct);
    // RightIn.spin(forward, outputR, pct);
    // RightOut.spin(forward, outputR, pct);

    // if (ct.ButtonA.pressing() && !aPrev) {
    //   trans.set(!trans.value());
    // }

    // aPrev = ct.ButtonA.pressing();


    // Driver Control Code --------------------------------------------------------------------------------

    // BLDrive.spin(forward, ct.Axis3.value() + (ct.Axis1.value() * 0.5), percent);
    // FLDrive.spin(forward, ct.Axis3.value() + (ct.Axis1.value() * 0.5), percent);
    // BRDrive.spin(forward, ct.Axis3.value() - (ct.Axis1.value() * 0.5), percent);
    // FRDrive.spin(forward, ct.Axis3.value() - (ct.Axis1.value() * 0.5), percent);
    
    double smoothFactor = 0;


    // double leftVelocity = (LeftOut.velocity(percent)  + LeftIn.velocity(percent)) / 2;
    // double rightVelocity = (RightOut.velocity(percent)  + RightIn.velocity(percent)) / 2;

    // outputL = (outputL + leftVelocity * smoothFactor) / (smoothFactor + 1);
    // outputR = (outputR + rightVelocity * smoothFactor) / (smoothFactor + 1);

    //printf("OL = %f, OR = %f, F = %f, S = %f\n", outputL, outputR, Axis3Adjusted, Axis1Adjusted);
    //printf("vel=%f, rO=%f\n",outputR, rightOffset);
    ///*

    // LeftIn.spin(forward, (outputL / 100)  * 12, volt);
    // LeftOut.spin(forward, (outputL / 100)  * 12, volt);
    // RightOut.spin(forward, (outputR / 100)  * 12, volt);
    // RightIn.spin(forward, (outputR / 100)  * 12, volt);

    //--------------------------------------------------------------------------------------------------------

    // INTAKE / ROLLER CODE -----------------------------------------------------
    while(true) {
      if(Controller1.ButtonL1.pressing())
      {
        Intake_Roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      }
    }

    if(Controller1.ButtonL2.pressing())
    {
      Intake_Roller.stop(brakeType::hold);
    }
    
    
    // FLYWHEEL CODE --------------------------------------------------
    while (true) {
      if (Controller1.ButtonR1.pressing()) {
        //make the flywheel go brrr
        FlywheelPID(maxspeed);
      }
    }


    // PUNCHER CODE --------------------------------------------------
    if (Controller1.ButtonR2.pressing()) {
      // puncher moves up a certain amount.
      Puncher.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      wait(puncherTime, msec);
      Puncher.stop();
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
