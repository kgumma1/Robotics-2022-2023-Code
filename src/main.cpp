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

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous FunController1ions                         */
/*                                                                           */
/*  You may want to perform some aController1ions before the competition starts.      */
/*  Do them in the following funController1ion.  You must return from this funController1ion   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  funController1ion is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

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

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
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

// ------------------------------------------------------- //
//                         TURNING
// ------------------------------------------------------- //
vex::timer Timer3 = vex::timer();
// spinbase
void spinLeft(double speed)
{
  BLDrive.spin(fwd, speed , velocityUnits::pct);
  FLDrive.spin(fwd, speed , velocityUnits::pct); 
}

void spinRight(double speed)
{
  BRDrive.spin(fwd, speed , velocityUnits::pct);
  FRDrive.spin(fwd, speed , velocityUnits::pct);
}

void spinLeftV(double speed)
{

  BLDrive.spin(speed > 0 ? forward : reverse, fabs(speed/8.333333) , volt);
  FLDrive.spin(speed > 0 ? forward : reverse, fabs(speed/8.333333), volt); 
}

void spinRightV(double speed)
{
  BRDrive.spin(speed > 0 ? forward : reverse, fabs(speed/8.333333) , volt);
  FRDrive.spin(speed > 0 ? forward : reverse, fabs(speed/8.333333) , volt);
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

void driveBasic(double MseController1ime, double Speed){
  spinLeft(Speed);
  spinRight(Speed);
  wait(MseController1ime, msec);
  stopBase();
}

// get inertial reading




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

void driveFwdPID(double dist)  //inches

{

  double kP = 4.5; //0.185

  double kI = 0;

  double kD = 0;

  double error = 2;

  double integral = 0;

  double derivative = 0;

  double prevError = 0;

  double powers = 0;

  double avgPos = (fabs(FLDrive.position(rotationUnits::deg)) + fabs(BLDrive.position(rotationUnits::deg)) + fabs(FRDrive.position(rotationUnits::deg)) + fabs(BLDrive.position(rotationUnits::deg)))/4;

  BRDrive.resetPosition(); //((frontMotorB.position(rotationUnits::deg)/360) * 12.96)

  BLDrive.resetPosition();

  FLDrive.resetPosition();

  FRDrive.resetPosition();

  //The following formula is used to convert an angle in degrees to length in feet.

  //To calculate feet from degrees, divide the angle by 360, multiply by 2 times pi, then finally, multiply by the radius. Then multiply by 12 to get inches

  while(error > 0.5)

  {
    double newPos = (fabs(FLDrive.position(rotationUnits::deg)) + fabs(BLDrive.position(rotationUnits::deg)) + fabs(FRDrive.position(rotationUnits::deg)) + fabs(BLDrive.position(rotationUnits::deg)))/4;
    
    error = dist - (((newPos/360) * (5.0/3.0) * (M_PI) * (3.25)));//((avgPos/360) * 10.205);
    printf("newpos: %f, %f\n", newPos, (((newPos/360) * (5.0/3.0) * (M_PI) * (3.25))));
    printf("error: %f\n", error);
    if(error != 0) {

      integral += 1;

    } else {

      integral = 0;

    }

 

    derivative = error-prevError;

    prevError = error;

 

    powers = error * kP + derivative * kD + integral * kI;
    printf("power: %f\n", powers);
 

    spinLeftV(powers);

    spinRightV(powers);
    wait(10, msec);
  }

}

void inertTurnDegPID(double targetValue, double kP)

{

  double turnError = targetValue - inertialSensor.heading();

 

  while(turnError > 1)

  {

    turnError = targetValue - inertialSensor.heading();

 

    spinLeft(turnError * kP);

    spinRight(-turnError * kP);

 

    task::sleep(20); //20 msc

    //wait(20, msec);

  }

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
    //                 ANGLE CORREController1                 //
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

double kp = 0.003;
double ki = 0.0;
double kd = 0.0;
double prevError = 0.0;
double error = 0.0;
double totalError = 0.0; // += error
double der = 0.0; // = error-preverror
double speedAvg = 0;
double Power = 0;
bool ReadyShoot = false;
double maxspeed = 4200; //4200 rpm
double puncherTime = 50;

void FlywheelPID(double targetSpeed, int numDiscs = 1) {
  int shots = 0;
  while(true){
    speedAvg = (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) * 7.0 /2; 
    error = targetSpeed - speedAvg;
    printf("error: %f\n", error);
    printf("speed: %f\n", speedAvg);
    if (fabs(error) <= 100){ //less that 0.5 RPM from target RPM
      ReadyShoot = true;
    }
    else{
      ReadyShoot = false;
    }
    Power = (targetSpeed / 350.0) + (error*kp + totalError * ki + (error - prevError) * kd); ///12 for voltage
    FlywheelUp.spin(forward, Power, volt); //final output in volts
    FlywheelDown.spin(forward, Power, volt);
    prevError = error; //derivative
    totalError += error;
    wait(20, msec);
   if (ReadyShoot) {
     Indexer.set(true);
     wait(300, msec);
     Indexer.set(false);
     shots++;
     printf("shots: %d\n", shots);
     if (shots == numDiscs) {
       return;
     }
   }
   wait(10, msec);
  }
}

void Fly(int targetSpeed = 2250, int Shots = 2) {
  int targetspeed = targetSpeed;
  FlywheelDown.spin(fwd, targetspeed / 7.0, rpm);
  FlywheelUp.spin(fwd, targetspeed / 7.0, rpm);
  int shots = 0;

  int withinSpeed = 0;

  while (shots < Shots){
    if (fabs((FlywheelDown.velocity(rpm) + FlywheelUp.velocity(rpm)) / 2.0 * 7 - targetspeed) < 10){
      withinSpeed++;
      if (withinSpeed >= 20) {
        Indexer.set(true);
        wait(300, msec);
        Indexer.set(false);
        wait(500, msec);
        shots++;

      }
    } else {
      withinSpeed = 0;
    }



  }
  FlywheelDown.stop(coast);
  FlywheelUp.stop(coast);
}



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
  Fly(2250, 2);
  spinLeft(3);
  spinRight(3);
  wait(500, msec);
  inertTurnDegPID(0, 0.25);
  // Intake_Roller_Roller.stop();
  Intake_Roller.spinFor(forward, 0.5, rev);
  stopBase();
 
  

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

  bool xPrev = false;
  bool bPrev = false;
  bool aPrev = false;

  bool l1Prev = false;
  bool flyOn = false;

  bool r1Prev = false;
  bool r2Prev = false;
  bool intRollOn = false;
  int intRollSpeed = 100;

  int flywheelSpeed = 600;

  Puncher.setStopping(hold);

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    // DRIVE
    double initSens = 0.8;
    double sensInc = -0.0001;


    double Axis3Adjusted = Controller1.Axis3.position();
    double Axis1Adjusted = Controller1.Axis1.position() * 0.75;


    double outputL = (Axis3Adjusted + (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    double outputR = (Axis3Adjusted - (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));

    FLDrive.spin(forward, outputL, pct);
    BLDrive.spin(forward, outputL, pct);
    FRDrive.spin(forward, outputR, pct);
    BRDrive.spin(forward, outputR, pct);


    // FLYWHEEL
    if (Controller1.ButtonUp.pressing()) {
      flywheelSpeed = 3000; // 380
    } else if (Controller1.ButtonDown.pressing()) {
      flywheelSpeed = 2500; // 330
    }

    if (Controller1.ButtonL1.pressing() && !l1Prev) {
      flywheelSpeed = 2300; // 290
      if (!flyOn) {
        flyOn = true;
      } else {
        FlywheelDown.stop(coast);
        FlywheelUp.stop(coast);
        flyOn = false;
      }
    }

    if (flyOn) {
      FlywheelDown.spin(forward, (flywheelSpeed / 7.0) / 50, volt);
      FlywheelUp.spin(forward, (flywheelSpeed / 7.0) / 50, volt);
    }



    printf("topM   : %f\n", FlywheelUp.velocity(pct));
    printf("BottomM: %f\n", FlywheelDown.velocity(pct));

    // TRANSMISSION
    if (Controller1.ButtonX.pressing() && !xPrev) {
      trans.set(!trans.value());
    }

    // INTAKE/ROLLER
    if (Controller1.ButtonR1.pressing() && !r1Prev) {
      if (intRollSpeed != -100) {
        intRollOn = true;
        intRollSpeed = -100;
      } else {
        intRollOn = false;
        intRollSpeed = 0;
      }
    }

    if (Controller1.ButtonR2.pressing() && !r2Prev) {
      if (intRollSpeed != 100) {
        intRollOn = true;
        intRollSpeed = 100;
      } else {
        intRollOn = false;
        intRollSpeed = 0;
      }
    } 

    if (intRollOn) {
      Intake_Roller.spin(forward, intRollSpeed, pct);
    } else {
      Intake_Roller.stop(coast);
    }

    //INDEXER
    if(Controller1.ButtonL2.pressing()) {
      Indexer.set(true);
    } else {
      Indexer.set(false);
    }
 

    // PUNCHER 
    if (Controller1.ButtonB.pressing()) {
      Puncher.spinFor((2200 - abs((int)(Puncher.position(degrees)) % 2200)) * -1, rotationUnits::deg, 100, velocityUnits::pct, false);
    }
    if (Controller1.ButtonA.pressing()) {
      Puncher.spin(reverse, 100, pct);
    }
    if (!Controller1.ButtonA.pressing() && aPrev) {
      Puncher.stop();
    }

    xPrev = Controller1.ButtonX.pressing();
    l1Prev = Controller1.ButtonL1.pressing();
    r1Prev = Controller1.ButtonR1.pressing();
    r2Prev = Controller1.ButtonR2.pressing();
    aPrev = Controller1.ButtonA.pressing();


    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
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
