#include "vex.h"

using namespace vex;
vex::timer Timer3 = vex::timer();

// get inertial reading
double getInertialReading()
{
  // COMMENTED OUT BECAUSE WE DON"T HAVE AN INERTIAL SENSOR YET 
  // double reading = inertialSensor.heading(degrees);
  // reading = reading > 180 ? reading - 360 : reading;
  // reading = reading < -180 ? reading + 360 : reading;
  // return reading;
  return 0;
}

void calibrateIntertial()
{
  // COMMENTED OUT BECAUSE WE DON"T HAVE AN INERTIAL SENSOR YET 
  // wait(3, sec);
  // inertialSensor.calibrate();
  // Brain.Screen.clearScreen();
  // Brain.Screen.print("...");
  // ct.Screen.print("...");
  // while (inertialSensor.isCalibrating())
  // {
  //   vex::task::sleep(20);
  // }

  // ct.Screen.print("Done");
  // Brain.Screen.print("Done");
}

// reset trackers
void resetTrackers()
{
  FRDrive.resetPosition();
  BRDrive.resetPosition();
  FLDrive.resetPosition();
  BLDrive.resetPosition();
  //TrackerS.setRotation(0, degrees);
}


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
