#include "vex.h"
#include "tracking.h"

using namespace vex;

double getInertialReading()
{
  // COMMENTED OUT BECAUSE WE DON"T HAVE AN INERTIAL SENSOR YET 
  double reading = inertialSensor.heading(degrees);
  reading = reading > 180 ? reading - 360 : reading;
  reading = reading < -180 ? reading + 360 : reading;
  return reading;
  return 0;
}

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

void spinLeftBack(double speed)
{
  BLDrive.spin(reverse, speed , velocityUnits::pct);
  FLDrive.spin(reverse, speed , velocityUnits::pct); 
}

void spinRightBack(double speed)
{
  BRDrive.spin(reverse, speed , velocityUnits::pct);
  FRDrive.spin(reverse, speed , velocityUnits::pct);
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
void stopBase(brakeType b = coast)
{
  FLDrive.stop(b);
  BLDrive.stop(b);
  FRDrive.stop(b);
  BRDrive.stop(b);
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
  double error, prevError, derivError, intError = 0;
  bool doProfile = true, check = true;
  double incFrac = 0.3;
  double decFrac = 0.3;
  double Kd = 1; //0.01
  double Ki = 0.015; //0.0
  double Kp = 0.15; //0.67, recent 0.8
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
    if (fabs(Kd * derivError) < 0.2) {
      intError = error + intError;
    }
    if (fabs(Kd * derivError) > 0.5) {
      intError = 0;
    }

    PIDspeed = Kp*error + Kd*derivError + Ki*intError;

    prevAngle = curAngle;


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
    if (fabs(output) > speed) {
      output = output > 0 ? speed : -speed;
    }

    speedR = -output;
    speedL = output;

    FRDrive.spin(fwd, speedR, volt);
    BRDrive.spin(fwd, speedR, volt);
    FLDrive.spin(fwd, speedL, volt);
    BLDrive.spin(fwd, speedL, volt);
    wait(10, msec);
    //printf("turnamount=%f curAngle=%f error=%f output=%f int=%f deriv=%f PIDcalculated=%f \n\n",turnAmount, curAngle, error, output, intError, derivError, PIDspeed);

    
    if(((curAngle>turnAmount-threshold && turnAmount>0) || (curAngle<turnAmount+threshold && turnAmount<0)) && !triggered)
    {
      loggedTime = curTime;
      triggered = true;
    }

    //printf("loggedTime = %f, loggedTime+bounceTime = %f, curTime = %f\n", loggedTime, loggedTime+bounceTime, curTime);

  }while(fabs(error) > 2/*curTime < bounceTime+loggedTime && curTime < timeOut*/);
  //((curAngle<turnAmount-threshold && turnAmount>0) || (curAngle>turnAmount+threshold && turnAmount<0)) && timeOut > curTime

  stopBase(hold);
}

// --------------------------------------------- //
//                   DRIVE
// --------------------------------------------- //
vex::timer Timer1 = vex::timer();



void driveFwdPID(double dist, bool forwards = true, double kP = 4.5) {

 //0.185

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
 

    spinLeftV(powers * (forwards ? 1 : -1));

    spinRightV(powers * (forwards ? 1 : -1));
    wait(10, msec);
  }

}

int drivePID(double dist = 5, double maxSpeed = 12, double timeOut = 10) {
  double error[2];
  double prevError[2];
  double powers[2];
  double kP = 0.023;
  double kI = 0.001;
  double kD = 0.12;

  double integral[2];
  double derivative[2];

  rightEncoder.resetRotation();
  leftEncoder.resetRotation();
  prevError[0] = ((dist / (3.25 * PI)) * 360);
  prevError[1] = ((dist / (3.25 * PI)) * 360);

  integral[0] = 0;
  integral[1] = 0;

  vex::timer exitTimer = vex::timer();
    exitTimer.reset();
  do {

    error[0] = ((dist / (3.25 * PI)) * 360) - rightEncoder.position(deg);
    error[1] = ((dist / (3.25 * PI)) * 360) - leftEncoder.position(deg);


    if (fabs(derivative[0] * kD) < 0.2) {
      integral[0] = integral[0] + error[0];
    }
    if (fabs(derivative[1] * kD) < 0.2) {
      integral[1] = integral[1] + error[1];
    }
    if (fabs(derivative[0] * kD) > 0.5) {
      integral[0] = 0;
    }
    if (fabs(derivative[1] * kD) > 0.5) {
      integral[1] = 0;
    }
    

    derivative[0] = error[0] - prevError[0];
    derivative[1] = error[1] - prevError[1];

    prevError[0] = error[0];
    prevError[1] = error[1];


    powers[0] = kP * error[0] + kI * integral[0] + kD * derivative[0];
    powers[1] = kP * error[1] + kI * integral[1] + kD * derivative[1];

    //printf("______________________________pRight=%f pLeft=%f\n", powers[0], powers[1]);

    if (fabs(powers[0]) > maxSpeed) {
      if (powers[0] > 0) {
        powers[0] = maxSpeed;
      } else {
        powers[0] = -maxSpeed;
      }
    }
    if (fabs(powers[1]) > maxSpeed) {
      if (powers[1] > 0) {
        powers[1] = maxSpeed;
      } else {
        powers[1] = -maxSpeed;
      }
    }

    //printf("pRight=%f pLeft=%f____________________________\n", powers[0], powers[1]);

    FRDrive.spin(fwd, powers[0], volt);
    BRDrive.spin(fwd, powers[0], volt);
    FLDrive.spin(fwd, powers[1], volt);
    BLDrive.spin(fwd, powers[1], volt);

    //printf("left = %f, right = %f, errorL = %f, errorR = %f\n", powers[1], powers[0], error[1], error[0]);
    //printf("prop = %f, int = %f, dev = %f, errorR = %f, errorL = %f, pow = %f\n", kP * error[0], integral[0], derivative[0], error[0], error[1], powers[1]);
    wait(10, msec);

  } while ((fabs(error[0]) > 10 && fabs(error[1]) > 10) && exitTimer.value() < timeOut);
  stopBase(hold);
  return 1;
}

void inertTurnDegPID(double targetValue, double kP, bool clockwise = true)

{

  double turnError =  targetValue - inertialSensor.heading();

 

  while(clockwise ? turnError > 3 : turnError < 3)

  {

    turnError = targetValue - inertialSensor.heading();

 

    spinLeft(turnError * kP * (clockwise ? 1 : 1));

    spinRight(-turnError * kP * (clockwise ? 1 : 1));

 

    task::sleep(20); //20 msc

    //wait(20, msec);

  }
  stopBase();

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

int discCount = 0;
double targetSpeed = 0;
double shotD = 0.2;
double maxShotT = 60;


void resetDiscCount() {
  discCount = 0;
  targetSpeed = 0;
  shotD = 0.2;
}

int numQueued() {
  return discCount;
}

int queueDiscs(int n, double target = -1, double shotDelay = -1, double maxShotTime = -1) {
  discCount += n;
  targetSpeed = target > 0 ? target : targetSpeed;
  shotD = shotDelay > 0 ? shotDelay : shotD;
  maxShotT = maxShotTime > 0 ? maxShotTime : maxShotT;
  return discCount;
}

double kP = 0.00012;
double kP2 = 0.0001;
double kPEqui = 0.00003;
double kI = 0;
double kD = 0.005;
double kDEqui = 0;

void adjustFPID(double newkP = 0.00012, double newkP2 = 0.0001, double newkI = 0, double newkD = 0.005, double newkPEqui = 0.00003, double newkDEqui = 0) {
  kP = newkP;
  kP2 = newkP2;
  kI = newkI;
  kD = newkD;
  kPEqui = newkPEqui;
  kDEqui = newkDEqui;
}

int flywheelPID() {


  double integral = 0, derivative;

  double error = targetSpeed;
  double prevError = targetSpeed;
  double output = 12;
  double outputChange = 0;
  vex::timer shotTimer = vex::timer();
  int errorCount = 0;
  do {
    error = targetSpeed - ((FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2 * 7);
    if (targetSpeed == 0) {
      FlywheelUp.stop(coast);
      FlywheelDown.stop(coast);
      continue;
    }
    //integral = integral + error;
    
    derivative = error - prevError;
 
    prevError = error;

    if (fabs(error) > 200) {
      outputChange = error * (error > 0 ? kP : kP2) + integral * kI + derivative * kD;  

    } else {
      outputChange = error * kPEqui + integral * kI + derivative * kDEqui; 
 
    }

  
    output = output + outputChange >= 12 ? 12 : output + outputChange;
    //output = 7.2;
    FlywheelUp.spin(fwd, output, volt);
    FlywheelDown.spin(fwd, output, volt);

    if (fabs(error) < 50) {
      errorCount++;
    } else {
      errorCount = 0;
    }
    if (/*fabs(derivative) < 20 &&*/ Indexer.value() == false && ((errorCount >= 2 && shotTimer.value() > shotD) || shotTimer.value() > maxShotT) && discCount > 0) {
      Indexer.set(true);
      discCount--;
      errorCount = 0;
      shotTimer.reset();
      printf("---------------shoots----------------%llu\n", 111111111111111111);
    }
    if (Indexer.value() == true && shotTimer.value() > shotD) {
      Indexer.set(false);
      shotTimer.reset();
    }
    //rintf("error=%f outputChange=%f output=%f speed=%f rotSpeed=%f deriv=%f\n", error, outputChange, output, (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2 * 7, (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2 * 7 - rotSensor.velocity(rpm), derivative);
    wait(10, msec);
  } while (true);
  return 1;
}


void Fly(int targetSpeed = 2250, int motorSpeed = 210, int Shots = 2, double waitTime = 0.2) {
  /*int targetspeed = targetSpeed;
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
  FlywheelUp.stop(coast);*/
  int shots = 0;
  vex::timer fly = vex::timer();


  while (shots < Shots || Indexer.value()) {
    double avgSpeed = (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2;
    if (motorSpeed - avgSpeed > 20 || (fly.value() > 0.05 && fly.value() < waitTime && fly.system() > waitTime)) {
      FlywheelDown.spin(forward, 11, volt);
      FlywheelUp.spin(forward, 11, volt);
    } else {
      FlywheelDown.spin(forward, (targetSpeed / 7.0) / 50, volt);
      FlywheelUp.spin(forward, (targetSpeed / 7.0) / 50, volt);
    }
    
    printf("t = %f : s = %f : ind = %d\n", fly.value(), motorSpeed - avgSpeed, Indexer.value() ? 1 : 0);
    if (fly.value() > waitTime && Indexer.value()) {
      Indexer.set(false);
      fly.reset();
    } else if (fly.value() > waitTime && motorSpeed - avgSpeed < 10 && !Indexer.value()) {
      Indexer.set(true);
      shots++;
      fly.reset();
    }
    wait(10, msec);
  }
  wait(waitTime, sec);
  Indexer.set(false);

}

int intake1() {
  Intake_Roller.spin(reverse, 100, pct); 

  while (IntakeSensor.reflectivity() < intakeSensorInit + 4) {
    wait(5, msec);
  }
  wait(50, msec);
  Intake_Roller.stop(coast);
  return 1;
}
int count = 0;
int outtakeSpeed = 100;
void changeCount(int n) {
  count = n >= 0 ? n : 0;
}
void changeOuttake(int n) {
  outtakeSpeed = n;
}
int shootDisc() {
  Indexer.set(true);
  wait(300, msec);
  Indexer.set(false);
  wait(300, msec);
  count--;
  return 1;
}

vex::task shoot = vex::task(shootDisc);

int countDiscs() {
  while(true){
    while (IntakeSensor.reflectivity() < intakeSensorInit + 4) {
      wait(10, msec);
    }
   
    while (IntakeSensor.reflectivity() >= intakeSensorInit + 4) {
      wait(10, msec);
    }
    count++;
    /*if (count > 3) {
      shoot.resume();
    }*/
    }
}

int maintain3() {
  while (true) {
    if (count >= 3 && BottomIntakeSensor.reflectivity() >= bottomIntakeSensorInit + 4) {
      Intake_Roller.spin(forward, 100, pct);
    } else if (count <= 3) {
      Intake_Roller.spin(reverse, 100, pct);
    } else {
      Intake_Roller.stop(coast);
    }
    //printf("count = %d\n", count);
    wait(10, msec);

  }
}

int intake3() {
  Intake_Roller.spin(reverse, 100, pct); 
  count = 0;
  while (count < 3) {
    while (IntakeSensor.reflectivity() < intakeSensorInit + 4) {
      wait(5, msec);
    }
    while (IntakeSensor.reflectivity() >= intakeSensorInit + 4) {
      wait(5, msec);
    }
    count++;
    /*if (count > 3) {
      shoot.resume();
    }*/
  }
  wait(200, msec);
  Intake_Roller.spin(forward, outtakeSpeed, pct);
  

  return 1;
}