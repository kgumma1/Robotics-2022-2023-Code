#include "vex.h"

using namespace vex;

int drivePID(double dist = 5, double maxSpeed = 10, double timeOut = 10) { // speed in volts?!?!?!?!?
  double error[2];
  double prevError[2];
  double powers[2];
  double kP = 0.027;
  double kI = 0.0004;
  double kD = 0.1;

  double PI = 3.14159265358979;

  double integral[2];
  double derivative[2];

  rightEncoder.resetRotation();
  leftEncoder.resetRotation();
  prevError[0] = ((dist / (3.25 * PI)) * 360);
  prevError[1] = ((dist / (3.25 * PI)) * 360);

  integral[0] = 0;
  integral[1] = 0;

  Brain.Screen.print("1");
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

    rDrive.spin(fwd, powers[0], volt);
    lDrive.spin(fwd, powers[1], volt);

    //printf("left = %f, right = %f, errorL = %f, errorR = %f\n", powers[1], powers[0], error[1], error[0]);
    //printf("prop = %f, int = %f, dev = %f, errorR = %f, errorL = %f, pow = %f\n", kP * error[0], integral[0], derivative[0], error[0], error[1], powers[1]);
    wait(10, msec);
    printf("errorRight = %f, errorLeft = %f\n", error[0], error[1]);

  } while ((fabs(error[0]) > 5 && fabs(error[1]) > 5) && exitTimer.value() < timeOut);
  rDrive.stop(hold);
  lDrive.stop(hold);
  return 1;
}


double getInertialReading()
{
  // COMMENTED OUT BECAUSE WE DON"T HAVE AN INERTIAL SENSOR YET 
  double reading = inertialSensor.heading(degrees);
  reading = reading > 180 ? reading - 360 : reading;
  reading = reading < -180 ? reading + 360 : reading;
  return reading;
  return 0;
}

vex::timer Timer3 = vex::timer();

void simple(float leftSpeed, float rightSpeed, float timeSec) {
  RFDrive.spin(forward, rightSpeed, pct);
  RMDrive.spin(forward, rightSpeed, pct);
  RBDrive.spin(forward, rightSpeed, pct);

  LFDrive.spin(forward, leftSpeed, pct);
  LMDrive.spin(forward, leftSpeed, pct);
  LBDrive.spin(forward, leftSpeed, pct);
  
  wait(timeSec, sec);

  RFDrive.stop();
  RMDrive.stop();
  RBDrive.stop();

  LFDrive.stop();
  LMDrive.stop();
  LBDrive.stop();

}

void Turn(double destination, double speed, double timeOut=5) {
  double curAngle, turnAmount, startAngle, output, speedR, speedL, absOutput, passed = 1000, PIDspeed, profile, curAngle2, tAngle, curAngleSgn, prevAngleSgn;
  double error, prevError, derivError, intError = 0;
  bool doProfile = true, check = true;
  double incFrac = 0.3;
  double decFrac = 0.3;
  double Kd = 2.5; //0.01
  double Ki = 0.11; //0.0
  double Kp = 0.3; //0.67, recent 0.8
  double speedConst = 8;
  double threshold = 0.2;

  double loggedTime = 1000;
  double bounceTime = 0.7;

  bool triggered = false;

  Timer3.clear();
  double curTime = Timer3.time(vex::timeUnits::sec);
  vex::timer exitTimer = vex::timer();

  startAngle = getInertialReading();
  turnAmount = (destination - startAngle);
  turnAmount = turnAmount > 180 ? turnAmount - 360 : turnAmount;
  turnAmount = turnAmount < -180 ? turnAmount + 360 : turnAmount;

  double prevAngle = startAngle;

  leftEncoder.resetRotation();
  rightEncoder.resetRotation();

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

    rDrive.spin(forward, speedR, volt);
    lDrive.spin(forward, speedL, volt);
    wait(10, msec);
    //printf("turnamount=%f curAngle=%f error=%f output=%f int=%f deriv=%f PIDcalculated=%f \n\n",turnAmount, curAngle, error, output, intError, derivError, PIDspeed);

    
    if(((curAngle>turnAmount-threshold && turnAmount>0) || (curAngle<turnAmount+threshold && turnAmount<0)) && !triggered)
    {
      loggedTime = curTime;
      triggered = true;
    }

    //printf("loggedTime = %f, loggedTime+bounceTime = %f, curTime = %f\n", loggedTime, loggedTime+bounceTime, curTime);
    printf("error = %f\n", error);
  }while(fabs(error) > 0.5 && exitTimer.value() < timeOut/*true*/);
  //((curAngle<turnAmount-threshold && turnAmount>0) || (curAngle>turnAmount+threshold && turnAmount<0)) && timeOut > curTime

  rDrive.stop(hold);
  lDrive.stop(hold);
}

int discCount = 0;
double targetSpeed = 0;
double shotD = 0.2;
double maxShotT = 60;
int discsShotCount = 0;


void resetDiscCount() {
  discCount = 0;
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

int discsIntaked;

int trackDiscsShot() {
  while (true) {
    while (flywheelSensor.reflectivity() - 4 <= flywheelSensorInit) {
      wait(5, msec);
      //printf("flywheelSensor = %ld\n", flywheelSensor.reflectivity());
    }
    while (flywheelSensor.reflectivity() - 4 > flywheelSensorInit) {
      wait(5, msec);
      //printf("flywheelSensor = %ld\n", flywheelSensor.reflectivity());
    }
    discCount = discCount > 0 ? discCount-1 : 0;
    discsIntaked = discsIntaked > 0 ? discsIntaked-1 : 0;

    wait(5, msec);
  }
  return 1;
}



int maintain3Discs() {
  while (true) {
    while (topIntakeSensor.reflectivity() - 6 <= topIntakeSensorInit) {
      if (discsIntaked >= 3 && discCount <= 0 && bottomIntakeSensor.reflectivity() - 4 > bottomIntakeSensorInit) {
        intake_roller.spin(reverse, 100, pct);
      }
      if (discsIntaked < 3 && discCount <= 0) {
        intake_roller.spin(forward, 100, pct);
      }
      wait(5, msec);
      printf("discCount = %d\n", discCount);
      printf("discsIntaked = %d\n", discsIntaked);
    }
    printf("discCount = %d\n", discCount);
    printf("discsIntaked = %d\n", discsIntaked);
    while (topIntakeSensor.reflectivity() - 6 > topIntakeSensorInit) {
      if (discsIntaked >= 3 && discCount <= 0 && bottomIntakeSensor.reflectivity() - 4 > bottomIntakeSensorInit) {
        intake_roller.spin(reverse, 100, pct);
      }
      if (discsIntaked < 3 && discCount <= 0) {
        intake_roller.spin(forward, 100, pct);
      }
      wait(5, msec);
      printf("discCount = %d\n", discCount);
      printf("discsIntaked = %d\n", discsIntaked);
    }
          printf("discCount = %d\n", discCount);
      printf("discsIntaked = %d\n", discsIntaked);
    discsIntaked++;
  }
}

int flywheelPID() {


  double integral = 0, derivative;

  double error = targetSpeed;
  double prevError = targetSpeed;
  double output = 12;
  double outputChange = 0;
  vex::timer shotTimer = vex::timer();
  int errorCount = 0;

  vex::task startShotCount = vex::task(trackDiscsShot);
  do {
    error = targetSpeed - (flywheel.velocity(rpm) * 6);
    if (targetSpeed == 0) {
      flywheel.stop(coast);
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
    flywheel.spin(fwd, output, volt);

    if (fabs(error) < 50) {
      errorCount++;
    } else {
      errorCount = 0;
    }

    if (((errorCount >= 2 && shotTimer.value() > shotD) || shotTimer.value() > maxShotT) && discCount > 0) {
      intake_roller.startRotateFor(reverse, 500, deg, 100, velocityUnits::pct);
      shotTimer.reset();
      errorCount = 0;
    } /*
    if (shotTimer.value() > shotD) {
      intake_roller.stop(coast);
      shotTimer.reset();
    }*/
    if (discCount > 0) {
      basket.set(true);
    } else {
      basket.set(false);
    }
    //printf("discs = %d\n", discCount);
    


    //printf("error=%f outputChange=%f output=%f speed=%f rotSpeed=%f deriv=%f\n", error, outputChange, output, (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2 * 7, (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2 * 7 - rotSensor.velocity(rpm), derivative);
    //printf("error = %f, outputChange = %f, output = %f\n", error, outputChange, output);
    printf("discs = %d\n", discsIntaked);
    wait(10, msec);
  } while (true);
  return 1;
}