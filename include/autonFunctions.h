#include "vex.h"
#include "tracking.h"
#include "bezier.h"

using namespace vex;



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