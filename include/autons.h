#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

#define TILE * 24
#define LEFT_TO_CENTER 6.5
#define TOP_TO_CENTER 7.75
#define TILE_EDGE .375

vex::task runWithDelay(int (*callback)(), double timeMsec) {
  wait(timeMsec, msec);
  vex::task runFunction = vex::task(callback);
  return runFunction;
}

int pauseAndTrack() {
  while(true) {
    displayTracking();
    wait(50, msec);
  }
}

void autoInit(double xInit, double yInit, double headingInit, int discInit) {
  globalX = xInit;
  globalY = yInit;
  initHeading = headingInit;
  globalAngle = headingInit;
  inertialSensor.setHeading(headingInit, deg);
  inertialSensor.resetRotation();
  discCount = discInit;
  vex::task track = vex::task(startTracking);
  vex::task startDiscCounting = vex::task(countDiscs);
  vex::task startCata_Intake = vex::task(manageCata_Intake);
}

void testing() {
   autoInit(0, 0, 0, 0);

  vex::task track = vex::task(startTracking);
  //move(forward, 1, 5, State(20, 50, 45, 10));
  pauseAndTrack();
  wait(10, sec);/*
  double steeringAngle = 5.01;
  double powerRight = 100;
  double powerLeft = 100;
  double maxSpeed = 100;
          if (steeringAngle > 0) {
          powerRight = powerLeft  - (trackWidth * fabs(steeringAngle));
          powerRight = fabs(powerRight) < maxSpeed ? powerRight : maxSpeed * sign(powerRight);
        } else if (steeringAngle < 0) {
          powerLeft  = powerRight - (trackWidth * fabs(steeringAngle));
          powerLeft  = fabs(powerLeft) < maxSpeed ? powerLeft : maxSpeed * sign(powerLeft);
        }
  while(true){
      printf("lp: %f, rp: %f\n", powerLeft, powerRight);
  lDrive.spin(forward, powerLeft, pct);
  rDrive.spin(forward, powerRight, pct);
  }*/

  move(forward, 2, 10, State(50, 60, 60, 10, 100), State(90, 10, 180, 20, 30));
  wait(10, sec);

}

void rightSide(bool redAlliance) {
  autoInit(5 TILE + 10.25, 4 TILE - 10.25, 135, 0);
  vex::timer timeout = vex::timer();

  intakeUp();
  move(reverse, 1, 1, 10, State(5 TILE - 6, 4 TILE + 6, 135, 1, 100));

  intakeDown();

  timeout.reset();
  while (discCount < 3 && timeout.time(msec) < 2000) {
    wait(10, msec);
  }

  Turn(290, 100);
  //fireCata(true);
  intakeUp();
  wait(500, msec);

  move(reverse, 1, 5, 10, State(5 TILE + 9, 3 TILE + 22, 300, 10));
  intakeDown();

  wait(500, msec);
  move(forward, 1, 1, 10, State(5 TILE - 6, 4 TILE + 6, 290, 1, 100));
  //fireCata(true);
  wait(500, msec);

  intakeControl(false);
  intake();
  

  moveParallel(reverse, 2, 0, State(5 TILE, 5 TILE - 15, 270, 3), State(6 TILE + 10, 5 TILE - 15, 270, 0));
  
  spinRoller(true, redAlliance);
  intakeControl(true);
  exitMove = true;
  wait(50, msec);
  move(forward, 1, 0, State(5 TILE + 10, 5 TILE - 15, 45, 0));


  removeBands();

  move(reverse, 2, 0, State(4 TILE + 12, 3 TILE + 12, 45, 5), State(4 TILE - 10, 2 TILE + 20, 315, 0));
  pauseAndTrack();
  //fireCata(true);
  wait(300, msec);

  move(reverse, 2, 0, State(4 TILE, 2 TILE + 7, 280, 1, 30), State(5 TILE - 5, 2 TILE + 6, 270, 7, 30));
  move(forward, 1, 10, State(3 TILE + 16, 2 TILE + 16, 315, 0));

  //fireCata(true);
  wait(300, msec);
  
  move(reverse, 2, 0, State(4 TILE - 7, 2 TILE, 350, 1), State(4 TILE - 6, 1 TILE + 5, 0, 7));
  move(forward, 1, 10, State(3 TILE + 16, 2 TILE + 16, 315, 0));

  //fireCata(true);
}


void leftSide(bool redAlliance) {
  autoInit(1 TILE + 18, 15, 180, 0);
  vex::timer timeout = vex::timer();
  
  intakeUp();
  
  move(reverse, 1, 0, State(1 TILE + 18, 1 TILE, 175, 0));
  intakeDown();

  timeout.reset();
  while (discCount < 3 && timeout.time(msec) < 2000) {
    wait(10, msec);
  }

  Turn(355, 100);
  //fireCata(true);
  intakeControl(false);
  intake();
  moveParallel(reverse, 3, 0, State(1 TILE + 12, 1 TILE, 45, 1), State(1 TILE - 4, 1 TILE - 18, 0, 5), State(1 TILE - 4, 0 TILE - 10, 0, 0));
  
  spinRoller(true, redAlliance);
  
  intakeControl(true);
  move(forward, 1, 0, State(1 TILE - 15, 1 TILE - 4,  0, 3));
  //fireCata(true);
  wait(500, msec);


  move(reverse, 3, 0.001, State(2 TILE, 1 TILE + 12, 225, 4), State(2 TILE + 12, 2 TILE + 7, 225, 0.001), State(3 TILE - 2, 2 TILE - 2, 155, 5));
  //fireCata(true);
  wait(500, msec);
  intakeUp();
  Turn(45, 100);
  move(reverse, 1, 0, State(3 TILE - 4, 2 TILE - 4, 45, 0));
  intakeDown();
  Turn(170, 100);

  move(reverse, 1, 3, State(2 TILE + 20, 12, 160, 0));
  move(forward, 1, 0, State(2 TILE + 6, 2 TILE - 6, 170, 10));
  //fireCata(true);







}

void skills() {
  //match load all
  autoInit(3 TILE, 8, 80, 3);

  move(reverse, 3, 0, State(2 TILE + 5, 1 TILE + 5, 225, 5), State(2 TILE + 12, 1 TILE + 12, 225, 5, 50), State(3 TILE + 4, 2 TILE + 4, 124, 0));

  //fireCata();

  Turn(225, 100);

  move(reverse, 2, 0, State(4 TILE + 8, 3 TILE + 8, 180, 0), State(5 TILE + 4, 2 TILE + 15, 320, 0));
  
  Turn(180, 100);
  //fireCata();

  Turn(35, 100);

  move(reverse, 2, 0, State(4 TILE + 18, 2 TILE + 7, 43, 0), State(3 TILE + 20, 2 TILE + 6, 90, 15));




}