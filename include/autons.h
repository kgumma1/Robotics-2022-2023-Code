#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

#define T * 24
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

  //move(forward, 1, 5, State(20, 50, 45, 10));

  Turn(45, 100);
  wait(2000, msec);
  Turn(135, 100);
  wait(2000, msec);
  Turn(0, 100);
  wait(2000, msec);
  Turn(3, 100);
  wait(2000, msec);
  Turn(180, 100);


  pauseAndTrack();
  while(true){
    printf("discCount: %d, ref: %ld\n", discCount, bottomIntakeSensor.reflectivity());
    wait(10, msec);
    if(discCount >= 3) {
      wait(5, sec);
      fireCata();
    }
  }


  
  wait(1000000000, sec);/*
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
  autoInit(5 T + 8.75, 4 T - 8.75, 135, 0);


  vex::timer timeout = vex::timer();

  intakeUp();
  move(reverse, 1, 1, 10, State(5 T - 6, 4 T + 6, 135, 1, 100));

  intakeDown();

  timeout.reset();
  while (discCount < 3 /*&& timeout.time(msec) < 3000*/) {
    wait(10, msec);
  }

  Turn(283, 100);

  fireCata(true);
  wait(300, msec);
  /*
  intakeUp();
  wait(500, msec);

  move(reverse, 1, 5, 10, State(5 T + 9, 3 T + 22, 300, 10));
  intakeDown();
  while (discCount < 2 && timeout.time(msec) < 3000) {
    wait(10, msec);
  }
  
  move(forward, 1, 1, 10, State(5 T - 6, 4 T + 6, 290, 1, 100));
  fireCata(true);
  wait(500, msec);
  */
  intakeControl(false);
  intake();
  

  moveParallel(reverse, 2, 0, State(5 T + 14, 5 T - 16, 275, 3, 50), State(6 T + 10, 5 T - 16, 270, 20, 50));
  //pauseAndTrack();
  spinRoller(true, redAlliance);
  intakeControl(true);
  exitMove = true;
  wait(50, msec);
  move(forward, 1, 0, State(5 T + 14, 5 T - 14, 45, 0));


  removeBands();

  move(reverse, 3, 0, 30, State(4 T + 12, 3 T + 12, 45, 5, 50), State(3 T + 20, 2 T + 20, 45, 4, 50), State(3 T + 6, 2 T + 18, 315, 0, 50));

  while (discCount < 3 /*&& timeout.time(msec) < 3000*/) {
    wait(10, msec);
  }
  fireCata(true);
  wait(300, msec);

  move(reverse, 2, 0, State(4 T, 2 T + 8, 280, 0, 40), State(5 T - 5, 2 T + 6, 270, 10, 40));
  move(forward, 1, 10, State(3 T + 8, 2 T + 16, 315, 0, 70));


  while (discCount < 3 /*&& timeout.time(msec) < 3000*/) {
    wait(10, msec);
  }
  fireCata(true);
  wait(300, msec);
  
  move(reverse, 2, 0, State(4 T - 10, 2 T - 3, 350, 1, 40), State(4 T - 6, 1 T + 5, 0, 7, 40));
  move(forward, 1, 10, State(3 T + 8, 2 T + 16, 315, 0));
  while (discCount < 3 /*&& timeout.time(msec) < 3000*/) {
    wait(10, msec);
  }
  fireCata(true);
}


void leftSide(bool redAlliance) {
  autoInit(1 T + 18, 15, 180, 0);
  vex::timer timeout = vex::timer();
  
  intakeUp();
  
  move(reverse, 1, 0, State(1 T + 18, 1 T, 175, 0));
  intakeDown();

  timeout.reset();
  while (discCount < 3 && timeout.time(msec) < 2000) {
    wait(10, msec);
  }

  Turn(355, 100);
  //fireCata(true);
  intakeControl(false);
  intake();
  moveParallel(reverse, 3, 0, State(1 T + 12, 1 T, 45, 1), State(1 T - 4, 1 T - 18, 0, 5), State(1 T - 4, 0 T - 10, 0, 0));
  
  spinRoller(true, redAlliance);
  
  intakeControl(true);
  move(forward, 1, 0, State(1 T - 15, 1 T - 4,  0, 3));
  //fireCata(true);
  wait(500, msec);


  move(reverse, 3, 0.001, State(2 T, 1 T + 12, 225, 4), State(2 T + 12, 2 T + 7, 225, 0.001), State(3 T - 2, 2 T - 2, 155, 5));
  //fireCata(true);
  wait(500, msec);
  intakeUp();
  Turn(45, 100);
  move(reverse, 1, 0, State(3 T - 4, 2 T - 4, 45, 0));
  intakeDown();
  Turn(170, 100);

  move(reverse, 1, 3, State(2 T + 20, 12, 160, 0));
  move(forward, 1, 0, State(2 T + 6, 2 T - 6, 170, 10));
  //fireCata(true);







}

void winPoint(bool redAlliance) {
  
}

void skills() {
  //match load all
  autoInit(3 T, 8, 80, 3);

  // close 3 stack
  move(reverse, 3, 0, State(2 T + 5, 1 T + 5, 225, 5), State(2 T + 12, 1 T + 12, 225, 5, 50), State(3 T + 4, 2 T + 4, 124, 0));

  fireCata();


  // close line
  Turn(225, 100);

  move(reverse, 2, 0, State(4 T + 8, 3 T + 8, 180, 0), State(5 T + 4, 2 T + 15, 320, 0));
  
  Turn(180, 100);
  fireCata();

  // low goal dis (horizontal)
  Turn(35, 100);

  move(reverse, 2, 0, State(4 T + 18, 2 T + 7, 43, 0), State(3 T + 12, 2 T + 6, 90, 15));

  Turn(130, 100);
  fireCata();

  // low goal discs (vertical)
  Turn(340, 100);

  move(reverse, 2, 0, State(3 T + 12, 2 T, 340, 0), State(3 T + 17, 12, 78, 0));

  fireCata();

  // close auto line 3 stack + roller

  moveParallel(reverse, 4, 10, State(1 T + 12, 1 T + 12, 114, 10), State(1 T + 4, 1 T + 20, 135, 0), State(14, 1 T + 13, 90, 10), State(-2 T, 1 T + 13, 90, 10));

  spinRoller(false);

  exitMove = true;
  wait(50, msec);

  //other roller
  move(forward, 1, 4, State(1 T, 1 T, 105, 2));
  moveParallel(reverse, 2, 0, State(1 T, 7, 0, 5), State(1 T, -2 T, 0, 4));

  spinRoller(false);

  exitMove = true;
  wait(50, msec);

  // shoot 3 stack
  move(forward, 1, 10, State(8, 3 T + 8, 359, 5));

  fireCata();


  // horizontal low goal discs (far)
  Turn(215, 100);

  move(reverse, 2, 2, State(1 T + 3, 3 T + 14, 217, 0), State(2 T, 3 T + 16, 270, 0));
  move(forward, 1, 5, State(1 T + 4, 3 T + 7, 350, 5));


  // get far line

  move(reverse, 2, 0, State(1 T + 10, 2 T + 19, 347, 3), State(2 T + 12, 3 T + 12, 225, 10));

  Turn(315, 100);

  fireCata();


  // vertical low goal (far)

  Turn(166, 100);

  move(reverse, 2, 0, State(2 T + 8, 4 T, 165, 0), State(2 T + 6, 5 T + 12, 260, 0));

  fireCata();

  // match load

  move(reverse, 1, 0, State(3 T, 5 T + 14, 260, 5));


  // far 3 stack (not on auto line)

  move(reverse, 1, 5, State(3 T + 12, 4 T + 12, 350, 6));

  move(forward, 1, 0, State(3 T, 4 T + 18, 281, 10));

  fireCata();

  // auto line far stack + roller
  moveParallel(reverse, 2, 0, State(4 T + 12, 4 T + 12, 270, 10), State(7 T + 12, 4 T + 12, 270, 10));
  spinRoller(false);

  exitMove = true;

  // shoot discs
  move(forward, 1, 10, State(4 T + 12, 3 T + 16, 166, 10));

  fireCata(true);


  // roller

  moveParallel(reverse, 2, 0, State(5 T, 5 T, 180, 10), State(5 T, 7 T, 180, 10));
  spinRoller(false);

  exitMove = true;

  // expands
  move(forward, 1, 3, State(5 T, 5 T, 225, 0));
  expansionLeft.set(true);
  expansionRight.set(true);




  

  




}