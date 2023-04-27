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

vex::timer runTime = vex::timer();
int matchTime = 15;
int timeRun() {
  runTime.reset();
  while (runTime.value() < matchTime) {
    wait(100, msec);
  }

  Controller.rumble("----");
  return 1;
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
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);
  vex::task track = vex::task(startTracking);
  vex::task startDiscCounting = vex::task(countDiscs);
  vex::task startCata_Intake = vex::task(manageCata_Intake);
  vex::task startTimer = vex::task(timeRun);
}
double target;
int printafd() {
  while(true) {
    printf("t: %f, a: %f\n", target, globalAngle);
    wait(10, msec);
  }
}

void testing() {
   autoInit(0, 0, 0, 0);
   
  while (true) {
    if (discCount > 2) {
      wait(1000, msec);
      fireCata(true);
    }
    wait(5000, msec);
  }
  //move(forward, 1, 5, State(20, 50, 45, 10));
  vex::task a = vex::task(printafd);
  target = 45;
  Turn(45, 100);
  wait(2000, msec);
    target = 135;

  Turn(135, 100);
  wait(2000, msec);
    target = 0;

  Turn(0, 100);
  wait(2000, msec);
    target = 3;

  Turn(3, 100);
  wait(2000, msec);
  Turn(180, 100);
    target = 180;



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

  moveParallel(reverse, 1, 1, 10, State(5 T - 6.5, 4 T + 6.5, 135, 1, 100));
  while (Point(globalX, globalY).distTo(Point(5 T - 6.5, 4 T + 6.5)) > 2.5) {
    wait(10, msec);
  }
  intakeDown();

  timeout.reset();
  while (discCount < 3 && timeout.time(msec) < 2500) {
    wait(10, msec);
  }

  exitMove = true;
  wait(50, msec);

  Turn(277, 100, 2);
  removeBands();
  fireCata(true);
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

  

  moveParallel(reverse, 2, 0, 10, State(5 T + 14, 5 T - 11, 275, 3, 100), State(7 T + 10, 5 T - 11, 270, 20, 100));
  //pauseAndTrack();
  spinRoller(false, redAlliance);

  exitMove = true;
  wait(50, msec);

  move(forward, 1, 0, 10, State(5 T + 8, 5 T - 14, 40, 0));




  move(reverse, 3, 0, 2, State(4 T + 12, 3 T + 12, 45, 5, 60), State(3 T + 16, 2 T + 16, 45, 4, 60), State(3 T + 15, 2 T + 16, 307, 0, 60));

  fireCata(true);

  move(reverse, 2, 0, 50, 2800, State(4 T - 2, 2 T + 11, 270, 0, 30), State(6 T + 10, 2 T + 8, 280, 10, 40));
 
  move(forward, 1, 0, 1, State(3 T + 10, 2 T + 14, 310, 0, 100));



  fireCata(true);
  /*
  move(reverse, 2, 0, 50, 2000, State(4 T - 10, 2 T - 3, 0, 1, 30), State(4 T - 5, 1 T - 10, 350, 7, 55));
  move(forward, 1, 0, 3, State(3 T + 8, 2 T + 13, 314, 0, 100));

  fireCata(true);*/
  pauseAndTrack();
}


void leftSide(bool redAlliance) {
  autoInit(2 T - 5, 1 T - (TILE_EDGE + 8), 159.7, 0);
  vex::timer timeout = vex::timer();
  intakeUp();
  move(reverse, 1, 1, 1, State(39, 27.792, 159.7, 1));
  intakeDown();
  timeout.reset();
  while (discCount < 3 && timeout.time() < 3000) {
    wait(10, msec);
  }
  Turn(348, 100);
  fireCata(true);
  wait(700, msec);
  move(reverse, 1, 0, 2, State(1 T + 6, 1 T + 3, 110, 3, 100));

  //lDrive.spin(forward, 10, pct);
  /*lDrive.spin(reverse, 10, pct);
  rDrive.spinFor(forward, 3, rev, 50, velocityUnits::pct);
  lDrive.stop(hold);*/
  move(forward, 1, 1, 5, State(1 T + 9, 22, 0, 1));
  
  move(reverse, 1, 0, 10, 2000, State(1 T + 11, 12, 0, 3, 100));
  wait(500, msec);
  moveParallel(reverse, 1, 3, State(1 T + 11, -2 T, 0, 3, 70));
  spinRoller(true, redAlliance);
  exitMove = true;
  wait(50, msec);
  move(forward, 1, 3, 2, State(1 T + 18, 1 T + 4, 342, 0));
  discCount > 2 ? fireCata(true) : fireCata();
  intakeUp();
  Turn(250, 100, 5);
  move(reverse, 1, 3, 2, State(2 T + 6, 1 T + 8, 245, 0));
  intakeDown();

  timeout.reset();
  while (discCount < 3 && timeout.time() < 3000) {
    wait(10, msec);
  }
  //move(forward, 1, 0, 2, State(2 T + 8, 1 T + 16, 334, 5));
  Turn(334, 100, 1);
  //removeBands();
  discCount > 2 ? fireCata(true) : fireCata();



  
 


}



void winPoint(bool redAlliance) {
  
  autoInit(5 T + 6.850394, 4 T - 6.968504, 180, 2);

  moveParallel(reverse, 2, 2, State(5 T + 20, 4 T + 4, 270, 5, 50), State(7 T, 4 T + 4, 270, 10, 50));
  spinRoller(false, redAlliance);
  exitMove = true;
  wait(50, msec);

  move(forward, 1, 7, 3, State(5 T + 4, 4 T + 2, 276, 0));

  //removeBands();

  fireCata(true);
  
  

  Turn(20, 100, 5);
  move(reverse, 2, 0, 50, State(4 T + 12, 3 T + 12, 45, 0, 35), State(3 T + 13, 2 T + 13, 50, 0, 50));
  //move(forward, 1, 0, State(3 T + 9, 2 T + 15, 315, 0, 40));
  Turn(310, 200, 2);

  fireCata(true);
  wait(200, msec);
  move(reverse, 2, 0, 30, 2500, State(4 T - 2, 2 T + 10, 270, 1, 20), State(5 T + 5, 2 T + 5, 270, 5, 40));

  move(forward, 1, 3, 1, State(3 T + 8, 2 T + 7, 313, 0, 100));

  wait(300, msec);
  fireCata(true);

  /*
  move(reverse, 2, 0, 30, State(4 T - 10, 2 T, 0, 0, 25), State(4 T - 7, 1 T, 0, 10, 40));
  move(forward, 1, 3, State(3 T + 9, 2 T + 15, 315, 13, 100));
  fireCata(true);*/

  moveParallel(reverse, 3, 0, State(2 T + 18, 1 T + 16, 45, 5, 80), State(2 T - 4, 1 T, 90, 0, 40), State(2 T - 12, -2 T, 0, 10, 50));
  spinRoller(true, redAlliance, 5000);
  exitMove = true;
  wait(50, msec);
  move(forward, 1, 0, State(2 T + 2, 1 T, 350, 0));

  fireCata();
// removeBands();

  
  wait(10000, sec);

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