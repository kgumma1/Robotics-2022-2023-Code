#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

#define TILE * 24
#define LEFT_TO_CENTER 6.5
#define TOP_TO_CENTER 8.25
#define TILE_EDGE .375

Point findControlPoint(Point anchor, double angle, double adherence) {
  angle = angle * M_PI / 180;
  return Point(adherence * cos(-angle + M_PI / 2) + anchor.x, adherence * sin(-angle + M_PI / 2) + anchor.y);
}

vex::task runWithDelay(int (*callback)(), double timeMsec) {
  wait(timeMsec, msec);
  vex::task runFunction = vex::task(callback);
  return runFunction;
}

void pauseAndTrack() {
  while(true) {
    displayTracking();
    wait(50, msec);
  }
}

void testing() {
  resetDiscCount();
  volleySpeed = 30;
  vex::task flywheelOn1 = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3320, 0.5, -1, false);
  //flywheel.spin(forward, 12, volt);
  discsIntaked = 3;
  vex::task runIntake = vex::task(maintain3Discs);
  wait(3, sec);
  queueDiscs(3, 3320, 0.5, 0.5, true, 500);




  wait(1000, sec);

  globalX = 1 TILE + 4 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER + 5.25);
  vex::task track2 = vex::task(startTracking);

  moveParallel(forward, 2, 0.001, 2, State(1 TILE, 1 TILE, 315, 0.001, 50), State(17, 3 TILE + 4, 3, 2));

  while (true) {
    printf("WORKING %d\n", 1);
    wait(50, msec);
  }
  wait(1, sec);
  Turn(180, 100);
  wait(1, sec);
  Turn(90, 100);
  wait(1, sec);
  Turn(45, 100);
  wait(1, sec);
  Turn(44, 100);
  wait(1, sec);
  Turn(0, 100);
  wait(1000, sec);
  intake_roller.spin(forward, 100, pct);

  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 2900, 0.5);

  wait(10000, sec);


  globalX = 3 TILE + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER);
  vex::task track = vex::task(startTracking);
  leftRollerSensor.setLightPower(100);
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  intake_roller.spin(forward, 100, pct);
  spinRoller(true);
  move(forward, 1, 10, State(3 TILE + TILE_EDGE + LEFT_TO_CENTER, 1.2 TILE - (TILE_EDGE + TOP_TO_CENTER), 0, 1));
  wait(10000, sec);
  wait(1, sec);
  Turn(180, 100);
  wait(1, sec);
  Turn(90, 100);
  wait(1, sec);
  Turn(45, 100);
  wait(1, sec);
  Turn(44, 100);
  wait(1, sec);
  Turn(0, 100);
  wait(10000, sec);
  move(forward, 1, 10, State(5 TILE + TILE_EDGE + LEFT_TO_CENTER, 3 TILE - (TILE_EDGE + TOP_TO_CENTER), 50, 10));
  wait(100, msec);
  Turn(0, 100);
  while (true) {
    displayTracking();
    wait(50, msec);
  }
  while (true) {
    compressionBar.set(false);
    flywheel.spin(forward, 12, volt);
    intake_roller.spin(forward, 100, pct);
    wait(5, sec);
    wait(100, msec);
    while (flywheel.velocity(rpm) * 6 < 3400) {
      wait(10, msec);
      printf("speed = %f\n", flywheel.velocity(rpm) * 6);
    }
    compressionBar.set(true);
    intake_roller.spin(reverse, 100, pct);
    wait(5, sec);
  }
  
}

void matchLoadTesting() {
  vex::task flywheelOn = vex::task(flywheelPID);
    queueDiscs(0, 2570, 0.5, -1, true, 180);

  queueDiscs(0, 2900);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  intake_roller.spin(forward, 100, percent);
  wait(10000, sec);
}

void skills() {
  globalX = 1 TILE + 4 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER + 5.25);
  vex::task track = vex::task(startTracking);
  resetDiscCount();
  vex::timer matchTime = vex::timer();

  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 2540, 0.5, -1, true, 180);
  discsIntaked = 2; //2
  intake_roller.spin(forward, 100, pct);
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  rightRollerSensor.setLightPower(100);
  leftRollerSensor.setLightPower(100);
  spinRoller(true, true, 1300);
  
  vex::task runIntake = vex::task(maintain3Discs);
  // move to preload shoot position
  

  move(forward, 2, 0.001, 1, 2600, State(1 TILE, 1 TILE, 315, 0.001, 50), State(19, 3 TILE + 3, 1, 2));

  queueDiscs(3);
  waitWhileShooting();
  queueDiscs(0, 2710);

  // get discs along low goal barrier (far low goal)
  move(forward, 2, 0.001, 50, 2200, State(1 TILE - 1, 4 TILE - 12, 97, 0.001, 40), State(2 TILE + 2, 4 TILE - 7, 90, 20, 30));


  // reverse to shooting position
  move(reverse, 1, 4, 2, State(1 TILE, 3 TILE, 355, 1));
  
  queueDiscs(3);
  waitWhileShooting();

  queueDiscs(0, 2740);
  // get line discs
  Turn(110, 100, 2);

  move(forward, 2, 4, 50, State(1 TILE + 6, 3 TILE - 5, 100, 2, 40), State(2.5 TILE, 3.5 TILE, 45, 9, 40));

  Turn(315, 100);
  wait(300, msec);

  queueDiscs(3);
  waitWhileShooting();
  queueDiscs(0, 2400);


  // get rest of low goal discs (vertical)
  move(forward, 3, 0.001, 50, 3500, State(2 TILE + 12, 4 TILE - 9, 5, 1, 30), State(2 TILE + 7, 5 TILE, 0, 10, 50), State(2 TILE + 7, 5 TILE + 12, 0, 1, 40));
  Turn(257, 100, 0.5, 600);
  wait(200, msec);
  queueDiscs(3);
  waitWhileShooting();

  queueDiscs(0, 2840, -1, -1, true, 170);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  // match load position
  double matchLoadY = globalY + ((distanceSensor.objectDistance(inches) + 6) * sin(toRad(globalAngle - 180))) - (2.7 + LEFT_TO_CENTER);
  move(reverse, 2, 3, 0.5, State(3 TILE + 1, matchLoadY, 265, 1), State(3 TILE + 3, matchLoadY, 265, 0.001));
  
  wait(4000, msec);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 2600, -1, -1, true, 180);
  // intake opponent 3 stack
  //Turn(145, 100);
  /*intakeLift.set(true);
  move(forward, 1, 3, State(3 TILE + 9, 4 TILE + 20, 135, 10, 50));
  intakeLift.set(false);
  wait(2000, msec);
  // Turn + shoot
  Turn(280, 100);
  wait(300, msec);

    queueDiscs(3);
  while (numQueued() > 0) {
    wait(10, msec);
  }*/
  //wait(2000, msec);
  // Get to right roller
  //move(reverse, 2, 0.001, State(4 TILE + 5, 4 TILE + 2, 270, 10, 30), State(5.35 TILE, 4.45 TILE, 270, 10, 70));
  moveParallel(reverse, 4, 0.001, State(4.5 TILE - 3, 5 TILE + 10, 270, 7, 80), State(5 TILE - 6, 5 TILE + 7, 270, 2, 50), State(5 TILE, 5 TILE + 1, 270, 3, 50), State(6 TILE + 4, 5 TILE - 2, 270, 15, 70));

  // spinRoller(false);
  spinRoller(true, true, 3500);
  exitMove = true;
  // intake far center 3-stack
  wait(50, msec);

  move(forward, 2, 0.001, 50, State(5 TILE - 3, 5 TILE - 10, 240, 0.001, 60), State(4 TILE + 5, 4 TILE + 5, 240, 15, 30));
  wait(200, msec);

  // move to far roller
  moveParallel(reverse, 1, 0.001, State(4 TILE + 12, 6 TILE + 4, 180, 10));
  wait(300, msec);
  if (!discAtBottom()) {
    resetIntakeSpeed();
  }
  spinRoller(true, true, 3500);
  exitMove = true;
  wait(100, msec);
  exitMove = false;
  // go to shoot discs in robot
  move(forward, 1, 0.001, 1, State(5 TILE + 6, 3 TILE - 2, 177, 20));
  queueDiscs(3);
  waitWhileShooting();

 

  // intake low goal discs (close goal)
  move(forward, 2, 0.001, 50, 2200, State(5 TILE, 2 TILE + 11, 270, 0.001, 40), State(4 TILE, 2 TILE + 7, 270, 15, 40));

  // reverse to shooting position
  move(reverse, 1, 4, 2, State(5 TILE, 3 TILE, 177, 1));
  queueDiscs(3);
  waitWhileShooting();

   queueDiscs(0, 2700);

  // intake second line
   
   Turn(300, 100, 2);

  move(forward, 2, 4, 50, State(5 TILE - 8, 3 TILE + 6, 280, 4, 40), State(3.5 TILE, 2.5 TILE, 225, 9, 60));
  Turn(131, 100);
  wait(300, msec);

  queueDiscs(3);
  waitWhileShooting();


  queueDiscs(0, 2350);
  // get rest of low goal discs (vertical)
  move(forward, 3, 0.001, 50, 3200, State(4 TILE - 12, 2 TILE + 7, 180, 1, 30), State(4 TILE - 7, 1.5 TILE, 180, 10, 40), State(4 TILE - 7, 1 TILE - 13, 180, 1, 40));
  Turn(74, 100);
  queueDiscs(3);
  waitWhileShooting();


  // match loading
  
  queueDiscs(0, 2900, -1, -1, true, 150);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  matchLoadY = globalY - ((distanceSensor.objectDistance(inches) + 6) * sin(toRad(globalAngle))) + 2.7 + LEFT_TO_CENTER;

  move(reverse, 1, 3, State(3 TILE - 1, matchLoadY, 87, 3));


  wait(4000, msec);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 2600, -1, -1, true, 180);

  // move to match load position

  // move to and intake own 3 stack
  //move(forward, 1, 3, State(2.5 TILE, 1.5 TILE, 315, 10));

  // turn to shoot
  //Turn(110, 100);

  // move to roller
  
  moveParallel(reverse, 2, 10, State(2 TILE, 0.75 TILE, 110, 5), State(0 TILE - 3, 1 TILE, 90, 30));
  //lDrive.spin(reverse, 100, pct);
  //rDrive.spin(reverse, 100, pct);
  spinRoller(true, true, 3000);
  exitMove = true;
  // go to endgame position
  wait(100, msec);
  exitMove = false;
  move(forward, 1, 15, State(1 TILE, 1 TILE, 45, 15));
  moveParallel(reverse, 1, 0.001, State(0.5 TILE, 0.5 TILE, 45, 10, 100));
  while (matchTime.time() < 55800) {
    wait(10, msec);
  }

  expansion.set(true);

  while(true) {
    displayTracking();
    wait(10, msec);
  }

  while(true) {
    displayTracking();
    wait(10, msec);
  }
}

void skillsP2() {
  inertialSensor.setHeading(270, deg);
  globalX = 3 TILE + TILE_EDGE + 2.75;
  globalY = 5 TILE + (LEFT_TO_CENTER + 8 + 5/16);
  globalAngle = 270;
  initHeading = 270;
  vex::task track = vex::task(startTracking);
  vex::timer matchTime = vex::timer();
  resetDiscCount();
    vex::task flywheelOn = vex::task(flywheelPID);

  queueDiscs(0, 2600, 0.5, 0.5, true, 180);

  rightRollerSensor.setLightPower(100);
  leftRollerSensor.setLightPower(100);
  vex::task runIntake = vex::task(maintain3Discs);

  wait(4000, msec);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 2600, -1, -1, true, 180);
  // intake opponent 3 stack
  //Turn(145, 100);
  /*intakeLift.set(true);
  move(forward, 1, 3, State(3 TILE + 9, 4 TILE + 20, 135, 10, 50));
  intakeLift.set(false);
  wait(2000, msec);
  // Turn + shoot
  Turn(280, 100);
  wait(300, msec);

    queueDiscs(3);
  while (numQueued() > 0) {
    wait(10, msec);
  }*/
  //wait(2000, msec);
  // Get to right roller
  //move(reverse, 2, 0.001, State(4 TILE + 5, 4 TILE + 2, 270, 10, 30), State(5.35 TILE, 4.45 TILE, 270, 10, 70));
  moveParallel(reverse, 4, 0.001, State(4.5 TILE - 3, 5 TILE + 10, 270, 7, 80), State(5 TILE - 6, 5 TILE + 7, 270, 2, 50), State(5 TILE, 5 TILE + 1, 270, 3, 50), State(6 TILE + 4, 5 TILE - 3, 270, 15, 70));

  // spinRoller(false);
  spinRoller(true, true, 4000);
  exitMove = true;
  // intake far center 3-stack
  wait(50, msec);

  move(forward, 2, 0.001, 50, State(5 TILE - 3, 5 TILE - 10, 240, 0.001, 60), State(4 TILE + 5, 4 TILE + 5, 240, 15, 30));
  wait(200, msec);

  // move to far roller
  moveParallel(reverse, 1, 0.001, State(4 TILE + 12, 6 TILE + 4, 180, 10));
  wait(300, msec);
  if (!discAtBottom()) {
    resetIntakeSpeed();
  }
  spinRoller(true, true, 3500);
  exitMove = true;
  wait(100, msec);
  exitMove = false;
  // go to shoot discs in robot
  move(forward, 1, 0.001, 1, State(5 TILE + 4, 3 TILE - 3, 178, 20));
  queueDiscs(3);
  waitWhileShooting();

 

  // intake low goal discs (close goal)
  move(forward, 2, 0.001, 50, 2200, State(5 TILE - 3, 2 TILE + 9, 270, 0.001, 40), State(4 TILE, 2 TILE + 6, 270, 15, 40));

  // reverse to shooting position
  move(reverse, 1, 4, 2, State(5 TILE, 3 TILE, 177, 1));
  queueDiscs(3);
  waitWhileShooting();

   queueDiscs(0, 2700);

  // intake second line
   
   Turn(300, 100, 2);

  move(forward, 2, 4, 50, State(5 TILE - 8, 3 TILE + 6, 280, 4, 40), State(3.5 TILE, 2.5 TILE, 225, 9, 60));
  Turn(131, 100);
  wait(300, msec);

  queueDiscs(3);
  waitWhileShooting();


  queueDiscs(0, 2350);
  // get rest of low goal discs (vertical)
  move(forward, 3, 0.001, 50, 3500, State(4 TILE - 15, 2 TILE + 7, 180, 1, 30), State(4 TILE - 8, 1 TILE, 180, 10, 40), State(4 TILE - 8, 1 TILE - 14, 180, 1, 40));
  Turn(74, 100);
  queueDiscs(3);
  waitWhileShooting();


  // match loading
  
  queueDiscs(0, 2900, -1, -1, true, 150);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  double matchLoadY = globalY - ((distanceSensor.objectDistance(inches) + 6) * sin(toRad(globalAngle))) + 2.7 + LEFT_TO_CENTER;

  move(reverse, 1, 3, State(3 TILE - 2, matchLoadY, 88, 3));


  wait(4000, msec);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 2600, -1, -1, true, 180);

  // move to match load position

  // move to and intake own 3 stack
  //move(forward, 1, 3, State(2.5 TILE, 1.5 TILE, 315, 10));

  // turn to shoot
  //Turn(110, 100);

  // move to roller
  
  moveParallel(reverse, 2, 10, State(2 TILE, 0.5 TILE, 110, 5), State(0 TILE - 3, 1 TILE, 90, 30));
  //lDrive.spin(reverse, 100, pct);
  //rDrive.spin(reverse, 100, pct);
  spinRoller(true, true, 4000);
  exitMove = true;
  // go to endgame position
  wait(100, msec);
  exitMove = false;
  move(forward, 1, 15, State(1 TILE, 1 TILE, 45, 15));
  moveParallel(reverse, 1, 0.001, State(0.5 TILE, 0.5 TILE, 45, 10, 100));
  while (matchTime.time() < 55800) {
    wait(10, msec);
  }

  expansion.set(true);

  while(true) {
    displayTracking();
    wait(10, msec);
  }

  while(true) {
    displayTracking();
    wait(10, msec);
  }
}

void winPoint9(bool redAlliance) {
  globalX = 1 TILE + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (2.625 + TILE_EDGE + TOP_TO_CENTER);
  vex::task track = vex::task(startTracking);
  resetDiscCount();

  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3400, 0.5, -1, false);
  //flywheel.spin(forward, 12, volt);
  discsIntaked = 2;
  vex::task runIntake = vex::task(maintain3Discs);

  move(forward, 1, 0.001, State(1 TILE + 4, 1 TILE - 10, 330, 1, 30));
  wait(300, msec);
  //endOfMovePrecision = 40;
  intakeSpeed = 25;
  intake_roller.spin(forward, 25, percent);
  moveParallel(reverse, 1, 5, State(1 TILE + 4, 12, 30, 5, 30));
  spinRoller(true, redAlliance, 2000);
  runWithDelay(resetIntakeSpeed, 100);
  move(forward, 1, 1, 0.5, State(1 TILE + 4, 1 TILE - 5, 340, 0.001));

  //endOfMovePrecision = 2;

  queueDiscs(3, 3400, 0.5, 0.5, false, 300);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  queueDiscs(0, 3000);
  intakeLift.set(true);
  Turn(65, 100);
  move(forward, 1, 0.001, 50, State(2.3 TILE, 1.3 TILE, 65, 0.001, 100));
  intakeLift.set(false);
  wait(2000, msec);
  Turn(320, 100);

  queueDiscs(3, 3000, 0.5, 0.5, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  queueDiscs(0, 3400, 0.5, 0.5, false);


  Turn(45, 100);
  move(forward, 1, 10, 10, State(5 TILE + 2, 4 TILE + 8, 270, 0.001, 50));
  intakeSpeed = 25;
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  spinRoller(false, redAlliance);

 move(forward, 1, 1, State(5 TILE - 3, 4 TILE + 8, 280, 0.001));
  
  queueDiscs(3, 3400, 0.5, 0.5, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }


}

void winPoint6(bool redAlliance) {
  globalX = 1 TILE + 4 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER + 5.25);
  vex::task track = vex::task(startTracking);
  resetDiscCount();

  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3250, 0.2, 0.7, false);
  //flywheel.spin(forward, 12, volt);
  intakeSpeed = 25;
  discsIntaked = 0;
  //endOfMovePrecision = 20;
  vex::task runIntake = vex::task(maintain3Discs);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);

  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(true, redAlliance, 500);


  intakeLift.set(true);


  runWithDelay(resetIntakeSpeed, 100);
 
  //endOfMovePrecision = 10;

  move(forward, 1, 0.001, State(2 TILE + 4.5, 1 TILE + 5.5, 60, 7, 70));
  intakeLift.set(false);
  wait(2000, msec);
  //endOfMovePrecision = 10;
  move(forward, 1, 1, State(3 TILE, 2 TILE, 45, 10, 100));
  wait(300, msec);
  Turn(323, 100, 1);

  queueDiscs(3, 3250, 0.5, 0.8, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  
  queueDiscs(0, 3400, 0.5, -1, false);

  wait(200, msec);
  Turn(45, 100, 10);
  move(forward, 2, 1, State(4.5 TILE, 3.5 TILE, 45, 0.001, 100), State(5 TILE + 5, 4 TILE + 10, 45, 0.001, 100));
  Turn(270, 100, 3);
  intakeSpeed = 25;
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  spinRoller(false, redAlliance);
  runWithDelay(resetIntakeSpeed, 400);
  //endOfMovePrecision = 2;
 move(forward, 1, 1, State(5 TILE + 5, 4 TILE + 13, 276, 0.001));
  
  queueDiscs(3, 3400, 0.5, 1.0, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }


}

void leftSide(bool redAlliance) {
  globalX = 1 TILE + 4 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER + 5.25);
  vex::task track = vex::task(startTracking);
  resetDiscCount();

  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3150, 0.5, -1, false);
  //flywheel.spin(forward, 12, volt);
  intakeSpeed = 25;
  discsIntaked = 0;
  //endOfMovePrecision = 5;
  vex::task runIntake = vex::task(maintain3Discs);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);

  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(true, redAlliance);


  intakeLift.set(true);


  runWithDelay(resetIntakeSpeed, 200);
  move(forward, 2, 0.001, State(2 TILE, 1 TILE, 45, 3), State(2 TILE + 4.5, 1 TILE + 5.5, 45, 4, 100));
  intakeLift.set(false);
  wait(2000, msec);
  move(forward, 1, 1, State(3 TILE, 2 TILE, 45, 10, 100));
  wait(300, msec);
  Turn(326, 100);

  queueDiscs(3, 3150, 0.5, -1, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }
}

void leftSide9(bool redAlliance) {
  globalX = 1 TILE + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (2.625 + TILE_EDGE + TOP_TO_CENTER);
  vex::task track = vex::task(startTracking);
  resetDiscCount();
  volleySpeed = 45;
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3420, 0.5, -1, false);
  //flywheel.spin(forward, 12, volt);
  discsIntaked = 2;
  vex::task runIntake = vex::task(maintain3Discs);

  move(forward, 1, 0.001, 30, State(27, 18, 330, 1, 100));

  //endOfMovePrecision = 40;
  intakeSpeed = 25;
  intake_roller.spin(forward, 25, percent);
  moveParallel(reverse, 1, 0.001, State(1 TILE + 8, 0, 0, 7, 100));
  spinRoller(true, redAlliance, 1000);
  exitMove = true;
  wait(100, msec);
  exitMove = false;
  runWithDelay(resetIntakeSpeed, 100);
  move(forward, 1, 0.001, 0.5, State(38.5, 19 , 347, 0.001));

  //endOfMovePrecision = 2;
  intakeLift.set(true);
  
  //queueDiscs(3, 3600, 0.5, 0.5, true, 500);

  queueDiscs(3, 3420, 0.5, 0.5, true, 500);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  queueDiscs(0, 3400);
 
  wait(100, msec);
  printf("mark: %d\n", 1);
  move(forward, 1, 1, 0.5, State(36.8, 27.8, 347, 1));
  printf("mark: %d\n", 2);

  intakeLift.set(false);
  wait(2000, msec);
  intakeLift.set(true);
  queueDiscs(3, 3400, 0.5, 0.5, true, 500);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  queueDiscs(0, 3350);
  exitMove = true;
  wait(50, msec);

  //move(reverse, 1, 3, 10, State(2 TILE - 10, 1 TILE - 1, 5, 3));
  Turn(54, 100, 10);
  moveParallel(forward, 1, 0.001, 10, State(2 TILE + 3, 1 TILE + 10, 55, 5));
  while (Point(globalX, globalY).distTo(Point(2 TILE + 3, 1 TILE + 10)) > 5) {
    wait(10, msec);
  }
  intakeLift.set(false);

  
  
  wait(1000, msec);
  exitMove = true;
  Turn(338, 100);
  intakeLift.set(true);
  queueDiscs(3, 3350, 0.5, 0.5, true, 500);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  wait(1000, sec);
}

void leftSideCut(bool redAlliance) {
  globalX = 1 TILE + 4 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER + 5.25);
  vex::task track = vex::task(startTracking);
  resetDiscCount();

  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3250, 0.5, -1, false);
  //flywheel.spin(forward, 12, volt);
  intakeSpeed = 25;
  discsIntaked = 2;
  //endOfMovePrecision = 5;
  vex::task runIntake = vex::task(maintain3Discs);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);

  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(true, redAlliance);


  intakeLift.set(true);


  runWithDelay(resetIntakeSpeed, 200);
  move(forward, 1, 0.001, State(1.5 TILE, 1 TILE, 345, 3));
  
  queueDiscs(2, 3250, 0.5, -1, false);  
  while (numQueued() > 0) {
    wait(10, msec);
  }
  Turn(75, 100);
  //endOfMovePrecision = 50;
  move(forward, 1, 5, State(2 TILE + 5, 1 TILE + 10, 75, 0.001, 70));
  intakeLift.set(false);
  wait(2000, msec);
  Turn(337, 100);

  queueDiscs(3, 3150, 0.5, -1, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }
}

void leftSafe(bool redAlliance) {
  globalX = 1 TILE + 4 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER + 5.25);
  vex::task track = vex::task(startTracking);
  resetDiscCount();

  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3150, 0.5, -1, false);
  //flywheel.spin(forward, 12, volt);
  intakeSpeed = 25;
  discsIntaked = 0;
  //endOfMovePrecision = 5;
  vex::task runIntake = vex::task(maintain3Discs);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);

  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(true, redAlliance);


  intakeLift.set(true);


  runWithDelay(resetIntakeSpeed, 200);
  move(forward, 2, 0.001, State(2 TILE, 1 TILE, 45, 3), State(2 TILE + 4.5, 1 TILE + 5.5, 45, 4, 100));
  intakeLift.set(false);
  wait(2000, msec);
 
  Turn(333, 100);

  queueDiscs(3, 3200, 0.5, -1, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }
}

void rightSide5(bool redAlliance) {
  globalX = 5 TILE + 2.5 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 4 TILE - (TILE_EDGE + TOP_TO_CENTER);
  vex::task track = vex::task(startTracking);
  resetDiscCount();
  discsIntaked = 2;
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3300, 0.5, -1, false);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);
  //endOfMovePrecision = 10;
  move(forward, 1, 1, State(5 TILE + 9.375, 5 TILE - 12, 0, 1));
  Turn(270, 100, 5);
  intake_roller.spin(forward, 50, pct);
  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(false, redAlliance);
  //endOfMovePrecision = 1;
  move(forward, 1, 2, State(5 TILE + 6, 5 TILE - 12, 277, 1));
  vex::task runIntake = vex::task(maintain3Discs);

  queueDiscs(2, 3300, -1, 1, false);
  while(numQueued() > 0){
    wait(10, msec);
  }
  queueDiscs(0, 2950, -1, -1, false);
  move(forward, 1, 0.001, /*State(4.5 TILE, 3.5 TILE, 45, 10, 30),*/ State(3.5 TILE, 2.5 TILE, 313, 0.001, 50));
  wait(800, msec);

  queueDiscs(3, 2950, -1, 0.7, false);
  while(numQueued() > 0){
    wait(10, msec);
  }

}

void rightSide6(bool redAlliance) {
  globalX = 5 TILE + 2.5 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 4 TILE - (TILE_EDGE + TOP_TO_CENTER);
  vex::task track = vex::task(startTracking);
  resetDiscCount();
  discsIntaked = 2;
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3400, 0.5, -1, false);
  vex::task runIntake = vex::task(maintain3Discs);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);
  //endOfMovePrecision = 5;
  move(forward, 1, 1, State(5 TILE + 2, 5 TILE - 9, 340, 5)); 
  move(reverse, 1, 1, State(5 TILE + 9.375, 5 TILE - 12, 270, 1));

  intake_roller.spin(forward, 50, pct);
  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(false, redAlliance);
  //endOfMovePrecision = 1;
  move(forward, 1, 2, State(5 TILE + 6, 5 TILE - 12, 277, 1));


  queueDiscs(3, 3300, -1, -1, false);
  while(numQueued() > 0){
    wait(10, msec);
  }
  queueDiscs(0, 2950, -1, -1, false);
  move(forward, 1, 0.001, /*State(4.5 TILE, 3.5 TILE, 45, 10, 30),*/ State(3.5 TILE, 2.5 TILE, 313, 0.001, 50));
  wait(800, msec);

  queueDiscs(3, 2950, -1, 0.7, false);
  while(numQueued() > 0){
    wait(10, msec);
  }
}

void rightMod(bool redAlliance) {
  globalX = 5 TILE + 2.5 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 4 TILE - (TILE_EDGE + TOP_TO_CENTER);
  vex::task track = vex::task(startTracking);
  resetDiscCount();
  discsIntaked = 2;
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3400, 0.5, -1, false);
  vex::task runIntake = vex::task(maintain3Discs);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);
  //endOfMovePrecision = 5;
  move(forward, 1, 1, State(5 TILE + 2, 5 TILE - 9, 340, 5)); 
  move(reverse, 1, 1, State(5 TILE + 9.375, 5 TILE - 12, 270, 1));

  intake_roller.spin(forward, 50, pct);
  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(false, redAlliance);
  //endOfMovePrecision = 1;
  move(forward, 1, 2, State(5 TILE + 6, 5 TILE - 12, 277, 1));


  queueDiscs(3, 3400, -1, -1, false);
  while(numQueued() > 0){
    wait(10, msec);
  }


}

void rightSide9(bool redAlliance) {
  intakeLift.set(false);
  globalX = 5 TILE + 2.5 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 4 TILE - (TILE_EDGE + TOP_TO_CENTER);
  vex::task track = vex::task(startTracking);
  resetDiscCount();
  discsIntaked = 2;
  volleySpeed = 30;
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3280, 0.5, -1, false, 500);
  vex::task runIntake = vex::task(maintain3Discs);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);
  //endOfMovePrecision = 5;
  move(forward, 1, 1, 20, State(5 TILE + 4, 5 TILE - 8, 340, 5)); 
  moveParallel(reverse, 2, 0.001, State(6 TILE - 15, 4 TILE + 15, 275, 1), State(6 TILE + 30, 4 TILE + 9, 270, 40));
  intakeSpeed = 25;
  intake_roller.spin(forward, 25, pct);

  spinRoller(false, redAlliance, 1300);
  exitMove = true;
  
  
  wait(50, msec);
  //endOfMovePrecision = 1;

  runWithDelay(resetIntakeSpeed, 100);
intakeLift.set(true); 
  moveParallel(forward, 1, 0.001, 0.5, State(5 TILE - 3, 4 TILE + 12, 279, 0.001, 50));

  while (Point(globalX, globalY).distTo(Point(5 TILE - 3, 4 TILE + 12)) > 2) {
    wait(10, msec);
  }
  queueDiscs(3, 3280, 0.5, 0.5, true, 500);

  waitWhileShooting();
  

  //moveParallel(forward, 1, 0.001, 0.5, State(5 TILE - 4, 4 TILE + 13, 278, 0.001));

  queueDiscs(0, 3280, 0.5, 0.5, true, 500);
  intakeLift.set(false);
  wait(1500, msec);
  queueDiscs(3, 3280, 0.5, 0.5, true, 500);
  intakeLift.set(true);
  waitWhileShooting();
  intakeLift.set(false);

  queueDiscs(0, 3230, -1, -1, false);
  Turn(210, 100, 10);
  move(forward, 3, 0.001, 0.5, State(4.5 TILE, 3.5 TILE, 220, 3, 60), State(4 TILE, 3 TILE, 225, 10), State(3.75 TILE, 2.75 TILE, 311, 0.001, 100));

  queueDiscs(3, 3230, 0.5, 0.5, true, 300);
    intakeLift.set(true);
  while(numQueued() > 0){
    wait(10, msec);
  }

}


