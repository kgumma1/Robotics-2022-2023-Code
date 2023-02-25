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

void testing() {
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

void skills() {
  globalX = 1 TILE + 4 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER + 5.25);
  vex::task track = vex::task(startTracking);
  resetDiscCount();

  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 2600, 0.5);
  discsIntaked = 2; //2
  intake_roller.spin(forward, 100, pct);
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  rightRollerSensor.setLightPower(100);
  leftRollerSensor.setLightPower(100);
  spinRoller(true);
  
  vex::task runIntake = vex::task(maintain3Discs);
  // move to preload shoot position
  
  endOfMovePrecision = 2;
  move(forward, 1, 0.001, State(0.7 TILE, 1 TILE, 315, 0.001, 50));
  Turn(90, 100);
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  spinRoller(true);
  move(forward, 1, 2, State(17, 3 TILE + 4, 7, 3, 100));
  queueDiscs(3);
  while (numQueued() > 0) {
    printf("shooting %d\n", 1);
    wait(10, msec);
  }

  // get discs along low goal barrier (far low goal)
  move(forward, 2, 0.001, State(17, 4 TILE - 18, 90, 0.001, 20), State(2 TILE, 4 TILE - 8, 90, 30, 30));


  // reverse to shooting position
  move(reverse, 1, 4, State(24, 3 TILE, 5, 10));
  
  queueDiscs(3);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  
  // get line discs
  Turn(110, 100, 2);

  move(forward, 2, 4, State(1 TILE + 13, 3 TILE - 12, 45, 5, 40), State(2.5 TILE, 3.5 TILE, 45, 9, 40));
  wait(300, msec);
  Turn(319, 100);

  queueDiscs(3);
  while (numQueued() > 0) {
    wait(10, msec);
  }

  // get rest of low goal discs (vertical)
  move(forward, 2, 0.001, State(2 TILE + 11, 4 TILE - 11.75, 0, 1, 20), State(2 TILE + 6, 5 TILE + 11, 0, 30, 30));
  Turn(260, 100);
  queueDiscs(3);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  queueDiscs(0, 2900);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  // match load position
  move(reverse, 1, 3, State(3 TILE + 2, 5 TILE + 12, 270, 3));
  
  wait(7000, msec);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 2600);
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
  move(reverse, 2, 0.001, State(4 TILE, 5 TILE, 270, 10, 30), State(5 TILE + 5, 5 TILE, 270, 10, 70));
  
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  // spinRoller(false);
  spinRoller(true);
  // intake far center 3-stack
  runIntake.suspend();
  intake_roller.spin(reverse, 100, pct);
  move(forward, 1, 0.001, State(4 TILE + 6, 4.5 TILE, 270, 3));

  // move to far roller
  move(reverse, 1, 0.001, State(4 TILE + 9, 5 TILE + 12, 180, 10));
  runIntake.resume();
  lDrive.spin(reverse, 100, pct); 
  rDrive.spin(reverse, 100, pct);
  spinRoller(true);
  move(forward, 1, 2, State(5 TILE, 5 TILE, 225, 5, 100));
  expansion.set(true);
  // go to shoot discs in robot
 
  // intake second line
  /* 
  move(forward, 3, 40, State(4 TILE + 6, 3 TILE + 6, 225, 6, 100), State(3.5 TILE, 2.5 TILE, 225, 3), State(3.5 TILE, 0.65 TILE, 90, 20, 30));

  Turn(80, 100);
  queueDiscs(3);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  queueDiscs(0, 2900);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  move(reverse, 1, 5, State(3 TILE, 0.5 TILE, 90, 5, 100));


  wait(7000, msec);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 2600);

  // move to match load position

  // move to and intake own 3 stack
  //move(forward, 1, 3, State(2.5 TILE, 1.5 TILE, 315, 10));

  // turn to shoot
  //Turn(110, 100);

  // move to roller
  move(reverse, 1, 50, State(0.75 TILE, 1 TILE + 3, 90, 20));
  //lDrive.spin(reverse, 100, pct);
  //rDrive.spin(reverse, 100, pct);
  spinRoller(true);
  // go to endgame position
  move(forward, 1, 15, State(1 TILE, 1 TILE, 45, 15));

  expansion.set(true);

  while(true) {
    displayTracking();
    wait(10, msec);
  }*/
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
  endOfMovePrecision = 40;
  intakeSpeed = 25;
  move(reverse, 1, 5, State(1 TILE + 4, 12, 30, 5, 30));
  wait(500, msec);
  runWithDelay(resetIntakeSpeed, 100);
  move(forward, 1, 1, State(2 TILE, 1 TILE, 55, 0.001));

  endOfMovePrecision = 2;
  Turn(346, 100);

  queueDiscs(3, 3400, 0.5, -1, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  queueDiscs(0, 3000);
  intakeLift.set(true);
  Turn(45, 100);
  move(forward, 1, 0.001, State(2.3 TILE, 1.3 TILE, 45, 0.001, 100));
  intakeLift.set(false);
  wait(2000, msec);
  move(forward, 1, 1, State(3.1 TILE, 2.1 TILE, 45, 10, 100));
  wait(300, msec);
  Turn(322, 100);

  queueDiscs(3, 3000, 0.5, -1, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  queueDiscs(0, 3400, 0.5, -1, false);

  wait(100, msec);
  Turn(45, 100);
  move(forward, 1, 1, State(5 TILE + 2, 4 TILE + 8, 270, 0.001, 50));
  intakeSpeed = 50;
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  spinRoller(false, redAlliance);

 move(forward, 1, 1, State(5 TILE - 3, 4 TILE + 8, 280, 0.001));
  
  queueDiscs(3, 3400, 0.5, -1, false);
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
  endOfMovePrecision = 20;
  vex::task runIntake = vex::task(maintain3Discs);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);

  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(true, redAlliance, 500);


  intakeLift.set(true);


  runWithDelay(resetIntakeSpeed, 200);
 
  endOfMovePrecision = 10;

  move(forward, 1, 0.001, State(2 TILE + 4.5, 1 TILE + 5.5, 60, 7, 70));
  intakeLift.set(false);
  wait(2000, msec);
  endOfMovePrecision = 10;
  move(forward, 1, 1, State(3 TILE, 2 TILE, 45, 10, 100));
  wait(300, msec);
  Turn(323, 100, 1);

  queueDiscs(3, 3250, 0.5, 0.8, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  
  queueDiscs(0, 3450, 0.5, -1, false);

  wait(200, msec);
  Turn(45, 100, 10);
  move(forward, 2, 1, State(4.5 TILE, 3.5 TILE, 45, 0.001, 100), State(5 TILE + 5, 4 TILE + 10, 45, 0.001, 100));
  Turn(270, 100, 3);
  intakeSpeed = 25;
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  spinRoller(false, redAlliance);
  runWithDelay(resetIntakeSpeed, 400);
  endOfMovePrecision = 2;
 move(forward, 1, 1, State(5 TILE + 5, 4 TILE + 13, 276, 0.001));
  
  queueDiscs(3, 3450, 0.5, 1.0, false);
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
  endOfMovePrecision = 5;
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

void leftSideCut(bool redAlliance) {
  globalX = 1 TILE + 4 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER + 5.25);
  vex::task track = vex::task(startTracking);
  resetDiscCount();

  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00015, 0.00005, 0, 0.001, 0.00007, 0);
  queueDiscs(0, 3150, 0.5, -1, false);
  //flywheel.spin(forward, 12, volt);
  intakeSpeed = 25;
  discsIntaked = 2;
  endOfMovePrecision = 5;
  vex::task runIntake = vex::task(maintain3Discs);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);

  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(true, redAlliance);


  intakeLift.set(true);


  runWithDelay(resetIntakeSpeed, 200);
  move(forward, 1, 0.001, State(1.5 TILE, 1 TILE, 355, 3));
  
  queueDiscs(2, 3150, 0.5, -1, false);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  Turn(45, 100);

  move(forward, 2, 0.001, State(2 TILE, 1 TILE, 45, 3), State(2 TILE + 5, 1 TILE + 6, 45, 4, 100));
  intakeLift.set(false);
  wait(4000, msec);
  Turn(336, 100);

  queueDiscs(3, 3150, 0.5, -1, false);
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
  queueDiscs(0, 3450, 0.5, -1, false);
  leftRollerSensor.setLightPower(100);
  rightRollerSensor.setLightPower(100);
  endOfMovePrecision = 10;
  move(forward, 1, 1, State(5 TILE + 9.375, 5 TILE - 12, 0, 1));
  Turn(270, 100, 5);
  intake_roller.spin(forward, 50, pct);
  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(false, redAlliance);
  endOfMovePrecision = 1;
  move(forward, 1, 2, State(5 TILE + 6, 5 TILE - 12, 277, 1));
  vex::task runIntake = vex::task(maintain3Discs);

  queueDiscs(2, 3450, -1, 1, false);
  while(numQueued() > 0){
    wait(10, msec);
  }
  queueDiscs(0, 3100, -1, -1, false);
  move(forward, 1, 0.001, /*State(4.5 TILE, 3.5 TILE, 45, 10, 30),*/ State(3.5 TILE, 2.5 TILE, 316, 0.001, 50));
  wait(800, msec);

  queueDiscs(3, 3100, -1, 0.7, false);
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
  endOfMovePrecision = 5;
  move(forward, 1, 1, State(5 TILE + 2, 5 TILE - 9, 340, 5)); 
  move(reverse, 1, 1, State(5 TILE + 9.375, 5 TILE - 12, 270, 1));

  intake_roller.spin(forward, 50, pct);
  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  spinRoller(false, redAlliance);
  endOfMovePrecision = 1;
  move(forward, 1, 2, State(5 TILE + 6, 5 TILE - 12, 277, 1));


  queueDiscs(3, 3400, -1, -1, false);
  while(numQueued() > 0){
    wait(10, msec);
  }


}



