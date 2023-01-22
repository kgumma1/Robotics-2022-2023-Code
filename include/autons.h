#include "vex.h"
#include "autonFunctions.h"

using namespace vex;


void testing() {
  /*resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(3, 3500, 0.4);*/
  int discsIntaked = 0;
  //drivePID(24);
  /*
  while (true) {
    while (topIntakeSensor.reflectivity() - 6 <= topIntakeSensorInit) {
      if (discsIntaked >= 3 && discCount <= 0 && bottomIntakeSensor.reflectivity() - 4 > bottomIntakeSensorInit) {
        intake_roller.spin(reverse, 100, pct);
      }
      if (discsIntaked < 3 && discCount <= 0) {
        intake_roller.spin(forward, 100, pct);
      }
      printf("sensor = %ld\n", topIntakeSensor.reflectivity());
      printf("discs = %d\n", discsIntaked);
      wait(5, msec);
    }
    printf("discs = %d\n", discsIntaked);
    while (topIntakeSensor.reflectivity() - 6 > topIntakeSensorInit) {
      if (discsIntaked >= 3 && discCount <= 0 && bottomIntakeSensor.reflectivity() - 4 > bottomIntakeSensorInit) {
        intake_roller.spin(reverse, 100, pct);
      }
      if (discsIntaked < 3 && discCount <= 0) {
        intake_roller.spin(forward, 100, pct);
      }
      printf("sensor = %ld\n", topIntakeSensor.reflectivity());
      printf("discs = %d\n", discsIntaked);
      wait(5, msec);
    }
    discsIntaked++;
    printf("discs = %d\n", discsIntaked);
  }
  Turn(-90, 100);
  wait(30, sec);*/
  maintain3Discs();
  /*
  lDrive.spin(reverse, 100, pct);
  rDrive.spin(reverse, 100, pct);
  wait(500, msec);
  */
  //intake_roller.spinFor(reverse, 1.0, rev, 100, velocityUnits::pct);
}

void roller() {
  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  wait(500, msec);

  intake_roller.spinFor(forward, 1.25, rev, 100, velocityUnits::pct);
}


void farRoller() {
  drivePID(20);
  wait(500, msec);
  Turn(-90, 100);
  lDrive.spin(reverse, 50, pct);
  rDrive.spin(reverse, 50, pct);
  wait(1000, msec);

  intake_roller.spinFor(forward, 1.25, rev, 100, velocityUnits::pct);
}

vex::task runWithDelay(int (*callback)(), double timeMsec) {
  wait(timeMsec, msec);
  vex::task runFunction = vex::task(callback);
  return runFunction;
}

void winPoint9DiscAttempt() {
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 3500, 0.4);

  drivePID(-3, 100, 0.5);
  intake_roller.spinFor(1.25, rev, 100, velocityUnits::pct, false);
  wait(300, msec);
 
  drivePID(2);
  Turn(-20, 100);
  vex::task runIntake = runWithDelay(maintain3Discs, 500);

  drivePID(7);
  wait(1000, msec);
  Turn(-4, 100);
  queueDiscs(3);
  wait(100, sec);
  queueDiscs(0);
}

void winPoint() {
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 3200, 0.5);

  drivePID(-3, 100, 0.5);
  intake_roller.spinFor(1.25, rev, 100, velocityUnits::pct, false);
  wait(300, msec);
 
  drivePID(3);
  Turn(50, 100);
  vex::task runIntake = vex::task(maintain3Discs);/*
  PISTON LIFT ALTERNATIVE CODE
  intakeLift.set(true);
  drivePID(24);
  intakeLift.set(false);

  wait(3000, msec);

  Turn(-16, 100);*/
  drivePID(25);
  drivePID(30, 2);
  wait(200, msec);
  Turn(-32, 100);

  queueDiscs(3);
  vex::timer discTimer = vex::timer();
  while (numQueued() > 0 && discTimer.value() < 4) {
    wait(5, msec);
  }
  resetDiscCount();
  wait(200, msec);
  Turn(45, 100);
  drivePID(85 - 28, 8);/* 2nd volley - not in time
  Turn(-65, 100);
  wait(2000, msec);
  queueDiscs(3);
  discTimer.reset();
  while (numQueued() > 0 && discTimer.value() < 4) {
    wait(5, msec);
  }
  resetDiscCount();
  queueDiscs(0, 3400);
  wait(200, msec);*/
  Turn(-137, 100);
  runIntake.stop();
  intake_roller.stop(coast);
  drivePID(-16);
  Turn(-90, 100);
  drivePID(-20, 100, 1);
  intake_roller.spinFor(1, rev, 100, velocityUnits::pct, false);



  wait(100, sec);

}

void halfWP(){
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 3200, 0.5);

  drivePID(-3, 100, 0.5);
  intake_roller.spinFor(1.25, rev, 100, velocityUnits::pct, false);
  wait(300, msec);
 
  drivePID(3);
  Turn(-162, 90);
  // simple(60, -60, 0.4); // spins left for 60, right for -60, and holds for 0.5 second. 
  vex::task runIntake = vex::task(maintain3Discs);/*
  PISTON LIFT ALTERNATIVE CODE
  intakeLift.set(tmrue);
  drivePID(24);
  intakeLift.set(false);

  wait(3000, msec);

  Turn(-16, 100);*/
  drivePID(25);
  drivePID(30, 2);
  wait(200, msec);
  Turn(-32, 100);

  queueDiscs(3);
  vex::timer discTimer = vex::timer();
  while (numQueued() > 0 && discTimer.value() < 4) {
    wait(5, msec);
  }
  resetDiscCount();
  wait(200, msec);

  Turn(35, 100);
  drivePID(-15);


  
  Turn(45, 100);
  drivePID(85 - 28, 8);
  //2nd volley - not in time
  Turn(-65, 100);
  wait(2000, msec);
  queueDiscs(3);
  discTimer.reset();
  while (numQueued() > 0 && discTimer.value() < 4) {
    wait(5, msec);
  }
  resetDiscCount();
  queueDiscs(0, 3400);
  wait(200, msec);
  
  //Turn(-137, 100);
  runIntake.stop();
  intake_roller.stop(coast);
  /*
  drivePID(-16);
  Tu rn(-90, 100);
  drivePID(-20, 100, 1);
  intake_roller.spinFor(1, rev, 100, velocityUnits::pct, false);



  wait(100, sec);
  */
  
}


void twoRoller() {
  drivePID(-3, 100, 0.5);
  intake_roller.spinFor(1.25, rev, 100, velocityUnits::pct, false);
  wait(300, msec);
 
  drivePID(3);
  Turn(-131, 4);
  drivePID(-125);
  Turn(-90, 100);
  drivePID(-20, 100, 1);
  intake_roller.spinFor(1, rev, 100, velocityUnits::pct, false);
}