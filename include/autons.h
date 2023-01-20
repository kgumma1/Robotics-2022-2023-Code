#include "vex.h"
#include "autonFunctions.h"

using namespace vex;


void testing() {
  /*resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(3, 3500, 0.4);*/

  drivePID(20);
  wait(30, sec);/*
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

void winPoint() {
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.00005, 0);
  queueDiscs(0, 3500, 0.4);

  drivePID(-3, 100, 2);
  intake_roller.spinFor(1.25, rev, 100, velocityUnits::pct, false);
  wait(300, msec);
  Turn(-5, 100);
  vex::task startIntake = vex::task(maintain3Discs);
  drivePID(7);


}
