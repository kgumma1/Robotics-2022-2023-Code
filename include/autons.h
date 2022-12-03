#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

void skills() {

}

void matchFarRoller() {
  drivePID(-12);
  drivePID(57, 9);
  Turn(90, 80);
  spinLeft(15);
  spinRight(15);
  wait(2000, msec);
  Intake_Roller.spinFor(0.3, rotationUnits::rev, 100, velocityUnits::pct, true);

}

void matchWP3() {
  // flywheel setup
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.0001, 0);
  queueDiscs(0, 2300, 0.4);

  // first roller
  spinLeft(15);
  spinRight(15);
  wait(400, msec);
  Intake_Roller.spinFor(0.25, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(500, msec);

 
  drivePID(-4);

  Turn(-131, 100);
  vex::task intake = vex::task(intake1);

  drivePID(66, 11);
  wait(500, msec);
  intake.stop();
  Intake_Roller.spinFor(forward, 0.25, rev, 100, velocityUnits::pct);
  Turn(-43, 100);
  wait(200, msec);
  Intake_Roller.spin(reverse, 100, pct);
  queueDiscs(3, 2300, 0.4);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  flywheelOn.stop();
  FlywheelUp.spin(forward, 4, volt);
  FlywheelDown.spin(forward, 4, volt);

  vex::task intake2 = vex::task(intake3);
  wait(200, msec);
  Turn(-133, 100);
  intake.resume();
  drivePID(68, 11);
  FlywheelUp.stop(coast);
  FlywheelDown.stop(coast);
  intake2.stop();
  spinLeft(30);
  FRDrive.stop(hold);
  BRDrive.stop(hold);
  while (getInertialReading() < -110) {
      wait(10, msec);
  }
  stopBase(hold);
  spinLeft(20);
  spinRight(20);
  Indexer.set(false);
  Intake_Roller.stop(coast);
  wait(1500, msec);

  Intake_Roller.spinFor(0.25, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(10, sec);

}

void matchWP5() {
  // flywheel setup
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.005);
  queueDiscs(0, 2800, 0.3);

  // first roller
  spinLeft(15);
  spinRight(15);
  wait(400, msec);
  Intake_Roller.spinFor(0.25, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(500, msec);

  // align to shoot
  drivePID(-5);
  Turn(-7, 100);

  // shoot
  wait(100, msec);
  queueDiscs(2, 2800, 0.3);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  wait(500, msec);
  queueDiscs(0, 2300, 0.3);
  drivePID(1);
  Turn(-133, 100);
  vex::task intake = vex::task(intake3);

  drivePID(66, 11);
  Turn(-45, 100);
  queueDiscs(3, 2300, 0.3);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  intake.stop();
  vex::task intake2 = vex::task(intake3);
  wait(200, msec);
  Turn(-130, 100);
  intake.resume();
  drivePID(66, 11);
  spinLeft(100);
  FRDrive.stop(hold);
  BRDrive.stop(hold);
  wait(500, msec);
  Intake_Roller.spinFor(0.25, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(10, sec);
}

void testing() {/*
  while(true) {
    printf("i = %f\n", getInertialReading());
    wait(10, msec);
  }*//*
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  queueDiscs(0, 2000, 0.5);
  wait(3, sec);
  // shoot
  wait(300, msec);
  queueDiscs(2, 2600, 0.5);
  adjustFPID(0.00004, 0.00006, 0, 0);
  while (numQueued() > 0) {
    wait(10, msec);
  }*/
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.0001, 0);
  queueDiscs(0, 2300, 0.4);
  wait(4, sec);

  queueDiscs(3, 2300, 0.4);
  while (numQueued() > 0) {
    wait(10, msec);
  }

  wait(30, sec);
  printf("dsfakjdlkj:%d\n", 5);
  /*leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  while(true){
    printf("left = %f, right = %f\n", leftEncoder.position(deg), rightEncoder.position(deg));
  }*/
}