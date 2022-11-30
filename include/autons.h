#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

void skills() {

}

void matchCloseSide() {
  // flywheel setup
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  queueDiscs(0, 2300, 0.5);

  // first roller
  spinLeft(15);
  spinRight(15);
  wait(400, msec);
  Intake_Roller.spinFor(0.25, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(500, msec);

  // align to shoot
  drivePID(-5);
  Turn(-10, 100);

  // shoot
  wait(300, msec);
  queueDiscs(2, 2600, 0.5);
  adjustFPID(0.00004, 0.00006, 0, 0);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  wait(500, msec);
  Turn(-130, 100);
  vex::task intake = vex::task(intakeNum);
  drivePID(120, 8);

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
  drivePID(50, 5);
  printf("dsfakjdlkj:%d\n", 5);
  /*leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  while(true){
    printf("left = %f, right = %f\n", leftEncoder.position(deg), rightEncoder.position(deg));
  }*/
}