#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

vex::timer runTime = vex::timer();

int displayTime() {
  Brain.Screen.setFont(mono60);
  while (runTime.value() < 15) {
    Brain.Screen.printAt(5, 50, "Time Left=%f", 15 - runTime.value());
    wait(100, msec);
    Brain.Screen.clearScreen();
  }
  Controller1.rumble("----");
  return 1;
}

void lowGoal() {
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.0001, 0);
  queueDiscs(0, 1800, 0.4);

  // first roller
  spinLeft(15);
  spinRight(15);
  wait(400, msec);
  Intake_Roller.spinFor(0.25, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(500, msec);


}
void halfWP() {
 // flywheel setup
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.0001, 0);
  queueDiscs(0, 2500, 0.4);

  // first roller
  spinLeft(15);
  spinRight(15);
  wait(400, msec);
  Intake_Roller.spinFor(0.25, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(500, msec);

 
  drivePID(-3);

  Turn(-131, 100);
  vex::task intake = vex::task(intake1);

  drivePID(66, 11);
  wait(400, msec);
  intake.stop();
  Intake_Roller.spinFor(forward, 0.25, rev, 100, velocityUnits::pct);
  Turn(-43, 100);
  wait(200, msec);
  Intake_Roller.spin(reverse, 100, pct);
  queueDiscs(3, 2500, 0.4);
  vex::timer t = vex::timer();
  t.reset();
  while (numQueued() > 0 && t.value() < 3.2) {
    wait(10, msec);
  }
  flywheelOn.stop();
  Turn(0, 100);
  drivePID(30);

}

void skills() {
  // flywheel setup
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00005, 0.00005, 0, 0.004, 0.0001, 0);
  queueDiscs(0, 2300, 0.4);

  // first roller
  spinLeft(15);
  spinRight(15);
  wait(400, msec);
  Intake_Roller.spinFor(0.75, rotationUnits::rev, 100, velocityUnits::pct, false);
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
  wait(2000, msec);

  Intake_Roller.spinFor(0.75, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(5, sec);
  FlywheelUp.stop(brake);
  FlywheelDown.stop(brake);
  drivePID(-5);
  Intake_Roller.spin(reverse, 100, pct);
  queueDiscs(5);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  Turn(-130, 100);
  StringShooters.set(true);
  wait(10, sec);
  

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
  vex::timer t = vex::timer();
  t.reset();
  while (numQueued() > 0 && t.time() < 3.5) {
    wait(10, msec);
  }
  flywheelOn.stop();

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
  runTime.reset();
  vex::task keepTime = vex::task(displayTime);
  // flywheel setup

  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
 adjustFPID(0.00007, 0.00005, 0, 0.004, 0.00001, 0);
  queueDiscs(0, 2500, 0.3);

  // first roller
  spinLeft(15);
  spinRight(15);
  wait(400, msec);
  Intake_Roller.spinFor(0.3, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(500, msec);

  // align to shoot
  drivePID(-4);
  Turn(-10, 100);

  // shoot
  wait(100, msec);
  queueDiscs(2, 2500, 0.3);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  wait(200, msec);
  queueDiscs(0, 2300, 0.3);
  drivePID(2);
  Turn(-133, 100);
  vex::task intake = vex::task(intake3);

  drivePID(66, 11);
  Turn(-45, 100);
  wait(100, msec);
  queueDiscs(3, 2300, 0.3);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  intake.stop();
  vex::task intake2 = vex::task(intake3);
  wait(200, msec);
  queueDiscs(0, 2500, 0.3);
  Turn(-132, 100);
  drivePID(67, 12);
  spinLeft(100);
  FRDrive.stop(coast);
  BRDrive.stop(coast);
  intake2.stop();
  Intake_Roller.stop();
  while (getInertialReading() < -110) {
      wait(10, msec);
  }

  spinBase(30, 30);
  wait(200, msec);
  Intake_Roller.spinFor(0.3, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(0.4, sec);
  stopBase(coast);
  drivePID(-2);
  Intake_Roller.spin(reverse, 100, pct);
  Turn(-84, 100);
  queueDiscs(3, 2500, 0.3);
  while(numQueued() > 0){
    wait(10, msec);
  }
}

void testing() {
  /*
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
  adjustFPID(0.00007, 0.00005, 0, 0.004, 0.00001, 0);
  queueDiscs(0, 2700, 0.4); // 2300
  Intake_Roller.spin(reverse, 100, pct);
  wait(4, sec);
  while (true) {
    queueDiscs(3, 2500, 0.4); // 2300
    while (numQueued() > 0) {
      wait(10, msec);
    }
    wait(10, sec);
  }

  wait(30, sec);
  printf("dsfakjdlkj:%d\n", 5);
  /*leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  while(true){
    printf("left = %f, right = %f\n", leftEncoder.position(deg), rightEncoder.position(deg));
  }*/
}