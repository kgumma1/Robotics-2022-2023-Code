#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

vex::timer runTime = vex::timer();
int matchTime = 15;
int displayTime() {
  Brain.Screen.setFont(mono60);
  while (runTime.value() < matchTime) {
    Brain.Screen.printAt(5, 50, "Time Left=%f", matchTime - runTime.value());
    wait(100, msec);
    Brain.Screen.clearScreen();
  }
  Brain.Screen.setPenColor(red);
  Brain.Screen.printAt(5, 50, "Time Left=%f", 0.0);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(5, 100, 200, 300);
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
  runTime.reset();
  matchTime = 60;
  vex::task keepTime = vex::task(displayTime);
  // flywheel setup
  
  vex::task startCounter = vex::task(countDiscs);

  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00007, 0.00005, 0, 0.004, 0.00001, 0);
  angleChanger.set(true);

  // preloads
  queueDiscs(0, 1900, 0.25);
  Turn(5, 100);
  queueDiscs(2, 1900, 0.25);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  wait(100, msec);
  angleChanger.set(false);
  queueDiscs(0, 2000, 0.25);

  // move to/intake first line
  Turn(-32, 100);

  vex::task intake = vex::task(maintain3);
  drivePID(32.5);
  Turn(-135, 100);

  drivePID(35);
  Turn(-50, 100);
  wait(200, msec);

  // shoot first line
  queueDiscs(3, 2000, 0.25);
  while(numQueued() > 0){
    wait(10, msec);
    changeCount(numQueued());
  }
  changeCount(0);

  wait(100, msec);
  queueDiscs(0, 2070, 0.25);

  // move to/intake alliance 3 stack
  Turn(-131, 100);
  drivePID(49, 6.5);
  drivePID(-19);
  Turn(-77, 100);
  wait(100, msec);

  // shoot alliance 3-stack
  queueDiscs(3);
  while(numQueued() > 0){
    wait(10, msec);
    changeCount(numQueued());
  }
  changeCount(0);
  wait(100, msec);
  queueDiscs(0, 2400, 0.25);

  // move to / intake center 3-stack
  Turn(-91, 100);
  drivePID(32, 8);
  drivePID(16, 5, 5);

  // roller 1 manip
  wait(150, msec);
  spinBase(30, 30);
  intake.suspend();
  Intake_Roller.stop(coast);
  wait(500, msec);

  Intake_Roller.spinFor(-0.8, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(1500, msec);
  drivePID(-18);
  Turn(-180, 100);

  // shoot excess disc
  wait(100, msec);
  queueDiscs(1);
  while(numQueued() > 0){
    wait(10, msec);
  }
  changeCount(2);
  wait(100, msec);

  // intake excess disc + roller 2 manip
  queueDiscs(0, 2200, 0.25);
  intake.resume();
  drivePID(20, 8);
  wait(150, msec);
  FLDrive.spin(forward, 6, volt);
  FRDrive.spin(forward, 6, volt);
  BRDrive.spin(forward, 6, volt);
  BLDrive.spin(forward, 6, volt);
  intake.suspend();
  Intake_Roller.stop(coast);
  wait(500, msec);

  Intake_Roller.spinFor(-0.8, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(1500, msec);

  // align and move to shoot center 3 stack
  drivePID(-2);
  intake.resume();
  changeCount(3);
  Turn(177, 100);
  drivePID(-35);
  queueDiscs(3);
  while (numQueued() > 0) {
    wait(10, msec);
    changeCount(numQueued());
  }
  changeCount(0);
  wait(100, msec);
  queueDiscs(0, 2000, 0.25);

  // align/intake 2nd line
  Turn(48, 100);
  drivePID(47);
  Turn(136, 100);
  wait(100, msec);

  // shoot 2nd line
  queueDiscs(3);
  while(numQueued() > 0){
    wait(10, msec);
    changeCount(numQueued());
  }
  changeCount(0);
  wait(100, msec);

  queueDiscs(0, 2070, 0.25);

  // intake opponent 3 stack
  Turn(49, 100);
  drivePID(52, 6.5);
  drivePID(-19);
  Turn(107, 100);
  wait(100, msec);

  // shoot opponent 3 stack
  queueDiscs(3);
  while(numQueued() > 0){
    wait(10, msec);
    changeCount(numQueued());
  }
  changeCount(0);
  wait(100, msec);
  queueDiscs(0, 2400, 0.25);

  // intake center 3 stack #2 + roller 3 manip
  Turn(88, 100);
  drivePID(32, 8);
  drivePID(16, 5, 5);
  wait(150, msec);
  spinBase(30, 30);
  intake.suspend();
  Intake_Roller.stop(coast);
  wait(500, msec);

  Intake_Roller.spinFor(-0.8, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(1500, msec);
  drivePID(-18);
  Turn(0, 100);
  wait(100, msec);

  // shoot excess disc
  queueDiscs(1);
  while(numQueued() > 0){
    wait(10, msec);
  }
  changeCount(2);

  // roller manip + intake excess disc
  wait(100, msec);
  queueDiscs(0, 2250, 0.25);
  intake.resume();
  drivePID(20, 8);
  wait(150, msec);
  FLDrive.spin(forward, 6, volt);
  FRDrive.spin(forward, 6, volt);
  BRDrive.spin(forward, 6, volt);
  BLDrive.spin(forward, 6, volt);
  intake.suspend();
  Intake_Roller.stop(coast);
  wait(500, msec);

  Intake_Roller.spinFor(-0.8, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(1500, msec);
  drivePID(-2);

  // align and shoot center 3 stack #2
  Turn(-4, 100);
  drivePID(-16);
  queueDiscs(3);
  while (numQueued() > 0) {
    wait(10, msec);
    changeCount(numQueued());
  }
  changeCount(0);
  intake.resume();
  wait(100, msec);
  queueDiscs(0, 2100, 0.25);

  // turn and intake center line
  Turn(-128, 100);
  drivePID(70);
  Turn(137, 100);
  wait(100, msec);

  // shoot center line
  queueDiscs(3);
  while (numQueued() > 0) {
    wait(10, msec);
    changeCount(numQueued());
  }
  changeCount(0);
  wait(100, msec);

  // go to expand
  Turn(-131, 100);
  flywheelOn.stop();
  FlywheelDown.stop(brake);
  FlywheelUp.stop(brake);
  drivePID(60);
  StringShooters.set(true);

  wait(30, sec);
  

}

void matchFarRoller() {
  runTime.reset();
  vex::task keepTime = vex::task(displayTime);
  // flywheel setup

  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00007, 0.00005, 0, 0.004, 0.00001, 0);
  queueDiscs(0, 2500, 0.25);
  changeCount(2);
  vex::task intake1 = vex::task(intake3);
  drivePID(31);
  wait(100, msec);
  drivePID(-8);
  Turn(90, 100);
  intake1.stop();
  Intake_Roller.stop(coast);
  spinLeft(30);
  spinRight(30);
  wait(400, msec);
  Intake_Roller.spinFor(0.3, rotationUnits::rev, 100, velocityUnits::pct, false);
  wait(500, msec);
  drivePID(-3);
  Turn(93, 100);

  Intake_Roller.spin(reverse, 100, pct);
  queueDiscs(3, 2500, 0.25);
  while (numQueued() > 0) {
    wait(10, msec);
  }

  wait(100, msec);
  queueDiscs(0, 2300, 0.25);
  vex::task intake2 = vex::task(intake3);

  Turn(-142, 100);
  drivePID(65);
  Turn(127, 100);
  wait(100, msec);
  intake2.stop();
  Intake_Roller.spin(reverse, 100, pct);
  queueDiscs(3, 2300, 0.25);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  wait(100, msec);
  vex::task intake_3 = vex::task(intake3);
  Turn(120, 100);
  drivePID(40, 9);
  drivePID(-40);
  Turn(124, 100);
  intake_3.stop();
  Intake_Roller.spin(reverse, 100, pct);
  queueDiscs(3, 2300, 0.25);
  while (numQueued() > 0) {
    wait(10, msec);
  }

  










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
  queueDiscs(0, 2300, 0.35);
  drivePID(2.5);
  Turn(-133, 100);
  vex::task intake = vex::task(intake3);

  drivePID(66, 11);
  Turn(-47, 100);
  wait(100, msec);
  queueDiscs(3, 2300, 0.35);
  while (numQueued() > 0) {
    wait(10, msec);
  }
  intake.stop();
  vex::task intake2 = vex::task(intake3);
  wait(100, msec);
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
  /*
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00007, 0.00005, 0, 0.004, 0.00001, 0);
  queueDiscs(0, 2300, 0.35); // 2300
  Intake_Roller.spin(reverse, 100, pct);
  wait(5, sec);
  while (true) {
    queueDiscs(3, 2300, 0.35); // 2300
    while (numQueued() > 0) {
      wait(10, msec);
    }
    wait(10, sec);
  }*/
  resetDiscCount();
  vex::task flywheelOn = vex::task(flywheelPID);
  adjustFPID(0.00007, 0.00005, 0, 0.004, 0.00001, 0);
    queueDiscs(0, 2000, 0.25);
    wait(5, sec);
      queueDiscs(2, 2000, 0.25);
  while(numQueued() > 0){
    changeCount(numQueued());
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