#include "vex.h"

using namespace vex;

void displayInfo() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(mono20);
  Brain.Screen.printAt(5, 20, "MOTOR TEMPERATURES");
  Brain.Screen.printAt(5, 40, "LeftFront:      %.0f", FLDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 60, "LeftRear:       %.0f", BLDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 80, "RightFront:     %.0f", FRDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 100, "RightRear:      %.0f", BRDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 120, "Roller/Intake:  %.0f", Intake_Roller.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 140, "TopFW:          %.0f", FlywheelUp.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 160, "BottomFW:       %.0f", FlywheelDown.temperature(vex::temperatureUnits::celsius));
}

int aimSpeed = 10;
int lowBound = 178;
int highBound = 183;
int cXs[10];

int autoAimMacro() {
  while (true) {
    Vision.takeSnapshot(Vision__SIG_1);
    
    if (Vision.largestObject.exists) {
      if (Vision.largestObject.centerX < lowBound) {
        FRDrive.spin(forward, aimSpeed, pct);
        BRDrive.spin(forward, aimSpeed, pct);
        FLDrive.spin(reverse, aimSpeed, pct);
        BLDrive.spin(reverse, aimSpeed, pct);
      } else if (Vision.largestObject.centerX > highBound) {
        FRDrive.spin(reverse, aimSpeed, pct);
        BRDrive.spin(reverse, aimSpeed, pct);
        FLDrive.spin(forward, aimSpeed, pct);
        BLDrive.spin(forward, aimSpeed, pct);
      } else {
        FRDrive.stop(hold);
        BRDrive.stop(hold);
        FLDrive.stop(hold);
        BLDrive.stop(hold);
      }
    }
    printf("Object1 = %d, x = %d, y = %d\n", Vision.largestObject.exists, Vision.largestObject.centerX, Vision.largestObject.centerY);
    /*
    Vision.takeSnapshot(Vision__SIG_2);
    
    if (Vision.largestObject.exists) {
      if (Vision.largestObject.centerX < lowBound) {
        FRDrive.spin(forward, aimSpeed, pct);
        BRDrive.spin(forward, aimSpeed, pct);
        FLDrive.spin(reverse, aimSpeed, pct);
        BLDrive.spin(reverse, aimSpeed, pct);
      } else if (Vision.largestObject.centerX > highBound) {
        FRDrive.spin(reverse, aimSpeed, pct);
        BRDrive.spin(reverse, aimSpeed, pct);
        FLDrive.spin(forward, aimSpeed, pct);
        BLDrive.spin(forward, aimSpeed, pct);
      } else {
        FRDrive.stop(hold);
        BRDrive.stop(hold);
        FLDrive.stop(hold);
        BLDrive.stop(hold);
      }
    }
    printf("Object2 = %d, x = %d, y = %d\n", Vision.largestObject.exists, Vision.largestObject.centerX, Vision.largestObject.centerY);
*/
    wait(100, msec);
  }


}


void drive() {
    // User control code here, inside the loop

  bool xPrev = false;
  bool bPrev = false;
  bool aPrev = false;
  bool yPrev = false;

  bool l1Prev = false;
  bool l2Prev = false;
  bool flyOn = true;

  bool r1Prev = false;
  bool r2Prev = false;
  bool intRollOn = false;
  int intRollSpeed = 0;
  int discCount = 0;
  bool intakingDisc = false;

  int flywheelSpeed = 600;
  int motorSpeed = 0;

  double waitTime = 0.17;
  double recoverTime = 0.25;

    flywheelSpeed = 1800; // 300y
    motorSpeed = 170;

  Puncher.setStopping(hold);
  
  
  angleChanger.set(true);

  vex::timer exp = vex::timer();
  vex::timer fly = vex::timer();
  vex::timer ind = vex::timer();
  vex::timer sen = vex::timer();

  vex::task autoAim = vex::task(autoAimMacro);

  bool aiming = false;

  autoAim.suspend();

  while (1) {
    
    displayInfo();
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    // DRIVE
    if (Controller1.ButtonL1.pressing() && !l1Prev) {
      aiming = true;
      autoAim.resume();
    }
    
    if (abs(Controller1.Axis1.position()) > 5 || abs(Controller1.Axis3.position()) > 5) {
      aiming = false;
      autoAim.suspend();
    }
    
    if (!aiming) {
      double initSens = 0.8;
      double sensInc = -0.0001;


      double Axis3Adjusted = Controller1.Axis3.position();
      double Axis1Adjusted = Controller1.Axis1.position() * 0.75;


      double outputL = (Axis3Adjusted + (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
      double outputR = (Axis3Adjusted - (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
      
      FLDrive.spin(forward, outputL, pct);
      BLDrive.spin(forward, outputL, pct);
      FRDrive.spin(forward, outputR, pct);
      BRDrive.spin(forward, outputR, pct);
    }

    

    

    // FLYWHEEL
    /*
    if (Controller1.ButtonUp.pressing()) {
      flywheelSpeed = 2500; // 
      motorSpeed = 340;
    } else if (Controller1.ButtonDown.pressing()) {
      flywheelSpeed = 2000; // 330
      motorSpeed = 260;
    }*/
    /*
    if (Controller1.ButtonL1.pressing() && !l1Prev) {
      flywheelSpeed = 1500; // 300
      motorSpeed = 210;
      if (!flyOn) {
        flyOn = true;
      } else {
       // FlywheelDown.stop(coast);
       // FlywheelUp.stop(coast);
      //  flyOn = false;
      }
    }*/
    

    if (flyOn) {
      double avgSpeed = (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2;
      //printf("speed = %f\n", avgSpeed * 7);
      if (motorSpeed - avgSpeed > 20 || (fly.value() > 0.05 && fly.value() < recoverTime && fly.system() > 0.2)) {
        FlywheelDown.spin(forward, 11, volt);
        FlywheelUp.spin(forward, 11, volt);
      } else {
        FlywheelDown.spin(forward, (flywheelSpeed / 7.0) / 50, volt);
        FlywheelUp.spin(forward, (flywheelSpeed / 7.0) / 50, volt);
      }

    }



    //printf("topM   : %f\n", FlywheelUp.velocity(pct));
    //printf("BottomM: %f\n", FlywheelDown.velocity(pct));

    // TRANSMISSION
    if (Controller1.ButtonX.pressing() && !xPrev) {
      trans.set(!trans.value());
    }
    double intakeSensorInit1 = 1;
    // INTAKE/ROLLER
    if (IntakeSensor.reflectivity() >= intakeSensorInit + 5 && !intakingDisc && intakeSensorInit != 0) {
      discCount++;
      intakingDisc = true;
    }
    if (IntakeSensor.reflectivity() >= intakeSensorInit + 6) {
      sen.reset();
    }
    if (IntakeSensor.reflectivity() <= intakeSensorInit + 2) {
      if (intakingDisc) {
        sen.reset();
      }
      intakingDisc = false;
    }
    //printf("Sensor = %ld : Init = %f : discCount %d\n", IntakeSensor.reflectivity(), intakeSensorInit, discCount);
    if (discCount >= 3 && sen.value() > 0.5) {
      discCount = discCount >= 0 ? discCount : 0;
      intRollOn = false;
      intRollSpeed = 0;
    }

    if (Controller1.ButtonR1.pressing() && !r1Prev) {
      if (intRollSpeed != -100) {
        if (discCount > 2) {
          discCount--;
          discCount = discCount >= 0 ? discCount : 0;
        }
        intRollOn = true;
        intRollSpeed = -100;
      } else {
        intRollOn = false;
        intRollSpeed = 0;
      }
    }

    if (Controller1.ButtonR2.pressing() && !r2Prev) {
      if (intRollSpeed != 100) {
        intRollOn = true;
        intRollSpeed = 100;
      } else {
        intRollOn = false;
        intRollSpeed = 0;
      }
    } 

    if (intRollOn) {
      Intake_Roller.spin(forward, intRollSpeed, pct);
    } else {
      Intake_Roller.stop(coast);
    }

    //INDEXER
    if (Controller1.ButtonL2.pressing() && !l2Prev) {
      fly.reset();
    }
    if(Controller1.ButtonL2.pressing()) {
        if (fly.value() > waitTime) {
          Indexer.set(!Indexer.value());
          if (Indexer.value() == false) {
            discCount--;
            discCount = discCount >= 0 ? discCount : 0;
          }
          fly.reset();
        }
    } else {
      Indexer.set(false);
    }

  
 
/*
    // PUNCHER 
    if (Controller1.ButtonB.pressing()) {
      Puncher.spinFor((2200 - abs((int)(Puncher.position(degrees)) % 2200)) * -1, rotationUnits::deg, 100, velocityUnits::pct, false);
    }
    if (Controller1.ButtonA.pressing()) {
      Puncher.spin(reverse, 100, pct);
    }
    if (!Controller1.ButtonA.pressing() && aPrev) {
      Puncher.stop();
    }
    */

    // EXPANSION

    if (Controller1.ButtonY.pressing() && !yPrev) {
      exp.reset();
    }
    //printf("val = %f\n", exp.value());
    if (Controller1.ButtonY.pressing()) {
      if (exp.value() > 0.500) {
        StringShooters.set(true);
      }
      FlywheelDown.stop(brake);
      FlywheelUp.stop(brake);
      flyOn = false;
    }

    xPrev = Controller1.ButtonX.pressing();
    l1Prev = Controller1.ButtonL1.pressing();
    l2Prev = Controller1.ButtonL2.pressing();
    r1Prev = Controller1.ButtonR1.pressing();
    r2Prev = Controller1.ButtonR2.pressing();
    aPrev = Controller1.ButtonA.pressing();
    yPrev = Controller1.ButtonY.pressing();


    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}