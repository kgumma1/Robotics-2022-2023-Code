#include "vex.h"

using namespace vex;

void displayInfo() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(mono20);
  Brain.Screen.printAt(5, 20, "MOTOR TEMPERATURES");
  Brain.Screen.printAt(5, 40, "RightFront:       %.0f", RFDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 60, "RightMiddle:      %.0f", RMDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 80, "RightRear:        %.0f", RBDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 100, "LeftFront:        %.0f", LFDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 120, "LeftMiddle:       %.0f", LMDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 140, "LeftRear:         %.0f", LBDrive.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 160, "Intake/Roller:    %.0f", intake_roller.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 180, "Flywheel:         %.0f", flywheel.temperature(vex::temperatureUnits::celsius));
}

void matchLoadTest() {
  while (true) {
    if (Controller.ButtonL2.pressing()) {
      flywheel.spin(forward, Controller.Axis2.position(), pct);
    }
    intake_roller.spin(forward, 100, pct);
  }
}

double straightExpFunction(double d) {
  double a = 4;
  return (1 / pow(100, a-1)) * pow(fabs(d), a) * (d > 0 ? 1 : -1);
}

double turnExpFunction(double d) {
  double a = 8;
  return (1 / pow(100, a-1)) * pow(fabs(d), a) * (d > 0 ? 1 : -1);
}

bool discAtBottom(int threshold = 8) {
  return bottomIntakeSensor.reflectivity() - threshold > bottomIntakeSensorInit;
}

bool discAtTop(int threshold = 4) {
  return topIntakeSensor.reflectivity() - threshold > topIntakeSensorInit;
}

bool discAtFlywheel(int threshold = 4) {
  return flywheelSensor.reflectivity() - threshold > flywheelSensorInit;
}

void drive() {

  bool shooting = false;
  bool expanded = false;
  bool expanding = false;

  bool r1prev = false;
  bool r2prev = false;
  bool yprev = false;
  
  int intRollSpeed = 0;

  vex::timer basketDelayTimer = vex::timer();
  double basketDelay = 0;

  vex::timer compressionTimer = vex::timer();
  double compressionDelay = 0.2;

  vex::timer shotTimer = vex::timer();
  vex::timer exp = vex::timer();


  int discCount = 0;
  bool intaking = false;
  bool exiting = false;

  while (1) {
    displayInfo();

    // DRIVE //
    double initSens = 0.93;
    double sensInc = -0.00005;


    double Axis3Adjusted = fabs(Controller.Axis3.position()) > 5 ? straightExpFunction(Controller.Axis3.position()) : 0;
    double Axis1Adjusted = fabs(Controller.Axis1.position()) > 5 ? turnExpFunction(Controller.Axis1.position() * 0.93) : 0;


    double outputL = (Axis3Adjusted + (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    double outputR = (Axis3Adjusted - (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    //printf("VerticalAxis = %ld, HorizontalAxis = %ld, outputL = %f, outputR = %f\n", Controller.Axis3.position(), Controller.Axis1.position(), outputL, outputR);
    lDrive.spin(forward, (outputL / 100.0 * 12), volt);
    rDrive.spin(forward, (outputR / 100.0 * 12), volt);


    if (discAtBottom(8)) {
      intaking = true;
    }

    if (intaking && !discAtBottom(2)) {
      intaking = false;
      discCount++;
    }

    if (discAtBottom(10) && discCount >= 3) {
      intRollSpeed = 0;
    }

    if (discAtFlywheel(3)) {
      exiting = true;
    }

    if (exiting && !discAtFlywheel(2)) {
      exiting = false;
      discCount--;
      if (discCount < 0) {
        discCount = 0;
      }
    }

    printf("discs = %d, bottomDisc = %d, bottomReflectivity = %ld\n", discCount, discAtBottom(), bottomIntakeSensor.reflectivity());

    // FLYWHEEL // 
    double speedUpDelay = Controller.ButtonL2.pressing() ? 0.5 : (Controller.ButtonL1.pressing() ? 0.00 : 0.5);
    
    if ((flywheel.velocity(rpm) * 6 < 2350 || basketDelayTimer.value() > speedUpDelay) && !(expanded || expanding)) {
      flywheel.spin(forward, Controller.ButtonL2.pressing() ? 10 : 10, volt);
    } else if (!(expanded || expanding)) {
      flywheel.spin(forward, Controller.ButtonL2.pressing() ? 8.3 : 8.3, volt);
    }
    //printf("flywheel = %f\n", flywheel.velocity(rpm) * 6);


    

    if (Controller.ButtonL1.pressing()) {
      shooting = true;
      
      angleChanger.set(false);
     if (compressionTimer.value() > compressionDelay) {
        compressionBar.set(true);
      }
      
      if (basketDelayTimer.value() > basketDelay) {
        intake_roller.spin(reverse, 100, pct);
      } else {
        intake_roller.spin(forward, 100, pct);
      }

    } else if (Controller.ButtonL2.pressing()) {
      shooting = true;

      angleChanger.set(true);
      if (compressionTimer.value() > compressionDelay) {
        compressionBar.set(true);
      }

      if (basketDelayTimer.value() > basketDelay) {
        intake_roller.spin(reverse, 100, pct);
      } else {
        intake_roller.spin(forward, 100, pct);
      }

    } else {
      basketDelayTimer.reset();
      compressionTimer.reset();
      shooting = false;
      compressionBar.set(false);
    }



    // INTAKE/ROLLER //
    if (!shooting) {
      if (Controller.ButtonR1.pressing() && !r1prev) {
        if (intRollSpeed != 100) {
          if (discCount >= 3) {
            discCount = 2;
          }
          intRollSpeed = 100;
        } else {
          intRollSpeed = 0;
        }

      }

      if (Controller.ButtonR2.pressing() && !r2prev) {
        if (intRollSpeed != -100) {
          intRollSpeed = -100;
        } else {
          intRollSpeed = 0;
        }

      }

      intake_roller.spin(forward, intRollSpeed, pct);
    }

    if (Controller.ButtonX.pressing()) {
      intakeLift.set(true);
    } else {
      intakeLift.set(false);
    }


    // EXPANSION //
    if (Controller.ButtonY.pressing() && !yprev) {
      exp.reset();
    }
    //printf("val = %f\n", exp.value());
    if (Controller.ButtonY.pressing()) {
      if (exp.value() > 0.500) {
        expansion.set(true);
        expanded = true;
      }
      flywheel.stop(brake);
      expanding = true;
    } else {
      expanding = false;
    }



    r1prev = Controller.ButtonR1.pressing();
    r2prev = Controller.ButtonR2.pressing();
    yprev = Controller.ButtonY.pressing();

    wait(10, msec);
  }
}
