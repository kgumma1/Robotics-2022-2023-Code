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

double straightExpFunction(double d) {
  double a = 4;
  return (1 / pow(100, a-1)) * pow(fabs(d), a) * (d > 0 ? 1 : -1);
}

double turnExpFunction(double d) {
  double a = 6;
  return (1 / pow(100, a-1)) * pow(fabs(d), a) * (d > 0 ? 1 : -1);
}

void drive() {

  bool shooting = false;

  bool r1prev = false;
  bool r2prev = false;
  
  int intRollSpeed = 0;

  vex::timer basketDelayTimer = vex::timer();
  double basketDelay = 0.1;

  vex::timer shotTimer = vex::timer();

  while (1) {
    displayInfo();

    // DRIVE //
    double initSens = 0.7;
    double sensInc = -0.0001;


    double Axis3Adjusted = fabs(Controller.Axis3.position()) > 5 ? straightExpFunction(Controller.Axis3.position()) : 0;
    double Axis1Adjusted = fabs(Controller.Axis1.position()) > 5 ? turnExpFunction(Controller.Axis1.position() * 0.9) : 0;


    double outputL = (Axis3Adjusted + (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    double outputR = (Axis3Adjusted - (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    //printf("VerticalAxis = %ld, HorizontalAxis = %ld, outputL = %f, outputR = %f\n", Controller.Axis3.position(), Controller.Axis1.position(), outputL, outputR);
    lDrive.spin(forward, (outputL / 100.0 * 12), volt);
    rDrive.spin(forward, (outputR / 100.0 * 12), volt);




    // FLYWHEEL // 
    if (flywheel.velocity(rpm) * 6 < 2450 || basketDelayTimer.value() > 0.3) {
      flywheel.spin(forward, 12, volt);
    } else {
      flywheel.spin(forward, 8.25, volt);
    }
    printf("flywheel = %f\n", flywheel.velocity(rpm) * 6);


    

    if (Controller.ButtonL1.pressing()) {
      shooting = true;
      
      angleChanger.set(false);
      basket.set(true);
      
      if (basketDelayTimer.value() > basketDelay) {
        intake_roller.spin(reverse, 100, pct);
      } else {
        intake_roller.spin(forward, 100, pct);
      }

    } else if (Controller.ButtonL2.pressing()) {
      shooting = true;

      angleChanger.set(true);
      basket.set(true);

      if (basketDelayTimer.value() > basketDelay) {
        intake_roller.spin(reverse, 100, pct);
      } else {
        intake_roller.spin(forward, 100, pct);
      }

    } else {
      basketDelayTimer.reset();
      shooting = false;
      basket.set(false);
    }



    // INTAKE/ROLLER //
    if (!shooting) {
      if (Controller.ButtonR1.pressing() && !r1prev) {
        if (intRollSpeed != 100) {
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



    r1prev = Controller.ButtonR1.pressing();
    r2prev = Controller.ButtonR2.pressing();

    wait(10, msec);
  }
}
