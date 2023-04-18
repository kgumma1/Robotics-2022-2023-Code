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
  Brain.Screen.printAt(5, 160, "Intake/Roller/Cata:    %.0f", intake_roller_cata.temperature(vex::temperatureUnits::celsius));
  Brain.Screen.printAt(5, 180, "Cata:         %.0f", cataMain.temperature(vex::temperatureUnits::celsius));
}


double straightExpFunction(double d, double a = 4) {
  return (1 / pow(100, a-1)) * pow(fabs(d), a) * (d > 0 ? 1 : -1);
}

double turnExpFunction(double d, double a = 8) {
  return (1 / pow(100, a-1)) * pow(fabs(d), a) * (d > 0 ? 1 : -1);
}

bool cataFired(double resetAngle, double cataAng) {
  return (cataAng < resetAngle || cataAng > 350);
}

/*==
bool discAtBottom(int threshold = 8) {
  return bottomIntakeSensor.reflectivity() - threshold > bottomIntakeSensorInit;
}*/


void drive() {

  bool shooting = false;
  bool expanded = false;
  bool expanding = false;

  bool r1prev = false;
  bool r2prev = false;
  bool yprev = false;
  
  int intRollSpeed = 0;

  
  vex::timer exp = vex::timer();


  int discCount = 0;
  bool intaking = false;
  bool resettingCata = false;
  bool cataFiring = false;
  bool pistonActive = false;
  
  double cataAng;



  while (1) {
    displayInfo();

    // DRIVE //
    double initSens = 0.93;
    double sensInc = -0.00005;


    double Axis3Adjusted = fabs(Controller.Axis3.position()) > 5 ? straightExpFunction(Controller.Axis3.position()) : 0;
    double turnAxis = 0;
    //printf("4V: %ld, 1V: %ld\n", Controller.Axis4.position(), Controller.Axis1.position());
    if (abs(Controller.Axis4.position()) > 10 && abs(Controller.Axis3.position()) < 25) {
      turnAxis = abs(Controller.Axis4.position()) > 5 ? turnExpFunction(Controller.Axis4.position() * 0.85) : 0;
      //printf("T: %f, S: %f\n", turnAxis, Axis3Adjusted);
    } else {
      turnAxis = abs(Controller.Axis1.position()) > 5 ? turnExpFunction(Controller.Axis1.position() * 0.93) : 0;
      //printf("T: %f, S: %f\n", turnAxis, Axis3Adjusted);

    }
  

    


    double outputL = (Axis3Adjusted + (turnAxis * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    double outputR = (Axis3Adjusted - (turnAxis * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    printf("sraw = %ld, tfraw = %ld, tsraw = %ld\n", Controller.Axis3.position(), Controller.Axis1.position(), Controller.Axis4.position());
    printf("VerticalAxis = %f, HorizontalAxis = %f, outputL = %f, outputR = %f\n", Axis3Adjusted, turnAxis, outputL, outputR);
    lDrive.spin(forward, (outputL / 100.0 * 12), volt);
    rDrive.spin(forward, (outputR / 100.0 * 12), volt);


    // intake/cata
    cataAng = cataSensor.angle();

    if (Controller.ButtonR2.pressing() && !r2prev) {
      intaking = !intaking;
      if (intaking && !resettingCata && !cataFiring) {
        intake_roller_cata.spin(reverse, 100, pct);
      }
      if (!intaking && !resettingCata && !cataFiring) {
        intake_roller_cata.spin(reverse, 0, pct);
      }
    }
    if (intaking && !resettingCata && !cataFiring) {
      intake_roller_cata.spin(reverse, 100, pct);
    }

    if (Controller.ButtonL2.pressing()) {
      pistonActive = true;
    }

    if (Controller.ButtonL1.pressing()) {
      pistonActive = false;
    }


    if (cataFired(resetAngle, cataAng) && !resettingCata /*&& fabs(cataSensor.velocity(dps)) < 2*/ && !expanding && !expanded) {


      resettingCata = true;
      cataFiring = false;
      intake_roller_cata.spin(forward, 100, pct);
      cataMain.spin(forward, 100, pct);
    }
    if (!cataFiring) {
      pistonBoost.set(false);
    }

    if (resettingCata && cataAng > resetAngle && cataAng < 350) {
      cataMain.stop(brake);
      intake_roller_cata.stop(brake);
      resettingCata = false;
    }

    if (!resettingCata && cataAng >= resetAngle && (Controller.ButtonL1.pressing() || Controller.ButtonL2.pressing())) {
      cataMain.spin(forward, 100, pct);
      intake_roller_cata.spin(forward, 100, pct);
 

      cataFiring = true;
    }

    if (expanding && cataAng >= resetAngle) {
      cataMain.spin(forward, 100, pct);
      intake_roller_cata.spin(forward, 100, pct);
 

      cataFiring = true;
    } else if (expanding) {
      cataFiring = false;
      cataMain.stop(coast);
      intake_roller_cata.stop(coast);
    }

    if (cataSensor.velocity(dps) < -1 && !resettingCata) {
  
      //cataMain.stop(brake); // TEST
      //intake_roller_cata.stop(brake); // TEST

    }
    if(cataSensor.velocity(dps) < 0.5 && !resettingCata && fabs(cataMain.voltage()) > 0.5 && !cataFiring) {
      cataMain.stop(coast);
      intake_roller_cata.stop(coast);
    }

    if (cataAng > resetAngle + 2 && cataFiring && pistonActive) {
      pistonBoost.set(true);
    }

    //printf("cataA: %f, resetting: %d \n", cataAng, resettingCata ? 1 : 0);
    // LIFT //
    if (Controller.ButtonA.pressing()) {
      intakeLift.set(true);
    } else {
      intakeLift.set(false);
    }


    // EXPANSION //
    if (Controller.ButtonY.pressing() && !yprev) {
      exp.reset();
    }
 
    if (Controller.ButtonY.pressing()) {
      if (exp.time() > 500 && exp.system() > 500) {
        if (!Controller.ButtonR1.pressing()) {
          expansionRight.set(true);
        }
        if (!Controller.ButtonL1.pressing()) {
          expansionLeft.set(true);
        }
        expanded = true;
      }
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
