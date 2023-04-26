#include "vex.h"
#include "movement.h"

using namespace vex;


void intakeUp() {
  intakeLift.set(true);
}

void intakeDown() {
  intakeLift.set(false);  
}

void intake() {
  intake_roller_cata.spin(reverse, 100, pct);
}

void removeBands() {
  bandBoost.set(true);
}

int spinRoller(bool left, bool redAlliance = true, double timeout = 2000) {
  vex::timer rollerTimer = vex::timer();

  if (left) {
    while ((!leftRollerSensor.isNearObject()) && rollerTimer.time() < timeout) {
      wait(10, msec);
    }
    if (redAlliance) {
      while ((!(leftRollerSensor.hue() > 180 && leftRollerSensor.hue() < 300)) && rollerTimer.time() < timeout) {
        wait(10, msec);

      }
    } else {
      while ((!(leftRollerSensor.hue() > 330 || leftRollerSensor.hue() < 30)) && rollerTimer.time() < timeout) {
        wait(10, msec);

      }
    }
  
  } else {
    while ((!rightRollerSensor.isNearObject()) && rollerTimer.time() < timeout) {
      wait(10, msec);
    }
    if (redAlliance) {
      while ((!(rightRollerSensor.hue() > 180 && rightRollerSensor.hue() < 300)) && rollerTimer.time() < timeout) {
        wait(10, msec);
      }
    } else {
      while ((!(rightRollerSensor.hue() > 330 || rightRollerSensor.hue() < 30)) && rollerTimer.time() < timeout) {
        wait(10, msec);

      }
    }
  }
  
  return 1;
}

int discCount = 0;
bool cataReset = true;
bool cataFiring = false;
bool intakeEnabled = true;
bool pistonBoostEnabled = false;

void fireCata(bool pistonBoostOn = false, bool w = true) {
  cataFiring = discCount > 0;
  pistonBoostEnabled = pistonBoostOn;
  if (w) {
    while(discCount > 0){
      wait(10, msec);
    }

  }
}

void intakeControl(double intakeControlOn) {
  intakeEnabled = intakeControlOn;
}

bool discAtBottom(double threshold = 8) {
  return (bottomIntakeSensorInit + threshold < bottomIntakeSensor.reflectivity());
}

int countDiscs() {
  while (true) {

    while (!discAtBottom(8)) {
      wait(10, msec);
      if (cataSensor.velocity(dps) < -30) {discCount = 0;}
    }

    while (discAtBottom(2)) {
      wait(10, msec);
      if (cataSensor.velocity(dps) < -30) {discCount = 0;}
    }

    discCount++;
  }
}



int manageCata_Intake() {
  while(true){
    double cataAngle = cataSensor.angle(deg);
    if (cataAngle > resetAngle && cataAngle < 350 && cataFiring) {
      cataMain.spin(forward, 12, volt);
      intake_roller_cata.spin(forward, 12, volt); 
      cataReset = false;
  
    } else if (!cataFiring && cataSensor.velocity(dps) < 0.0001 && cataAngle < resetAngle2 && cataReset) {
      cataReset = false;
      resetAngle = resetAngle2;
      cataMain.spin(forward, 12, volt);
      intake_roller_cata.spin(forward, 12, volt);
    } else if (cataAngle > resetAngle && cataAngle < 350) {
      cataMain.stop(brake);
      intake_roller_cata.stop(brake);
      cataReset = true;
    } else if (cataAngle < resetAngle || cataAngle > 350) {
      cataMain.spin(forward, 12, volt);
      intake_roller_cata.spin(forward, 12, volt);
      cataReset = false;
      cataFiring = false;
    }
    if (cataSensor.velocity(dps) <= -30) {
      resetAngle = resetAngle2 - resetAngleDiff;
    }


    if (cataFiring && cataAngle > 1.5 + resetAngle2 && pistonBoostEnabled) {
      pistonBoost.set(true);
    } else if (!cataReset && !cataFiring && cataAngle < 10) {
      pistonBoost.set(false);
    }
    if (cataReset && discCount < 4 && intakeEnabled) {
      intake_roller_cata.spin(reverse, 100, pct);
    } else if (intakeEnabled && !cataFiring && cataReset) {
      intake_roller_cata.stop(coast);
    }
    //printf("angle: %f, r: %f, cataReset: %d, cataFiring: %d\n", cataAngle, resetAngle, cataReset ? 1 : 0, cataFiring ? 1 : 0);
    //printf("rangle %f\n", resetAngle);
    //printf("discs: %d, cataReset: %d, intakeV: %f, cataV: %f\n", discCount, cataReset ? 1 : 0, intake_roller_cata.voltage(), cataMain.voltage());
    wait(5, msec);
  }
}



