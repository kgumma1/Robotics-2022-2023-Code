#include "vex.h"
#include "autonFunctions.h"

using namespace vex;


void testing() {
  flywheel.spin(forward, 12, volt);
  basket.set(true);
  while(1) {
    if (Controller.ButtonL1.pressing()) {
      intake_roller.spin(reverse, 5, pct);
    } else {
      intake_roller.stop(coast);
    }
  }
}
