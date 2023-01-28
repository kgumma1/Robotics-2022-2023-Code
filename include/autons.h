#include "vex.h"
#include "autonFunctions.h"

using namespace vex;



void testing() {
  globalX = 0;
  globalY = 0;
  vex::task track = vex::task(startTracking);
      double initSens = 0.93;
    double sensInc = -0.0001;
  //lDrive.spinFor(1, sec, 25, velocityUnits::pct);
  //rDrive.spinFor(1, sec, 25, velocityUnits::pct);
  while(true){
    displayTracking();/*
    printf("x = %f, y = %f\n", globalX, globalY);
    double Axis3Adjusted = fabs(Controller.Axis3.position()) > 5 ? straightExpFunction(Controller.Axis3.position()) : 0;
    double Axis1Adjusted = fabs(Controller.Axis1.position()) > 5 ? turnExpFunction(Controller.Axis1.position() * 0.9) : 0;


    double outputL = (Axis3Adjusted + (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    double outputR = (Axis3Adjusted - (Axis1Adjusted * fabs(sensInc * fabs(Axis3Adjusted) + initSens)));
    //printf("VerticalAxis = %ld, HorizontalAxis = %ld, outputL = %f, outputR = %f\n", Controller.Axis3.position(), Controller.Axis1.position(), outputL, outputR);
    lDrive.spin(forward, (outputL / 100.0 * 12), volt);
    rDrive.spin(forward, (outputR / 100.0 * 12), volt);*/
    

    wait(50, msec);
  }
}






vex::task runWithDelay(int (*callback)(), double timeMsec) {
  wait(timeMsec, msec);
  vex::task runFunction = vex::task(callback);
  return runFunction;
}
