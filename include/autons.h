#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

  Point convertToDisplay(Point p) {
    return Point(240 + p.x / 144 * 240, 240 - (p.y / 144.0 * 240));
  }

void testing() {

  globalX = 0;
  globalY = 0;
  vex::task track = vex::task(startTracking);
      double initSens = 0.93;
    double sensInc = -0.0001;
  //lDrive.spinFor(1, sec, 25, velocityUnits::pct);
  //rDrive.spinFor(1, sec, 25, velocityUnits::pct);
  Bezier b = Bezier(Pose(10, 10, -50, 70), Pose(100, 100, 100, 60));
  while(true){
    displayTracking();
    b.display(30);
    Point x = Point(110, 112);
    double t = b.closestPointTo(x);
    x = convertToDisplay(x);
    Point y = b.getValue(t);
    printf("x = %f, y = %f\n", y.x, y.y);
    y = convertToDisplay(y);


    Brain.Screen.drawCircle(x.x, x.y, 5, red);
    Brain.Screen.drawCircle(y.x, y.y, 5, blue);
    Point z = Point(100, 100);
    z = convertToDisplay(z);
    Brain.Screen.drawCircle(z.x, z.y, 5, green);
 



    /*
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
