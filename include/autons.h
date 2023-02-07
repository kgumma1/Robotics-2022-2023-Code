#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

Point findControlPoint(Point anchor, double angle, double adherence) {
  angle = angle * M_PI / 180;
  return Point(adherence * cos(-angle + M_PI / 2) + anchor.x, adherence * sin(-angle + M_PI / 2) + anchor.y);
}

void testing() {

  globalX = 0;
  globalY = 0;
    printf("WORKING%d\n", 1);
  vex::task track = vex::task(startTracking);
      double initSens = 0.93;
    double sensInc = -0.0001;
    wait(5, sec);
  printf("WORKING%d\n", 1);
  //move(forward, 3, 20, State(50, 30, 160, 30, 100), State(20, 30, 0, 15, 100), State(0, 70, -30, 40, 50));
  //move(reverse, 3, 40, State(20, 30, 0, 15, 20), State(50, 30, 160, 30, 20), State(0, 0, 0, 20, 20));
  move(reverse, 3, 20, State(50, 30, -20, 30, 20), State(20, 30, -180, 15, 20), State(0, 70, 150, 40, 20));  
  //printf("x: %f, y: %f, angle: %f, adherence: %f, speed: %f\n", ss.location.x, ss.location.y, ss.angle, ss.adherence, ss.speed);
  //Bezier b = Bezier(ss, ss);
  while(true){
    //displayTracking();
    /*
    b.display(30);
    Point x = Point(globalX, globalY);
    double t = b.closestPointTo(x);
    x = convertToDisplay(x);
    Point y = b.getValue(t);
    Point c = findControlPoint(y, b.getAngle(t), 90);
    Point d = findControlPoint(y, b.getAngle(t) + 180, 90);
    c = convertToDisplay(c);
    d = convertToDisplay(d);

    //printf("x = %f, y = %f, angle = %f\n", y.x, y.y, b.getAngle(t));
    y = convertToDisplay(y);

    Brain.Screen.drawLine(c.x, c.y, d.x, d.y);
    Brain.Screen.drawCircle(x.x, x.y, 5, red);
    Brain.Screen.drawCircle(y.x, y.y, 5, blue);
    Point z = Point(100, 100);
    z = convertToDisplay(z);
    Brain.Screen.drawCircle(z.x, z.y, 5, green);

    */
    wait(50, msec);
  }
}






vex::task runWithDelay(int (*callback)(), double timeMsec) {
  wait(timeMsec, msec);
  vex::task runFunction = vex::task(callback);
  return runFunction;
}
