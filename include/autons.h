#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

Point convertToDisplay(Point p) {
  return Point(240 + p.x / 144 * 240, 240 - (p.y / 144.0 * 240));
}

Point findControlPoint(Point anchor, double angle, double adherence) {
  angle = angle * M_PI / 180;
  return Point(adherence * cos(-angle + M_PI / 2) + anchor.x, adherence * sin(-angle + M_PI / 2) + anchor.y);
}

void testing() {

  globalX = 0;
  globalY = 0;
  vex::task track = vex::task(startTracking);
      double initSens = 0.93;
    double sensInc = -0.0001;
    wait(1, sec);
  move(3, 10, State(10, 10, 30, 10), State(Point(30, 20), 70, 20), State(Point(100, 1), 150, 19, 15));


  /*
  //printf("x: %f, y: %f, angle: %f, adherence: %f, speed: %f\n", ss.location.x, ss.location.y, ss.angle, ss.adherence, ss.speed);
  Bezier b = Bezier(ss, ss);
  while(true){
    displayTracking();
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

    
    wait(50, msec);
  }*/
}






vex::task runWithDelay(int (*callback)(), double timeMsec) {
  wait(timeMsec, msec);
  vex::task runFunction = vex::task(callback);
  return runFunction;
}
