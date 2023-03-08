#include "vex.h"

using namespace vex;

#define sideWheelCirc 2.73 * M_PI 
#define backWheelCirc 2.78 * M_PI 

#define sideDist 1.75
#define backDist 4.5
#define errorPerRotation 2.5


double globalX;
double globalY;
double globalAngle;

double initHeading = 0;

void displayTracking() {
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(5, 20, "X: %.3f", globalX);
  Brain.Screen.printAt(5, 40, "Y: %.3f", globalY);
  Brain.Screen.printAt(5, 60, "0: %.3f", globalAngle);
  Brain.Screen.printAt(5, 80, "L: %.3f", leftEncoder.position(deg));
  Brain.Screen.printAt(5, 100, "B: %.3f", backEncoder.position(deg));
  Brain.Screen.drawRectangle(240, 0, 240, 240, color(150, 150, 150));
  for (int i = 1; i < 6; i++) {
    Brain.Screen.drawLine(240 + 24 * i / 144.0 * 240, 0, 240 + 24 * i / 144.0 * 240, 240);
  }
  for (int i = 1; i < 6; i++) {
    Brain.Screen.drawLine(240, 24 * i / 144.0 * 240, 480, 24 * i / 144.0 * 240);
  }

  Brain.Screen.drawLine(240, 240, 480, 0);

  Brain.Screen.drawCircle(240 + globalX / 144.0 * 240, 240 - (globalY / 144.0 * 240), 5, orange);
}

double inertialAdjusted() {
  double curr = inertialSensor.heading();
  return curr - (inertialSensor.rotation() - initHeading) / 360.0 * errorPerRotation;
}

double getAngleDiff(double prev, double curr) {
  double diff = curr - prev;
  if (fabs(diff) > 180) {
    return diff > 0 ? (diff) - 360 : (diff) + 360;
  } else {
    return diff;
  }
}

double toRad(double deg) {
  return deg * M_PI / 180;
}

double toDeg(double rad) {
  return rad * 180 / M_PI;
}

int startTracking() {

  double prevAngle = initHeading;
  double currAngle = initHeading;
  double angleChange = 0;
  double headingDiff;

  double sideWheelCurr;
  double backWheelCurr;
  double sideWheelPrev = 0;
  double backWheelPrev = 0;
  double sideWheelDelta;
  double backWheelDelta;
  
  double deltaX;
  double deltaY;

  leftEncoder.resetRotation();
  backEncoder.resetRotation();

  while (true) {
    currAngle = inertialAdjusted();
    angleChange = toRad(getAngleDiff(prevAngle, currAngle));

    sideWheelCurr = leftEncoder.position(deg);
    sideWheelDelta = (sideWheelCurr - sideWheelPrev) * sideWheelCirc / 360;
    backWheelCurr = backEncoder.position(deg);
    backWheelDelta = (backWheelCurr - backWheelPrev) * backWheelCirc / 360;


    headingDiff = toRad(getAngleDiff(0, currAngle));

    if (angleChange == 0) {
      deltaX = backWheelDelta * cos(headingDiff) - sideWheelDelta * sin(headingDiff);
      deltaY = backWheelDelta * sin(headingDiff) + sideWheelDelta * cos(headingDiff);
    } else {
      deltaX = (2 * sin(angleChange/2) * (backWheelDelta/angleChange - backDist) * cos(headingDiff)) - (2 * sin(angleChange/2) * (sideWheelDelta/angleChange - sideDist) * sin(headingDiff));
      deltaY = (2 * sin(angleChange/2) * (backWheelDelta/angleChange - backDist) * sin(headingDiff)) + (2 * sin(angleChange/2) * (sideWheelDelta/angleChange - sideDist) * cos(headingDiff));
    }

    globalX -= deltaX;
    globalY += deltaY;
    globalAngle = currAngle;

    prevAngle = currAngle;
    sideWheelPrev = sideWheelCurr;
    backWheelPrev = backWheelCurr;

  

    wait(5, msec);

  }
  return 1;
}