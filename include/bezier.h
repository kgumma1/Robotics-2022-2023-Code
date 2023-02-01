#include "vex.h"

using namespace vex;

class Point {
  public:

    // FIELDS
    double x;
    double y;

    // CONSTRUCTORS
    Point() {
      x = 0;
      y = 0;
    }

    Point(double x, double y) {
      this->x = x;
      this->y = y;
    }

    // FUNCTIONS
    double distTo(Point p) {
      return sqrt(pow(x-p.x, 2) + pow(y-p.y, 2));
    };
};

class Pose {
  public:

    // FIELDS
    Point location;
    double angle;
    double adherence;

    // CONSTRUCTORS
    Pose() {
      location = Point();
      angle = 0;
      adherence = 10;
    }

    Pose(Point p, double angle, double adherence) {
      this->location = p;
      this->angle = angle;
      this->adherence = adherence;
    }

    Pose(double x, double y, double angle, double adherence) {
      this->location = Point(x, y);
      this->angle = angle;
      this->adherence = adherence;
    }

    // FUNCTIONS
};

class Bezier {
  public:

    // CONSTRUCTORS
    Bezier(Pose p0, Pose p1, int sampleSize = 10, int divisions = 10) {
      poses[0] = p0,
      poses[1] = p1;
      points[0] = p0.location;
      points[3] = p1.location;
      points[1] = findControlPoint(p0);
      points[2] = findControlPoint(Pose(p1.location, p1.angle + 180, p1.adherence));

      printf("x: 0 = %f, 1 = %f, 2 = %f, 3 = %f,\n", points[0].x, points[1].x, points[2].x, points[3].x);
      printf("y: 0 = %f, 1 = %f, 2 = %f, 3 = %f,\n\n", points[0].y, points[1].y, points[2].y, points[3].y);
      this->sampleSize = sampleSize;
      this->divisions = divisions;
    }

    // PUBLIC FUNCTIONS
    double closestPointTo(Point p) {
      double minDist = p.distTo(getValue(0));
      double minT = 0;
      double spacing = 1.0 / sampleSize;
      for (int i = 0; i <= sampleSize; i++) {
        if (p.distTo(getValue(spacing * i)) < minDist) {
          minDist = p.distTo(getValue(spacing * i));
          minT = spacing * i;
        }
      }
      //printf("t = %f\n", minT);
      if (minT > 0 && minT < 1) {
        double newVals[] = {minT - spacing, minT - 0.5 * spacing, minT, minT + 0.5 * spacing, minT + spacing};
        return binarySearchMinDist(divisions, newVals, 0.5 * spacing, p);
      } else if (minT <= 0){
        double newVals[] = {minT, minT + 0.5 * spacing, minT + spacing, minT + 1.5 * spacing, minT + 2 * spacing};
        return binarySearchMinDist(divisions, newVals, 0.5 * spacing, p);
      } else if (minT >= 1){
        double newVals[] = {minT - 2 * spacing, minT - 1.5 * spacing, minT - spacing, minT - 0.5 * spacing, minT};
        return binarySearchMinDist(divisions, newVals, 0.5 * spacing, p);
      }
    }

    Point getValue(double t) {
      double x = (    pow(1-t, 3) *             points[0].x) + 
                 (3 * pow(1-t, 2) *     t     * points[1].x) +
                 (3 *    (1-t)    * pow(t, 2) * points[2].x) +
                 (                  pow(t, 3) * points[3].x);
      double y = (    pow(1-t, 3) *             points[0].y) + 
                 (3 * pow(1-t, 2) *     t     * points[1].y) +
                 (3 *    (1-t)    * pow(t, 2) * points[2].y) +
                 (                  pow(t, 3) * points[3].y);
      return Point(x, y);
    }

    double getAngle(double t) {
      double dx = (3 * pow(1-t, 2) *             (points[1].x - points[0].x)) + 
                  (6 *    (1-t)    *     t     * (points[2].x - points[1].x)) +
                  (3 *               pow(t, 2) * (points[3].x - points[2].x));

      double dy = (3 * pow(1-t, 2) *             (points[1].y - points[0].y)) + 
                  (6 *    (1-t)    *     t     * (points[2].y - points[1].y)) +
                  (3 *               pow(t, 2) * (points[3].y - points[2].y));
      return -atan2(dy, dx) * 180 / M_PI + 90; 

    }

    void display(int numSegments) {
      Point coords[numSegments + 1];

      for (int i = 0; i <= numSegments; i++) {
        coords[i] = getValue(i / (double) numSegments);
        coords[i] = convertToDisplay(coords[i]);
      }

      for (int i = 0; i < numSegments; i++) {
        Brain.Screen.drawLine(coords[i].x, coords[i].y, coords[i+1].x, coords[i+1].y);
        //printf("x = %f, y = %f\n", coords[i].x, coords[i].y);
      }

    }

  private:
    // PRIVATE FIELDS
    Pose poses[2];
    Point points[4];
    int sampleSize;
    int divisions;

    // PRIVATE FUNCTIONS
    Point findControlPoint(Point anchor, double angle, double adherence) {
      angle = angle * M_PI / 180;
      return Point(adherence * cos(-angle + M_PI / 2) + anchor.x, adherence * sin(-angle + M_PI / 2) + anchor.y);
    }

    Point findControlPoint(Pose p) {
      double adjangle = p.angle * M_PI / 180;
      return Point(p.adherence * cos(-adjangle + M_PI / 2) + p.location.x, p.adherence * sin(-adjangle + M_PI / 2) + p.location.y);
    }

    double binarySearchMinDist(int n, double vals[5], double spacing, Point p) {
      double minT = 0;
      for (int i = 0; i < 5; i++) {
        if (p.distTo(getValue(vals[i])) < p.distTo(getValue(minT))) {
          minT = vals[i];
        }
      }

      if (n == 0) {
        return minT;
      }
      
      if (minT != vals[0] && minT != vals[4]) {
        double newVals[] = {minT - spacing, minT - 0.5 * spacing, minT, minT + 0.5 * spacing, minT + spacing};
        return binarySearchMinDist(n - 1, newVals, 0.5 * spacing, p);
      } else if (minT == vals[0]){
        double newVals[] = {minT, minT + 0.5 * spacing, minT + spacing, minT + 1.5 * spacing, minT + 2 * spacing};
        return binarySearchMinDist(n - 1, newVals, 0.5 * spacing, p);
      } else if (minT == vals[4]){
        double newVals[] = {minT - 2 * spacing, minT - 1.5 * spacing, minT - spacing, minT - 0.5 * spacing, minT};
        return binarySearchMinDist(n - 1, newVals, 0.5 * spacing, p);
      }
    }

  Point convertToDisplay(Point p) {
    return Point(240 + p.x / 144 * 240, 240 - (p.y / 144.0 * 240));
  }

};