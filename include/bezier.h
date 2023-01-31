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
    Bezier(Pose p0, Pose p1) {
      poses[0] = p0,
      poses[1] = p1;
      points[0] = p0.location;
      points[3] = p1.location;
      points[1] = findControlPoint(p0);
      points[2] = findControlPoint(p1);
    }

    // PUBLIC FUNCTIONS
    double closestPointTo(Point p) { // WORK IN PROGRESS
      return p.x;
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

  private:
    // PRIVATE FIELDS
    Pose poses[2];
    Point points[4];

    // PRIVATE FUNCTIONS
    Point findControlPoint(Point anchor, double angle, double adherence) {
      return Point(adherence * cos(-angle + M_PI / 2) + anchor.x, adherence * sin(-angle + M_PI / 2) + anchor.y);
    }

    Point findControlPoint(Pose p) {
      return Point(p.adherence * cos(-p.angle + M_PI / 2) + p.location.x, p.adherence * sin(-p.angle + M_PI / 2) + p.location.y);
    }
    

};