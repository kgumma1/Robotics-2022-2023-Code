#include "vex.h"

using namespace vex;

const double PI = 3.14159265;
double r = 3.25/2; // radius of wheel or radius of robot?
double tWheelCirc = 3.25 * PI;
double magicOffset = 0.1;
double Al, Ar, As, rs, xPos1, yPos1;


float getRightTrackerArc() {
  return ((FRDrive.position(rev)+BRDrive.position(rev))*tWheelCirc*0.66666666*magicOffset);
}


float getLeftTrackerArc() {
  return ((FLDrive.position(rev)+BLDrive.position(rev))*tWheelCirc*0.66666666*magicOffset);
}

// float getSidewaysTrackerArc() {return ((TrackerS.position(rev))*tWheelCircS);}
float getRobotAngle()
{
  float leftArc = getLeftTrackerArc();
  float rightArc = getRightTrackerArc();
  float botAngle = (leftArc-rightArc)/(2*r);
  return botAngle;
}

void LinTrackOld()
{
  double Al_prev, Ar_prev, As_prev, theta, rdx, rdy, dx, dy, theta_prev=0, dTheta=0;
  // record current tracker measurements
  Al_prev = getLeftTrackerArc();
  Ar_prev = getRightTrackerArc();
  // As_prev = getSidewaysTrackerArc();
  theta_prev = getRobotAngle();

  // wait duh
  wait(10, msec);

  // calculate changes in encoder
  Al = getLeftTrackerArc() - Al_prev;
  Ar = getRightTrackerArc() - Ar_prev;
  // As = getSidewaysTrackerArc() - As_prev;
  theta = getRobotAngle();
  dTheta = theta - theta_prev;

  // calculate relative movement components
  rdx = As + dTheta*rs;
  rdy = (Ar + Al)/2;

  // calculate global movement components
  dy = rdy*cos(theta) - rdx*sin(theta);
  dx = rdx*cos(theta) + rdy*sin(theta);

  // update global position
  xPos1 += dx;
  yPos1 += dy; 

  //printf("Al = %f, Ar = %f, As = %f\n",getLeftTrackerArc(), getRightTrackerArc(), getSidewaysTrackerArc());
  //printf("Alp = %f, Arp = %f, Asp = %f\n",Al_prev, Ar_prev, As_prev);
}