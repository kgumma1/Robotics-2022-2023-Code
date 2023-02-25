#include "vex.h"
#include "tracking.h"
#include "bezier.h"

using namespace vex;


void Turn(double ang, double maxSpeed, double precision = 0.5) {
  double turnkP = 0.3;
  double turnkI = 0.01;
  double turnkD = 2.3 + fabs(getAngleDiff(globalAngle, ang) / 900.0);

  double angleError;
  double prevAngleError;
  double derivError;
  double intError = 0;
  double output;

  do {

      angleError = getAngleDiff(globalAngle, ang);

      derivError = angleError - prevAngleError;

      if (fabs(turnkD * derivError) < 0.2) {
        intError = angleError + intError;
      }
      if (fabs(turnkD * derivError) > 0.5) {
        intError = 0;
      }
      
      output = angleError * turnkP + intError * turnkI + derivError * turnkD;
      output = fabs(output) > maxSpeed ? maxSpeed : output;

      prevAngleError = angleError;

      lDrive.spin(fwd, output, volt);
      rDrive.spin(fwd, -output, volt);

      printf("output = %f, error = %f, derivError = %f\n", output, angleError, derivError);

      wait(10, msec);
  } while(fabs(angleError) > precision);
  lDrive.stop(brake);
  rDrive.stop(brake);
}

class State : public Pose {
  public:
    double maxSpeed;

    State() : Pose() {
      this->maxSpeed = 100;
    }

    State(double x, double y, double angle, double adherence, double maxSpeed = 100) : Pose(x, y, angle, adherence) {
      this->maxSpeed = maxSpeed;
    }

    State(Point location, double angle, double adherence, double maxSpeed = 100) : Pose(location, angle, adherence) {
      this->maxSpeed = maxSpeed;
    }
};

double endOfMovePrecision = 2;

void move(directionType d, int count, double initAdherence, ...) {

  va_list statesInputList;
  va_start(statesInputList, initAdherence);

  State states[count + 1];
  Bezier beziers[count];
    
  double dir = (d == forward ? 1 : -1);
  states[0] = State(globalX, globalY, globalAngle, initAdherence * dir);
  


  for (int i = 1; i <= count; i++) {
    states[i] = va_arg(statesInputList, State);
    states[i].adherence *= dir;
  }

  
  double totalLength = 0;
  for (int i = 0; i < count; i++) {

    beziers[i] = Bezier(states[i], states[i+1]);
    totalLength += beziers[i].lengthleft(0);
  }

  double tOfClosestPoint;
  Point closestPoint;

  Pose robotPose;

  double crossTrackError;
  double angleError;
  double totalError;

  double powerLeft;
  double powerRight;



  double maxSpeed;

  double kAngle = dir == 1 ? 0.5 : 0.5;
  double kCross = dir == 1 ? 5 : 5;

  double kP = 3.7;
  double kI = 0;
  double kD = 0;

  std::vector<Point> visited;

  
  for (int i = 0; i < count; i++) {
    totalLength -= beziers[i].lengthleft(0);
    maxSpeed = states[i+1].maxSpeed;
    
    do {
      robotPose.location.x = globalX;
      robotPose.location.y = globalY;
      visited.push_back(robotPose.location.convertToDisplay());
      robotPose.angle = dir == 1 ? globalAngle : (globalAngle - 180 >= 0 ? globalAngle - 180 : globalAngle + 180);
      tOfClosestPoint = beziers[i].closestPointTo(robotPose.location);
      closestPoint = beziers[i].getValue(tOfClosestPoint);

      angleError = getAngleDiff(beziers[i].getAngle(tOfClosestPoint), robotPose.angle);
      crossTrackError = closestPoint.distTo(robotPose.location) * ((closestPoint.x - robotPose.location.x) * cos(toRad(robotPose.angle)) - (closestPoint.y - robotPose.location.y) * sin(toRad(robotPose.angle)) >= 0 ? 1 : -1);
      displayTracking();
      beziers[i].display(20);
      Brain.Screen.drawCircle(closestPoint.convertToDisplay().x, closestPoint.convertToDisplay().y, 5, blue);
      Brain.Screen.drawLine(closestPoint.convertToDisplay().x, closestPoint.convertToDisplay().y, robotPose.location.convertToDisplay().x, robotPose.location.convertToDisplay().y);

      totalError = beziers[i].lengthleft(tOfClosestPoint) + fabs(crossTrackError) + totalLength;
      
      powerLeft  = totalError * kP > maxSpeed ? maxSpeed : totalError * kP;
      powerRight = totalError * kP > maxSpeed ? maxSpeed : totalError * kP;
      
      powerLeft  = powerLeft * dir  + ((0.064 * fabs(powerLeft) * kAngle * -angleError) + (kCross * crossTrackError))/**(1 / pow(fabs(powerLeft), kC))*/;
      powerRight = powerRight * dir + ((0.064 * fabs(powerRight) * kAngle * angleError) + (kCross * -crossTrackError))/**(1 / pow(fabs(powerRight), kC))*/;

      Brain.Screen.printAt(5, 120, "PowLeft:  %.3f", powerLeft);
      Brain.Screen.printAt(5, 140, "PowRight: %.3f", powerRight);
      Brain.Screen.printAt(5, 160, "AngE: %.3f", angleError);
      Brain.Screen.printAt(5, 180, "AngC: %.3f", 0.064 * fabs(powerLeft) * kAngle * -angleError);
      Brain.Screen.printAt(5, 200, "CrossE: %.3f", crossTrackError);
      Brain.Screen.printAt(5, 220, "CrossC: %.3f", kCross * crossTrackError);
      Brain.Screen.printAt(5, 240, "PointAng: %.3f", beziers[i].getAngle(tOfClosestPoint));
      
      
      lDrive.spin(fwd, powerLeft / 100.0 * 12, volt);
      rDrive.spin(fwd, powerRight / 100.0 * 12, volt);


      printf("totalError: %f, powerLeft: %f, powerRight: %f, lengthLeft: %f\n", totalError, powerLeft, powerRight, beziers[i].lengthleft(tOfClosestPoint));
      wait(10, msec);
    } while(beziers[i].lengthleft(tOfClosestPoint) > 2);
  }

  Turn(states[count].angle, states[count].maxSpeed, endOfMovePrecision);

  Brain.Screen.setPenColor(orange);
  for (int i = 0; i < visited.size() - 1; i++) {
    Brain.Screen.drawLine(visited[i].x, visited[i].y, visited[i+1].x, visited[i+1].y);
  }
  Brain.Screen.setPenColor(white);
  for (int i = 0; i < count; i++) {
    beziers[i].display(20);
  }

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
double targetSpeed = 0;
double shotD = 0.2;
double maxShotT = 60;
int discsShotCount = 0;

bool volley = false;


void resetDiscCount() {
  discCount = 0;
}

int numQueued() {
  return discCount;
}

int queueDiscs(int n, double target = -1, double shotDelay = -1, double maxShotTime = -1, bool setVolley = true) {
  discCount += n;
  targetSpeed = target > 0 ? target : targetSpeed;
  shotD = shotDelay > 0 ? shotDelay : shotD;
  maxShotT = maxShotTime > 0 ? maxShotTime : maxShotT;
  volley = setVolley;
  return discCount;
}

double kP = 0.00012;
double kP2 = 0.0001;
double kPEqui = 0.00003;
double kI = 0;
double kD = 0.005;
double kDEqui = 0;

void adjustFPID(double newkP = 0.00012, double newkP2 = 0.0001, double newkI = 0, double newkD = 0.005, double newkPEqui = 0.00003, double newkDEqui = 0) {
  kP = newkP;
  kP2 = newkP2;
  kI = newkI;
  kD = newkD;
  kPEqui = newkPEqui;
  kDEqui = newkDEqui;
}

int discsIntaked;

int trackDiscsShot() {
  while (true) {
    while (flywheelSensor.reflectivity() - 4 <= flywheelSensorInit) {
      wait(5, msec);
      //printf("flywheelSensor = %ld\n", flywheelSensor.reflectivity());
    }
    while (flywheelSensor.reflectivity() - 4 > flywheelSensorInit) {
      wait(5, msec);
      //printf("flywheelSensor = %ld\n", flywheelSensor.reflectivity());
    }
    discCount = discCount > 0 ? discCount-1 : 0;
    discsIntaked = discsIntaked > 0 ? discsIntaked-1 : 0;

    wait(5, msec);
  }
  return 1;
}

double intakeSpeed = 100;

int resetIntakeSpeed() {
  intakeSpeed = 100;
  intake_roller.spin(forward, 100, pct);
  return 1;
}

double intakeThreshold = 4;

int maintain3Discs() {
  while (true) {
    while (topIntakeSensor.reflectivity() - intakeThreshold <= topIntakeSensorInit) {
      if (discsIntaked >= 3 && discCount <= 0 && bottomIntakeSensor.reflectivity() - intakeThreshold > bottomIntakeSensorInit) {
        intake_roller.spin(reverse, -intakeSpeed, pct);
      }
      if (discsIntaked < 3 && discCount <= 0) {
        intake_roller.spin(forward, intakeSpeed, pct);
      }
      wait(5, msec);
      printf("discCount = %d\n", discCount);
      printf("discsIntaked = %d\n", discsIntaked);
    }
    printf("discCount = %d\n", discCount);
    printf("discsIntaked = %d\n", discsIntaked);
    while (topIntakeSensor.reflectivity() - intakeThreshold > topIntakeSensorInit) {
      if (discsIntaked >= 3 && discCount <= 0 && bottomIntakeSensor.reflectivity() - intakeThreshold > bottomIntakeSensorInit) {
        intake_roller.spin(reverse, -intakeSpeed, pct);
      }
      if (discsIntaked < 3 && discCount <= 0) {
        intake_roller.spin(forward, intakeSpeed, pct);
      }
      wait(5, msec);
      printf("discCount = %d\n", discCount);
      printf("discsIntaked = %d\n", discsIntaked);
    }
    printf("discCount = %d\n", discCount);
    printf("discsIntaked = %d\n", discsIntaked);
    discsIntaked++;
  }
}

bool volleying = false;

int flywheelPID() {


  double integral = 0, derivative;

  double error = targetSpeed;
  double prevError = targetSpeed;
  double output = 12;
  double outputChange = 0;
  vex::timer shotTimer = vex::timer();
  vex::timer matchLoadTimer = vex::timer();
  int errorCount = 0;

  vex::task startShotCount = vex::task(trackDiscsShot);
  
  do {
    error = targetSpeed - (flywheel.velocity(rpm) * 6);
    if (targetSpeed == 0) {
      flywheel.stop(coast);
      continue;
    }
    //integral = integral + error;
    
    derivative = error - prevError;
 
    prevError = error;

    if (fabs(error) > 200) {
      outputChange = error * (error > 0 ? kP : kP2) + integral * kI + derivative * kD;  

    } else {
      outputChange = error * kPEqui + integral * kI + derivative * kDEqui; 
 
    }

  
    output = output + outputChange >= 12 ? 12 : output + outputChange;
    output = volleying ? 12 : output;
    //output = 7.2;
    if (flywheelSensor.reflectivity() - 4 > flywheelSensorInit) {
      matchLoadTimer.reset();
    }

    if (matchLoadSensor.reflectivity() - 4 > matchLoadSensorInit) {
      matchLoadTimer.reset();
    }
    if (matchLoadTimer.time() < 500) {
      output = 12;
    }

    flywheel.spin(fwd, output, volt);



    if (fabs(error) < 50) {
      errorCount++;
    } else {
      errorCount = 0;
    }
    if (discsIntaked == 0) {
      discCount = 0;
    }

    if (((errorCount >= 2 && shotTimer.value() > shotD) || shotTimer.value() > maxShotT) && discCount > 0) {
      
      if (volley) {
        intake_roller.spin(reverse, 100, percent);
        volleying = true;
      } else {
        intake_roller.startRotateFor(reverse, 580, deg, 100, velocityUnits::pct);
      }

      shotTimer.reset();
      errorCount = 0;
    } /*
    if (shotTimer.value() > shotD) {
      intake_roller.stop(coast);
      shotTimer.reset();
    }*/
    if (discCount > 0) {
      compressionBar.set(true);
    } else {
      compressionBar.set(false);
      volleying = false;
    }
    //printf("discs = %d\n", discCount);
    


    //printf("error=%f outputChange=%f output=%f speed=%f rotSpeed=%f deriv=%f\n", error, outputChange, output, (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2 * 7, (FlywheelUp.velocity(rpm) + FlywheelDown.velocity(rpm)) / 2 * 7 - rotSensor.velocity(rpm), derivative);
    printf("error = %f, outputChange = %f, output = %f\n", error, outputChange, output);
    //printf("discs = %d\n", discsIntaked);
    wait(10, msec);
  } while (true);
  return 1;
}