#include "vex.h"
#include "tracking.h"
#include "bezier.h"

using namespace vex;


void Turn(double ang, double maxSpeed, double precision = 0.5, double timeout = 10000) {
  double turnkP = 0.13;
  double turnkI = 0.03;
  double turnkD = 0.7; /*+ fabs(getAngleDiff(globalAngle, ang) / 900.0)*/;

  double angleError;
  double prevAngleError;
  double derivError;
  double intError = 0;
  double output;
  vex::timer turnTime = vex::timer();

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
  } while(fabs(angleError) > precision && turnTime.time() < timeout);
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

bool exitMove = false;

void move(va_list statesInputList, directionType d, int count, double initAdherence, double endOfMovePrecision, double timeout) {
  exitMove = false;
  printf("inputs = d: %d, c: %d, adh: %f, emp: %f, tm: %f\n", d == forward ? 1 : 0, count, initAdherence, endOfMovePrecision, timeout);
  printf("not %d\n", 1);
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
  printf("not %d\n", 2);
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

  vex::timer exitTimer = vex::timer();

  printf("not %d\n", 3);
  for (int i = 0; i < count; i++) {
    totalLength -= beziers[i].lengthleft(0);
    maxSpeed = states[i+1].maxSpeed;
    printf("b#: %d, x: %f, y; %f, adh: %f, speed: %f\n", i + 1, states[i].location.x, states[i].location.y, states[i].angle, states[i].maxSpeed);
    
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
    } while(beziers[i].lengthleft(tOfClosestPoint) > 2 && exitTimer.time() < timeout && !exitMove);
  }
    printf("not %d\n", 4);

  if (exitTimer.time() < timeout && !exitMove) {
    Turn(states[count].angle, states[count].maxSpeed, endOfMovePrecision);
  }

  lDrive.stop(brake);
  rDrive.stop(brake);

  Brain.Screen.setPenColor(orange);
  for (int i = 0; i < visited.size() - 1; i++) {
    Brain.Screen.drawLine(visited[i].x, visited[i].y, visited[i+1].x, visited[i+1].y);
  }
  Brain.Screen.setPenColor(white);
  for (int i = 0; i < count; i++) {
    beziers[i].display(20);
  }

}

void move(directionType d, int count, double initAdherence, ...) {
  va_list statesInputList;
  va_start(statesInputList, initAdherence);

  move(statesInputList, d, count, initAdherence, 2, 60000);
}

void move(directionType d, int count, double initAdherence, double endOfMovePrecision, ...) {
  va_list statesInputList;
  va_start(statesInputList, endOfMovePrecision);

  move(statesInputList, d, count, initAdherence, endOfMovePrecision, 60000);
}


void move(directionType d, int count, double initAdherence, double endOfMovePrecision, double timeout, ...) {
  va_list statesInputList;
  va_start(statesInputList, timeout);

  move(statesInputList, d, count, initAdherence, endOfMovePrecision, timeout);
}

va_list moveParallelList;
directionType parallelD;
int parallelCount;
double parallelInitAdherence;
double parallelEndOfMovePrecision;
double parallelTimeout;

int parallelHandler() {
  printf("phand%d\n",1 );
  move(moveParallelList, parallelD, parallelCount, parallelInitAdherence, parallelEndOfMovePrecision, parallelTimeout);
  printf("phand%d\n",2 );
  return 1;
}

vex::task moveParallel(directionType d, int count, double initAdherence, ...) {
  va_list inputList;

  va_start(inputList, initAdherence);
  moveParallelList = inputList;
  parallelD = d;
  parallelCount = count;
  parallelInitAdherence = initAdherence;
  parallelEndOfMovePrecision = 2;
  parallelTimeout = 60000;

  vex::task runMove = vex::task(parallelHandler);
  
  return runMove;
}

vex::task moveParallel(directionType d, int count, double initAdherence, double endOfMovePrecision, ...) {
  va_list inputList;

  va_start(inputList, endOfMovePrecision);
  moveParallelList = inputList;
  parallelD = d;
  parallelCount = count;
  parallelInitAdherence = initAdherence;
  parallelEndOfMovePrecision = endOfMovePrecision;
  parallelTimeout = 60000;

  vex::task runMove = vex::task(parallelHandler);

  return runMove;
}

vex::task moveParallel(directionType d, int count, double initAdherence, double endOfMovePrecision, double timeout, ...) {
  va_list inputList;

  va_start(inputList, timeout);
  moveParallelList = inputList;
  parallelD = d;
  parallelCount = count;
  parallelInitAdherence = initAdherence;
  parallelEndOfMovePrecision = endOfMovePrecision;
  parallelTimeout = timeout;

  vex::task runMove = vex::task(parallelHandler);
  
  return runMove;
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



