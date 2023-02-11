#include "vex.h"
#include "autonFunctions.h"

using namespace vex;

#define TILE * 24
#define LEFT_TO_CENTER 6.5
#define TOP_TO_CENTER 8.25
#define TILE_EDGE .375

Point findControlPoint(Point anchor, double angle, double adherence) {
  angle = angle * M_PI / 180;
  return Point(adherence * cos(-angle + M_PI / 2) + anchor.x, adherence * sin(-angle + M_PI / 2) + anchor.y);
}

void testing() {


  //inertialSensor.setHeading(180, deg);
    printf("WORKING%d\n", 1);
  vex::task track = vex::task(startTracking);
      double initSens = 0.93;
    double sensInc = -0.0001;
    wait(1, sec);
  printf("WORKING%d\n", 1);
    globalX = 3 TILE + LEFT_TO_CENTER + TILE_EDGE;;
  globalY = 3 TILE - (TILE_EDGE + TOP_TO_CENTER);
  int speed = 20;
  //move(forward, 3, 20, State(50, 30, 160, 30, speed), State(20, 30, 0, 15, speed), State(0, 70, -30, 40, speed));
  //move(reverse, 3, 40, State(20, 30, 0, 15, 20), State(50, 30, 160, 30, 20), State(0, 0, 0, 20, 20));

  //move(forward, 2, 10, State(10, 10, 20, 30), State(30, 20, 50, 0.0001));
  //move(reverse, 2, 10, State(10, 10, 20, 30), State(30, 20, 50, 0.0001));\

  while(true) {
    displayTracking();
    wait(100, msec);
  }

  wait(300, msec);
  displayTracking();
  std::vector<Pose> poses;

  // line 1
  poses.push_back(Pose(18, 3 TILE, 5, 0.001));
  poses.push_back(Pose(1.5 TILE, 2.5 TILE, 110, 5));
  poses.push_back(Pose(2.5 TILE, 3.5 TILE, 315, 0.001));
  
  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();

  // vertical left low goal line

  poses.push_back(Pose(2.5 TILE, 3.5 TILE, 315, 0.001));
  poses.push_back(Pose(2.5 TILE, 5.5 TILE, 270, 0.001));

  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();

  // match load

  poses.push_back(Pose(2.5 TILE, 5.5 TILE, 270, -0.001));
  poses.push_back(Pose(3 TILE, 5.5 TILE, 270, -0.001));
  Brain.Screen.setPenColor(orange);
  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }
  Brain.Screen.setPenColor(white);
  poses.clear();

  // opponent stack

  poses.push_back(Pose(3 TILE, 5.5 TILE, 270, 3));
  poses.push_back(Pose(3.5 TILE, 4.5 TILE, 135, 10));

  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();

  // far center stack

  poses.push_back(Pose(3.5 TILE, 4.5 TILE, 135, 4));
  poses.push_back(Pose(5 TILE, 4.5 TILE, 90, 20));

  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();

  // TURN

  // far right roller
  poses.push_back(Pose(5 TILE, 4.5 TILE, 270, 0.001)); //reverse
  poses.push_back(Pose(6 TILE, 4.5 TILE, 270, 0.001));
  Brain.Screen.setPenColor(orange);
  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }
  Brain.Screen.setPenColor(white);
  poses.clear();

 // Turn+move

  poses.push_back(Pose(6 TILE, 4.5 TILE, 270, 10));
  poses.push_back(Pose(5 TILE, 4.5 TILE, 180, 0.001)); 

  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();
  
  // far top roller
  poses.push_back(Pose(5 TILE, 4.5 TILE, 180, 0.001));  //reverse
  poses.push_back(Pose(5 TILE, 6 TILE, 180, 0.001));
  Brain.Screen.setPenColor(orange);
  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }
  Brain.Screen.setPenColor(white);
  poses.clear();

  // shooting position before horizontal close low goal discs

  poses.push_back(Pose(5 TILE, 6 TILE, 180, 10));
  poses.push_back(Pose(5.5 TILE, 3 TILE, 210, 20)); 

  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();


  // horizontal close low goal discs

  poses.push_back(Pose(5.5 TILE, 3 TILE, 210, 20)); 
  poses.push_back(Pose(3.75 TILE, 2 TILE + 9, 270, 30));

  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();


  // position to shoot low goal discs

  poses.push_back(Pose(3.75 TILE, 2 TILE + 9, 270, -30));
  poses.push_back(Pose(5 TILE + 6, 3 TILE, 185, -3));
  
  Brain.Screen.setPenColor(orange);
  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }
  Brain.Screen.setPenColor(white);
  poses.clear();


  // line 2

  poses.push_back(Pose(5 TILE + 6, 3 TILE, 185, 0.001));
  poses.push_back(Pose(4.5 TILE, 3.5 TILE, 280, 5));
  poses.push_back(Pose(3.5 TILE, 2.5 TILE, 135, 0.001));


  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }
  
  poses.clear();

  // vertical right low goal line

  poses.push_back(Pose(3.5 TILE, 2.5 TILE, 135, 0.001));
  poses.push_back(Pose(3.5 TILE, 0.5 TILE, 90, 0.001));


  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();

  // match load

  poses.push_back(Pose(3.5 TILE, 0.5 TILE, 90, -0.001));
  poses.push_back(Pose(3 TILE, 0.5 TILE, 90, -0.001));
  Brain.Screen.setPenColor(orange);
  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }
  Brain.Screen.setPenColor(white);
  poses.clear();

  // opponent stack

  poses.push_back(Pose(3 TILE, 0.5 TILE, 90, 3));
  poses.push_back(Pose(2.5 TILE, 1.5 TILE, 315, 10));

  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();

  // roller
  poses.push_back(Pose(2.5 TILE, 1.5 TILE, 110, -50));
  poses.push_back(Pose(0 TILE, 1.5 TILE, 90, -20));
  Brain.Screen.setPenColor(orange);
  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }
  Brain.Screen.setPenColor(white);
  poses.clear();

  // endgame position
  poses.push_back(Pose(0 TILE, 1.5 TILE, 90, 15));
  poses.push_back(Pose(1 TILE, 1 TILE, 45, 15));

  for (int i = 0; i < poses.size() - 1; i++) {
    Bezier(poses[i], poses[i+1]).display(30);
  }

  poses.clear();

}

void skills() {
  globalX = 1 TILE + 4 + TILE_EDGE + LEFT_TO_CENTER;
  globalY = 1 TILE - (TILE_EDGE + TOP_TO_CENTER + 5.25);
  vex::task track = vex::task(startTracking);
  wait(5, sec);
  // move to preload shoot position
  move(forward, 2, 0.001, State(1 TILE, 1 TILE, 315, 4, 50), State(18, 3.25 TILE, 0, 20, 100));
  wait(2000, msec);

  // get discs along low goal barrier (far low goal)
  move(forward, 1, 0.001, State(2.25 TILE, 4 TILE - 9, 90, 30, 50));
  wait(2000, msec);

  // reverse to shooting position
  move(reverse, 1, 40, State(22, 3 TILE, 5, 0.001, 100));
  wait(2000, msec);
  
  // get line discs
  Turn(110, 100, 2);
    wait(2000, msec);

  move(forward, 2, 4, State(1.25 TILE, 2.75 TILE, 60, 3, 25), State(2.5 TILE, 3.5 TILE, 45, 30, 50));
  wait(2000, msec);
  Turn(315, 100);
  wait(2000, msec);

  // get rest of low goal discs
  move(forward, 1, 0.001, State(2 TILE + 8, 5.35 TILE, 270, 0.001, 50));
  wait(2000, msec);
  // match load position
  move(reverse, 1, 0.001, State(3 TILE, 5.7 TILE, 270, 0.001));
  wait(2000, msec);
  // intake opponent 3 stack
  move(forward, 1, 3, State(3.5 TILE, 4.5 TILE, 135, 10, 50));
  wait(2000, msec);
  // Turn + shoot
  Turn(290, 100);
  wait(2000, msec);
  // Get to right roller
  move(reverse, 1, 40, State(5.5 TILE, 4.38 TILE, 270, 5));
  wait(2000, msec);
  // intake far center 3-stack
  move(forward, 1, 0.001, State(4.5 TILE, 4.5 TILE, 270, 3));
  wait(2000, msec);
  // move to far roller
  move(reverse, 1, 5, State(5 TILE, 5.5 TILE, 180, 10));
  wait(2000, msec);
  // go to shoot discs in robot
  move(forward, 1, 10, State(5.3 TILE, 3.5 TILE, 190, 20));
  wait(2000, msec);
  // intake horizontal low goal line (close low goal)
  move(forward, 1, 5, State(3.75 TILE, 2 TILE + 10, 270, 40, 50));
  wait(2000, msec);
  // go to shoot low goal discs
  move(reverse, 1, 0.001, State(5 TILE + 6, 3 TILE, 185, 0.5));
  wait(2000, msec);
  // intake second line
  move(forward, 2, 0.001, State(4.5 TILE, 3.5 TILE, 280, 0.001, 25), State(3.5 TILE, 2.5 TILE, 225, 30));
  wait(2000, msec);
  Turn(135, 100);
  wait(2000, msec);
  // intake vertical low goal line (close low goal)
  move(forward, 1, 0.001, State(3.5 TILE, 0.5 TILE, 90, 0.001));
  wait(2000, msec);
  // move to match load position
  move(reverse, 1, 0.001, State(3 TILE, 0.5 TILE, 90, 0.001));
  wait(2000, msec);
  // move to and intake own 3 stack
  move(forward, 1, 3, State(2.5 TILE, 1.5 TILE, 315, 10));
  wait(2000, msec);
  // turn to shoot
  Turn(110, 100);
  wait(2000, msec);
  // move to roller
  move(reverse, 1, 50, State(0.5 TILE, 1.5 TILE, 90, 20));
  wait(2000, msec);
  // go to endgame position
  move(forward, 1, 15, State(1 TILE, 1 TILE, 45, 15));
  wait(2000, msec);


  while(true) {
    displayTracking();
    wait(10, msec);
  }
}





vex::task runWithDelay(int (*callback)(), double timeMsec) {
  wait(timeMsec, msec);
  vex::task runFunction = vex::task(callback);
  return runFunction;
}
