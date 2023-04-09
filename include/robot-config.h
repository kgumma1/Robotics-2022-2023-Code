using namespace vex;


extern brain Brain;
extern controller Controller;

extern motor cataMain;
extern motor intake_roller_cata;

extern motor RFDrive;
extern motor RMDrive;
extern motor RBDrive;
extern motor_group rDrive;

extern motor LFDrive;
extern motor LMDrive;
extern motor LBDrive;
extern motor_group lDrive;


extern triport Expander;

extern line bottomIntakeSensor;

extern encoder leftEncoder;
extern encoder backEncoder;

extern inertial inertialSensor;
extern optical leftRollerSensor;
extern optical rightRollerSensor;
extern distance distanceSensor;
extern rotation cataSensor;

extern digital_out intakeLift;
extern digital_out expansionLeft;
extern digital_out expansionRight;
extern digital_out pistonBoost;
extern digital_out bandBoost;



// GLOBAL VARIABLES
extern double bottomIntakeSensorInit;

extern double globalX;
extern double globalY;
extern double globalAngle;

extern double resetAngle;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
