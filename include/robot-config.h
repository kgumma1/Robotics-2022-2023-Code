using namespace vex;

extern brain Brain;
extern controller Controller;

extern motor flywheel;
extern motor intake_roller;

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
extern line topIntakeSensor;

extern encoder leftEncoder;
extern encoder rightEncoder;

extern inertial inertialSensor;
extern optical leftRollerSensor;
extern optical rightRollerSensor;

extern digital_out angleChanger;
extern digital_out basket;
extern digital_out intakeLift;



// GLOBAL VARIABLES
extern double topIntakeSensorInit;
extern double bottomIntakeSensorInit;
extern double Al, Ar, As, rs, xPos1, yPos1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
