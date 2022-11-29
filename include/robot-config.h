using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor FlywheelUp;
extern motor FLDrive;
extern motor Intake_Roller;
extern motor BLDrive;
extern motor FRDrive;
extern motor BRDrive;
extern motor FlywheelDown;
extern controller Controller1;
extern motor Puncher;
extern inertial inertialSensor;
extern digital_out trans;
extern digital_out Indexer;
extern triport Expander;
extern digital_out StringShooters;
extern line IntakeSensor;
extern encoder rightEncoder;
extern encoder leftEncoder;
extern encoder backEncoder;

extern double intakeSensorInit;
extern double Al, Ar, As, rs, xPos1, yPos1;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );