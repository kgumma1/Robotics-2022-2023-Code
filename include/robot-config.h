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
extern digital_out angleChanger;
extern line IntakeSensor;
extern line BottomIntakeSensor;
extern rotation rotSensor;
extern encoder rightEncoder;
extern encoder leftEncoder;
extern encoder backEncoder;
extern vex::vision::signature Vision__SIG_1;
extern vex::vision::signature Vision__SIG_2;
extern vex::vision::signature Vision__SIG_3;
extern vex::vision::signature Vision__SIG_4;
extern vex::vision::signature Vision__SIG_5;
extern vex::vision::signature Vision__SIG_6;
extern vex::vision::signature Vision__SIG_7;
extern vision Vision;

extern double intakeSensorInit;
extern double bottomIntakeSensorInit;
extern double Al, Ar, As, rs, xPos1, yPos1;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );