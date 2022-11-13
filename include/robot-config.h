using namespace vex;

extern brain Brain;


vex::motor LeftOut(vex::PORT20, true);
vex::motor LeftIn(vex::PORT19, false);
vex::motor RightOut(vex::PORT11, false);
vex::motor RightIn(vex::PORT18, true);
vex::motor FlywheelUp(vex::PORT17, true);
vex::motor FlywheelDown(vex::PORT17, true);
vex::motor Puncher(vex::PORT17, true);
vex::motor Intake_Roller(vex::PORT3, true);
motor_group Flywheel = motor_group(FlywheelUp, FlywheelDown);


vex::motor FLDrive(vex::PORT20, true); // ports need fixing
vex::motor FRDrive(vex::PORT19, false);
vex::motor BLDrive(vex::PORT11, false);
vex::motor BRDrive(vex::PORT18, true);

vex::inertial inertialSensor(vex::PORT1);

vex::controller ct;

// VEXcode devices
extern digital_out trans;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );