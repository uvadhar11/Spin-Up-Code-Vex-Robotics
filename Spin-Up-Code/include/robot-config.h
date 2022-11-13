using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern led Pneumatics;
extern optical Optical;
extern led ExpansionPneumatics;
extern motor LeftFrontMotor;
extern motor LeftBackMotor;
extern motor RightFrontMotor;
extern motor RightBackMotor;
extern motor IntakeMotor;
extern motor FlywheelMotor;
extern motor FlywheelMotor2;
extern inertial Gyro1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );