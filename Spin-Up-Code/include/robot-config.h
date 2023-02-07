using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern led Exp2;
extern optical Optical;
extern led Exp1;
extern motor LeftFrontMotor;
extern motor LeftMiddleMotor;
extern motor LeftBackMotor;
extern motor RightFrontMotor;
extern motor RightMiddleMotor;
extern motor RightBackMotor;
extern motor FlywheelMotor;
extern motor IntakeMotor;
extern inertial Inertial;
extern encoder LeftEncoder;
extern encoder RightEncoder;
extern encoder MiddleEncoder;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );