#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
led Exp2 = led(Brain.ThreeWirePort.B);
optical Optical = optical(PORT10);
led Exp1 = led(Brain.ThreeWirePort.A);
motor LeftFrontMotor = motor(PORT1, ratio6_1, false);
motor LeftMiddleMotor = motor(PORT2, ratio6_1, false);
motor LeftBackMotor = motor(PORT3, ratio6_1, false);
motor RightFrontMotor = motor(PORT4, ratio6_1, false);
motor RightMiddleMotor = motor(PORT5, ratio6_1, false);
motor RightBackMotor = motor(PORT6, ratio6_1, false);
motor FlywheelMotor = motor(PORT7, ratio6_1, false);
motor IntakeMotor = motor(PORT8, ratio18_1, false);
inertial Gyro1 = inertial(PORT9);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}