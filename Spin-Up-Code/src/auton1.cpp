#include "vex.h"
using namespace std;
#include <iostream>
#include "linking.h"

void auton1() {
  // testing shooting
  // FlywheelMotor.spin(reverse, 12, volt);
  // wait(4, sec);
  // // IntakeMotor.spin(reverse, 12, volt);
  // IntakeMotor.spinFor(reverse, 500, deg);

  // LEFT SIDE
  drivePID(-600, 1);
  turnPID(90, 1);

  // drive backwards a little bit to rollers
    LeftFrontMotor.spin(reverse, 12, volt);
    RightFrontMotor.spin(reverse, 12, volt);
    LeftMiddleMotor.spin(reverse, 12, volt);
    RightMiddleMotor.spin(reverse, 12, volt);
    LeftBackMotor.spin(reverse, 12, volt);
    RightBackMotor.spin(reverse, 12, volt);
    FlywheelMotor.spin(fwd, 12, volt);

    // wait for 0.1 seconds to allow the robot to drive back to the rollers
    wait(0.1, sec);

    // stop the motors once at the rollers
    LeftFrontMotor.stop();
    RightFrontMotor.stop();
    LeftMiddleMotor.stop();
    RightMiddleMotor.stop();
    LeftBackMotor.stop();
    RightBackMotor.stop();

    // Spin the rollers and have flywheel spinning backwards so discs don't fly out
    IntakeMotor.spin(reverse, 12, volt);
    wait(0.2, sec);
    IntakeMotor.stop();
    FlywheelMotor.stop();
}