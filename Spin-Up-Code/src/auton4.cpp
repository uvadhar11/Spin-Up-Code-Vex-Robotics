#include "vex.h"
using namespace std;
#include <iostream>
#include "linking.h"

void driveBack(int pwr) {
  LeftFrontMotor.spin(reverse, pwr, volt);
  RightFrontMotor.spin(reverse, pwr, volt);
  LeftMiddleMotor.spin(reverse, pwr, volt);
  RightMiddleMotor.spin(reverse, pwr, volt);
  LeftBackMotor.spin(reverse, pwr, volt);
  RightBackMotor.spin(reverse, pwr, volt);
}

void driveForward(int pwr) {
  LeftFrontMotor.spin(fwd, pwr, volt);
  RightFrontMotor.spin(fwd, pwr, volt);
  LeftMiddleMotor.spin(fwd, pwr, volt);
  RightMiddleMotor.spin(fwd, pwr, volt);
  LeftBackMotor.spin(fwd, pwr, volt);
  RightBackMotor.spin(fwd, pwr, volt);
}

void auton4() {
  // prog skills
  LeftFrontMotor.spin(reverse, 12, volt);
  RightFrontMotor.spin(reverse, 12, volt);
  LeftMiddleMotor.spin(reverse, 12, volt);
  RightMiddleMotor.spin(reverse, 12, volt);
  LeftBackMotor.spin(reverse, 12, volt);
  RightBackMotor.spin(reverse, 12, volt);
  FlywheelMotor.spin(fwd, 12, volt);


  wait(0.1, sec);

  LeftFrontMotor.stop();
  RightFrontMotor.stop();
  LeftMiddleMotor.stop();
  RightMiddleMotor.stop();
  LeftBackMotor.stop();
  RightBackMotor.stop();

  // SPIN ROLLERS
  IntakeMotor.spin(reverse, 12, volt);
  wait(0.4, sec);
  IntakeMotor.stop();
  FlywheelMotor.stop();


  // turn and intake the other disc
  drivePID(325, 1);
  turnPID(90, 1);

  // turn and drive to rollers
  IntakeMotor.spin(fwd, 12, volt);
  drivePID(-650, 1);

  // SPIN ROLLERS
  LeftFrontMotor.spin(reverse, 12, volt);
  RightFrontMotor.spin(reverse, 12, volt);
  LeftMiddleMotor.spin(reverse, 12, volt);
  RightMiddleMotor.spin(reverse, 12, volt);
  LeftBackMotor.spin(reverse, 12, volt);
  RightBackMotor.spin(reverse, 12, volt);
  FlywheelMotor.spin(fwd, 12, volt);

  wait(0.1, sec);

  LeftFrontMotor.stop();
  RightFrontMotor.stop();
  LeftMiddleMotor.stop();
  RightMiddleMotor.stop();
  LeftBackMotor.stop();
  RightBackMotor.stop();

  IntakeMotor.spin(reverse, 12, volt);
  wait(0.4, sec);
  IntakeMotor.stop();
  FlywheelMotor.stop();

  // go to high goal
  drivePID(200, 1);
  drivePID(-100, 1);
  turnPID(3, 1);
  drivePID(1800, 1);
  turnPID(13, 1);

  // shoot
  FlywheelMotor.spin(reverse, 9, volt);

  // rpm
  while (FlywheelMotor.velocity(rpm) > -280) {

  }
  IntakeMotor.spin(reverse, 12, volt);
  wait(2, sec);
  IntakeMotor.stop();
  FlywheelMotor.stop();
  wait(0.1, sec);

  // after shooting drive back
  drivePID(-1350, 1);

  // turn towards 3 stack
  turnPID(-135, 1); // 90

  // intake the 3 stack
  IntakeMotor.spin(fwd, 12, volt);
  drivePID(-900, 0.5);


  // one you have intaked the 3 stack, drive back to shoot in high goal
  turnPID(45, 1);

  FlywheelMotor.spin(reverse, 12, volt);
  wait(3, sec);
  IntakeMotor.spin(reverse, 12, volt);
  wait(2.5, sec);
  FlywheelMotor.stop();
  IntakeMotor.stop();

  // now intake the other 3 discs
  // might have to go back to near high goal and shoot.
}