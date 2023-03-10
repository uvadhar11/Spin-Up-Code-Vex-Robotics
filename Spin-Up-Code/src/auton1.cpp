#include "vex.h"
using namespace std;
#include <iostream>
#include "linking.h"

void auton1() {
  // testing shooting
  FlywheelMotor.spin(reverse, 12, volt);
  wait(4, sec);
  // IntakeMotor.spin(reverse, 12, volt);
  IntakeMotor.spinFor(reverse, 500, deg);
}