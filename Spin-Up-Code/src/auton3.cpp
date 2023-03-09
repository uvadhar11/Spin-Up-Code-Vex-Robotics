#include "vex.h"
using namespace std;
#include <iostream>
#include "linking.h"

void auton3() {
  // MOVE BACK TO ROLLERS
    LeftFrontMotor.spin(reverse, 12, volt);
    RightFrontMotor.spin(reverse, 12, volt);
    LeftMiddleMotor.spin(reverse, 12, volt);
    RightMiddleMotor.spin(reverse, 12, volt);
    LeftBackMotor.spin(reverse, 12, volt);
    RightBackMotor.spin(reverse, 12, volt);
    FlywheelMotor.spin(fwd, 12, volt);


    wait(0.1, sec);

    // IntakeMotor.spin(reverse, 12, volt);
    // wait(0.375, sec);

    LeftFrontMotor.stop();
    RightFrontMotor.stop();
    LeftMiddleMotor.stop();
    RightMiddleMotor.stop();
    LeftBackMotor.stop();
    RightBackMotor.stop();

    // SPIN ROLLERS
    IntakeMotor.spin(reverse, 12, volt);
    wait(0.2, sec);
    IntakeMotor.stop();
    FlywheelMotor.stop();

    // // move forward away from rollers a bit
    // drivePID(100, 1);

    // // turn in line with the white line
    // turnPID(45, 1);

    // // drive fwd
    // drivePID(650, 1);

    // // shoot discs
    // turnPID(-7, 1);

    // // spin flywheel motor
    // FlywheelMotor.spin(reverse, 12, volt);

    // Brain.Screen.clearScreen();

    // // // once hits 270 ish then spin
    // while (FlywheelMotor.velocity(rpm) > -260) {
    //   // piston
    //   // Piston.on();
    //   Brain.Screen.setCursor(5, 2);
    //   Brain.Screen.print(FlywheelMotor.velocity(rpm));
    // }

    // IntakeMotor.spin(reverse, 12, volt);

    // wait(3.8, sec);
    // IntakeMotor.stop();
    // FlywheelMotor.stop();

    // // // firing 2 discs
    // // wait(0.1, sec);
    // // Piston.off();
    // // Piston.on();
    // // wait(0.1, sec);
    // // Piston.off();

    // // // turn intake facing 3 stack
    // turnPID(-130, 1); // 315

    // // // forward and intake
    // // IntakeMotor.spin(fwd, 12, volt);

    // // drivePID(-650, 0.10, true); // attempt for PID + intake - didn't really work
    // // but got one disc but then it stopped since that's what PID does.


    // LeftFrontMotor.spin(reverse, 2.5, volt);
    // RightFrontMotor.spin(reverse, 2.5, volt);
    // LeftMiddleMotor.spin(reverse, 2.5, volt);
    // RightMiddleMotor.spin(reverse, 2.5, volt);
    // LeftBackMotor.spin(reverse, 2.5, volt);
    // RightBackMotor.spin(reverse, 2.5, volt);
    // IntakeMotor.spin(fwd, 12, volt);



    // wait(4.23, sec);

    // // IntakeMotor.spin(reverse, 12, volt);
    // // wait(0.375, sec);

    // LeftFrontMotor.stop();
    // RightFrontMotor.stop();
    // LeftMiddleMotor.stop();
    // RightMiddleMotor.stop();
    // LeftBackMotor.stop();
    // RightBackMotor.stop();
    // IntakeMotor.stop();

    // turnPID(-20, 1);

    // // shoot
    // // // once hits 270 ish then spin
    // while (FlywheelMotor.velocity(rpm) > -270) {
    //   // piston
    //   // Piston.on();
    //   Brain.Screen.setCursor(5, 2);
    //   Brain.Screen.print(FlywheelMotor.velocity(rpm));
    // }

    // IntakeMotor.spin(reverse, 12, volt);

    // wait(5.5, sec);
    // IntakeMotor.stop();
    // FlywheelMotor.stop();

    Brain.Screen.clearScreen();
    Brain.Screen.print("Hello");
}