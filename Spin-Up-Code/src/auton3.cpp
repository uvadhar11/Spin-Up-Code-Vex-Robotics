#include "vex.h"
using namespace std;
#include <iostream>
#include "linking.h"

void auton3() {
  // MOVE BACK TO ROLLERS

  // UMCOMMENT OUT BEFORE COMP....
    // LeftFrontMotor.spin(reverse, 12, volt);
    // RightFrontMotor.spin(reverse, 12, volt);
    // LeftMiddleMotor.spin(reverse, 12, volt);
    // RightMiddleMotor.spin(reverse, 12, volt);
    // LeftBackMotor.spin(reverse, 12, volt);
    // RightBackMotor.spin(reverse, 12, volt);
    // FlywheelMotor.spin(fwd, 12, volt);


    // wait(0.1, sec);

    // LeftFrontMotor.stop();
    // RightFrontMotor.stop();
    // LeftMiddleMotor.stop();
    // RightMiddleMotor.stop();
    // LeftBackMotor.stop();
    // RightBackMotor.stop();

    // // SPIN ROLLERS
    // IntakeMotor.spin(reverse, 12, volt);
    // wait(0.2, sec);
    // IntakeMotor.stop();
    // FlywheelMotor.stop();

    // start fly spooling up
    // FlywheelMotor.spin(reverse, 12, volt);

    // // // move forward away from rollers a bit
    // drivePID(250, 1);

    // // turn to high goal
    // turnPID(-9, 1);

    // // spool up
    // // wait(3, sec);
    // while (FlywheelMotor.velocity(rpm) > -300) {

    // }

    // shoot
    // IntakeMotor.spin(reverse, 12, volt);
    // wait(1, sec);
    // IntakeMotor.stop();
    // wait(0.1, sec);
    // IntakeMotor.spin(reverse, 12, volt);
    // wait(0.15, sec);

    // FlywheelMotor.stop();
    // IntakeMotor.stop();

    // wait(0.15, sec);

    // turn to the 3 stack
    // turnPID(-130, 1);

    // // intake and drive
    // IntakeMotor.spin(fwd, 12, volt);
    // drivePID(650, 1);
    // drivePID(325, 1);
    // drivePID(325, 1);

    // IntakeMotor.stop();
    // FlywheelMotor.spin(reverse, 12, volt);
    // turnPID(45, 1);
    // wait(3, sec);

    // // shooting
    // for (int i = 0; i < 3; i++) {
    //   IntakeMotor.spin(reverse, 12, volt);
    //   wait(0.15, sec);
    //   IntakeMotor.stop();
    //   wait(0.1, sec);
    // }






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

    // // spin flywheel motor
    FlywheelMotor.spin(reverse, 12, volt);

    // drive away from the rollers a little
    drivePID(100, 1);

    // turn in line with the white line
    turnPID(45, 2);

    // drive forward just before the 3 stack
    drivePID(650, 1);

    // turn to the high goal
    turnPID(-11, 1);

    // // spin flywheel motor
    // FlywheelMotor.spin(reverse, 12, volt);

    // clear brain screen
    Brain.Screen.clearScreen();

    // once the flywheel hits 270-ish, spin the indexer
    while (FlywheelMotor.velocity(rpm) > -380) { // -330
      // print flywheel velocity to the brain so I can see
      Brain.Screen.setCursor(5, 2);
      Brain.Screen.print(FlywheelMotor.velocity(rpm));
    }
    wait(0.12,sec);
    // index
    IntakeMotor.spin(reverse, 12, volt);
    wait(1.2, sec);
    // IntakeMotor.spinFor(reverse, 500, deg);
    // wait(0.12, sec);
    // IntakeMotor.spinFor(reverse, 500, deg);
    // wait(0.1, sec);


    // index for this time to let all the discs shoot out
    // wait(1.7, sec);

    // stop flywheel and intake motor
    // IntakeMotor.stop();
    // FlywheelMotor.stop();

    // turn with the intake facing the 3 stack
    // turnPID(-127, 1); 

    // // drive and intake the 3 stack
    // LeftFrontMotor.spin(reverse, 3.5, volt);
    // RightFrontMotor.spin(reverse, 3.5, volt);
    // LeftMiddleMotor.spin(reverse, 3.5, volt);
    // RightMiddleMotor.spin(reverse, 3.5, volt);
    // LeftBackMotor.spin(reverse, 3.5, volt);
    // RightBackMotor.spin(reverse, 3.5, volt);
    // IntakeMotor.spin(fwd, 12, volt);

    // wait(3, sec);

    // // stop the motors after the above time is up
    // LeftFrontMotor.stop();
    // RightFrontMotor.stop();
    // LeftMiddleMotor.stop();
    // RightMiddleMotor.stop();
    // LeftBackMotor.stop();
    // RightBackMotor.stop();
    // // IntakeMotor.stop();

    // // turn to the high goal
    // FlywheelMotor.stop();
    // turnPID(-39, 1); // -27
    // wait(0.5, sec);
    // IntakeMotor.stop();

    // FlywheelMotor.spin(reverse, 12, volt);
    // // shoot - once the flywheel hits around 270 rpm, spin indexer
    // while (FlywheelMotor.velocity(rpm) > -330) {
    //   Brain.Screen.setCursor(5, 2);
    //   Brain.Screen.print(FlywheelMotor.velocity(rpm));
    // }

    // // spin indexer
    // IntakeMotor.spin(reverse, 12, volt);

    // // wait to allow all the discs to index out
    // wait(2.5, sec);

    // // stop the motors
    // IntakeMotor.stop();
    // FlywheelMotor.stop();

    // printing to signify the auton is over
    Brain.Screen.clearScreen();
    Brain.Screen.print("Auton Completed!");
}