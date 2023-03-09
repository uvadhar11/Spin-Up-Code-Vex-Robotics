#include "vex.h"
using namespace std;
#include <iostream>

// DEFINE VARIABLES
int ticks = 15000; // for faster turn speed. So doesn't break everything else if turn doesn't work.

// TURN PID FUNCTION
void turnPID(int desiredValue, double multiplier){
  //Settings - variables initializations
  double kP = 0.2;
  double kI = 0.000000000000; // last digit was 1
  double kD = 0.0001;
 
  //Autonomous Settings
  double error = 0;
  double prevError = 0;
  int derivative;
  double totalError = 0;

  int turnTime = 0; // to keep track of the current turn time (ticks I think). To check if > 15000 or not.
  
  //Reset the positions
  LeftFrontMotor.setPosition(0, degrees);
  LeftMiddleMotor.setPosition(0, degrees); 
  LeftBackMotor.setPosition(0, degrees);
  RightFrontMotor.setPosition(0, degrees);
  RightMiddleMotor.setPosition(0, degrees);
  RightBackMotor.setPosition(0, degrees);

  while(true) {
    //Get the position of the motors (maybe more accuracy could add this in with the gyro reading)

    //int leftFrontMotorPosition = LeftFrontMotor.position(degrees);
    //int leftBackMotorPosition = LeftBackMotor.position(degrees);
    //int rightFrontMotorPosition = RightFrontMotor.position(degrees);
    //int rightMidMotorPosition = RightMidMotor.position(degrees);
    //int rightBackMotorPosition = RightBackMotor.position(degrees);

    //Get the average of the motors (just using the gyro here but maybe we could try using the motor pos + gyro for more accuracy)
    double averagePosition = Inertial.yaw(rotationUnits::deg);

    Brain.Screen.setCursor(2, 2);
    Brain.Screen.print(Inertial.angle());

    //Potential
    error = averagePosition - desiredValue;

    //Derivative
    derivative = error - prevError;

    //Integral - keep out for drivetrain
    totalError += error;

    // other checks like integral
    if (error == 0) totalError = 0;

  
    if (fabs(error) < 0.1) {
      Brain.Screen.print(fabs(error));
      break;
    } 
    else {
      // means the error is greater than stop PID or the value set there.
      totalError = 0; // integral = 0
    }

    // turn time breaking
    if (turnTime > ticks) {
      break;
    }

    // printing the error
    Brain.Screen.setCursor(4, 2);
    Brain.Screen.print(error);

    //calculate the actual PID (take out the last part (kI) fo drivetrain cuz it makes small changes that messes it up)
    //If you were to have integral: add "+ totalError * kI" at the end then the semicolon
    double lateralMotorPower = (error * kP) + (derivative * kD) + (totalError * kI); //* kD
  
    // Spin the motors
    LeftFrontMotor.spin(reverse, lateralMotorPower*multiplier, voltageUnits::volt);//+ turnMotorPower
    RightFrontMotor.spin(fwd, (lateralMotorPower)*multiplier, voltageUnits::volt);//- turnMotorPower
    LeftMiddleMotor.spin(reverse, lateralMotorPower*multiplier, voltageUnits::volt);//+ turnMotorPower
    RightMiddleMotor.spin(fwd, (lateralMotorPower)*multiplier, voltageUnits::volt);//- turnMotorPower
    LeftBackMotor.spin(reverse, lateralMotorPower*multiplier, voltageUnits::volt);//+ turnMotorPower
    RightBackMotor.spin(fwd, (lateralMotorPower)*multiplier, voltageUnits::volt);//- turnMotorPower
  
  
    // if (abs(error) < 0.1) {
  
    //   break;
    // }
  
    turnTime = turnTime + 1; // update current turn time.
  
    // if (turnTime > ticks) {
    //   break;
    // }

    prevError = error;
  }

  //stop the wheels after the while loop is done
  LeftFrontMotor.stop();
  RightFrontMotor.stop();
  LeftMiddleMotor.stop();
  RightMiddleMotor.stop();
  LeftBackMotor.stop();
  RightBackMotor.stop();
}