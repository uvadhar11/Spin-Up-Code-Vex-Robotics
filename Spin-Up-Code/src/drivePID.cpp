#include "vex.h"

void drivePID(int desiredValue, double multiplier){
  //Settings - variables initializations
  double kP = 0.115; // 0.11
  double kI = 0.000000001; // 0.000000000001     2
  double kD = 0.0312; // 0.009 -> clean 0.02

  //Autonomous Settings
  int error = 0;
  int prevError = 0;
  int derivative;
  int totalError = 0;

  //Reset the motor encoder positions
  LeftFrontMotor.setPosition(0, degrees);
  LeftMiddleMotor.setPosition(0, degrees);
  LeftBackMotor.setPosition(0, degrees);
  RightFrontMotor.setPosition(0, degrees);
  RightMiddleMotor.setPosition(0, degrees);
  RightBackMotor.setPosition(0, degrees);
  double targetGyroPosition = Inertial.yaw(rotationUnits::deg);

  Brain.Screen.clearScreen();

  
  while(true) {
    //Get the position of the motors
    int leftFrontMotorPosition = LeftFrontMotor.position(degrees);
    int leftMiddleMotorPosition = LeftMiddleMotor.position(degrees);
    int leftBackMotorPosition = LeftBackMotor.position(degrees);
    int rightFrontMotorPosition = RightFrontMotor.position(degrees);
    int rightMiddleMotorPosition = RightMiddleMotor.position(degrees);
    int rightBackMotorPosition = RightBackMotor.position(degrees);
  
    // self-correction variables
    double leftValue;
    double rightValue;

    leftValue = 0.0;
    rightValue = 0.0;
  
    // printing values to brain


    // for self-correction stuff
    // int GyroPosition = Inertial.yaw(rotationUnits::deg) - targetGyroPosition;
  

    // if (Inertial.yaw(rotationUnits::deg) < targetGyroPosition) {

    //   rightValue = abs(GyroPosition) * abs(GyroPosition) / selfCorrect;

    // } else {

    //   rightValue = 0;
    // }
  

    // if (Inertial.yaw(rotationUnits::deg) > targetGyroPosition) {
    
    //   leftValue = abs(GyroPosition) * abs(GyroPosition) / selfCorrect;
  
    // } else {
  
    //   leftValue = 0;
    // }
  

    //Lateral Movement PID/Going forward and back

    //Get the average of the motors
    int averagePosition = ((leftFrontMotorPosition + leftBackMotorPosition + leftMiddleMotorPosition + rightFrontMotorPosition + rightMiddleMotorPosition + rightBackMotorPosition)/6);
    
    //Potential
    error = desiredValue - averagePosition;
    
    //Derivative
    derivative = error - prevError;
    
    //Integral - trying to make sure you don't undershoot. So as time goes on when u don't reach target, it increases speed. Integral wind-up when not reaching target over time so when closer to target it might not slow down all the way, which is the problem
    totalError += error; // this is the integral

    // std::cout <<  
    // error 
    // << std::endl; 

    // printing values
    // error
    Brain.Screen.setCursor(1, 2);
    Brain.Screen.print(error);
    // average positions
    Brain.Screen.setCursor(2, 2);
    Brain.Screen.print(LeftFrontMotor.position(deg));

    Brain.Screen.setCursor(3, 2);
    Brain.Screen.print(LeftMiddleMotor.position(deg));

    Brain.Screen.setCursor(4, 2);
    Brain.Screen.print(LeftBackMotor.position(deg));

    Brain.Screen.setCursor(5, 2);
    Brain.Screen.print(RightFrontMotor.position(deg));

    Brain.Screen.setCursor(6, 2);
    Brain.Screen.print(RightMiddleMotor.position(deg));

    Brain.Screen.setCursor(7, 2);
    Brain.Screen.print(RightBackMotor.position(deg));

    // motor encoder positions
    // std::cout <<  
    // leftFrontMotorPosition << " , " <<  leftMiddleMotorPosition << " , " 
    // << leftBackMotorPosition << " , " << rightFrontMotorPosition << " , " << rightMiddleMotorPosition << " , " << rightBackMotorPosition
    // << std::endl; 

    // error
    std::cout << error << std::endl;

    // other checks like integral
    if (error == 0) totalError = 0;

  
    if (abs(error) <= 15) {
      Brain.Screen.print(abs(error));
      break;
    } 
    else {
      // means the error is greater than stop PID or the value set there.
      totalError = 0; // integral = 0
    }

    // calculate motor power
    double lateralMotorPower = ((error * kP) + (derivative * kD) + (totalError * kI)) * multiplier;//* kD and (totalError * kI)
    
    // move the motors - R was rev
    LeftFrontMotor.spin(fwd, lateralMotorPower + leftValue, voltageUnits::volt);//+ turnMotorPower (if turning). L/R for self-correction
    RightFrontMotor.spin(fwd, lateralMotorPower + rightValue, voltageUnits::volt);//- turnMotorPower
    LeftMiddleMotor.spin(fwd, lateralMotorPower + leftValue, voltageUnits::volt);//+ turnMotorPower
    RightMiddleMotor.spin(fwd, lateralMotorPower + rightValue, voltageUnits::volt);//- turnMotorPower
    LeftBackMotor.spin(fwd, lateralMotorPower + leftValue, voltageUnits::volt);//+ turnMotorPower
    RightBackMotor.spin(fwd, lateralMotorPower + rightValue, voltageUnits::volt);//- turnMotorPower

    // Brain.Screen.print(error + ", " + averagePosition);
    
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

void drivePID(int desiredValue, double multiplier, bool intakeWhile){
  //Settings - variables initializations
  double kP = 0.115; // 0.11
  double kI = 0.000000001; // 0.000000000001     2
  double kD = 0.0312; // 0.009 -> clean 0.02

  //Autonomous Settings
  int error = 0;
  int prevError = 0;
  int derivative;
  int totalError = 0;

  //Reset the motor encoder positions
  LeftFrontMotor.setPosition(0, degrees);
  LeftMiddleMotor.setPosition(0, degrees);
  LeftBackMotor.setPosition(0, degrees);
  RightFrontMotor.setPosition(0, degrees);
  RightMiddleMotor.setPosition(0, degrees);
  RightBackMotor.setPosition(0, degrees);
  double targetGyroPosition = Inertial.yaw(rotationUnits::deg);

  Brain.Screen.clearScreen();

  
  while(true) {
    //Get the position of the motors
    int leftFrontMotorPosition = LeftFrontMotor.position(degrees);
    int leftMiddleMotorPosition = LeftMiddleMotor.position(degrees);
    int leftBackMotorPosition = LeftBackMotor.position(degrees);
    int rightFrontMotorPosition = RightFrontMotor.position(degrees);
    int rightMiddleMotorPosition = RightMiddleMotor.position(degrees);
    int rightBackMotorPosition = RightBackMotor.position(degrees);
  
    // self-correction variables
    double leftValue;
    double rightValue;

    leftValue = 0.0;
    rightValue = 0.0;
  
    // printing values to brain


    // for self-correction stuff
    // int GyroPosition = Inertial.yaw(rotationUnits::deg) - targetGyroPosition;
  

    // if (Inertial.yaw(rotationUnits::deg) < targetGyroPosition) {

    //   rightValue = abs(GyroPosition) * abs(GyroPosition) / selfCorrect;

    // } else {

    //   rightValue = 0;
    // }
  

    // if (Inertial.yaw(rotationUnits::deg) > targetGyroPosition) {
    
    //   leftValue = abs(GyroPosition) * abs(GyroPosition) / selfCorrect;
  
    // } else {
  
    //   leftValue = 0;
    // }
  

    //Lateral Movement PID/Going forward and back

    //Get the average of the motors
    int averagePosition = ((leftFrontMotorPosition + leftBackMotorPosition + leftMiddleMotorPosition + rightFrontMotorPosition + rightMiddleMotorPosition + rightBackMotorPosition)/6);
    
    //Potential
    error = desiredValue - averagePosition;
    
    //Derivative
    derivative = error - prevError;
    
    //Integral - trying to make sure you don't undershoot. So as time goes on when u don't reach target, it increases speed. Integral wind-up when not reaching target over time so when closer to target it might not slow down all the way, which is the problem
    totalError += error; // this is the integral

    // std::cout <<  
    // error 
    // << std::endl; 

    // printing values
    // error
    Brain.Screen.setCursor(1, 2);
    Brain.Screen.print(error);
    // average positions
    Brain.Screen.setCursor(2, 2);
    Brain.Screen.print(LeftFrontMotor.position(deg));

    Brain.Screen.setCursor(3, 2);
    Brain.Screen.print(LeftMiddleMotor.position(deg));

    Brain.Screen.setCursor(4, 2);
    Brain.Screen.print(LeftBackMotor.position(deg));

    Brain.Screen.setCursor(5, 2);
    Brain.Screen.print(RightFrontMotor.position(deg));

    Brain.Screen.setCursor(6, 2);
    Brain.Screen.print(RightMiddleMotor.position(deg));

    Brain.Screen.setCursor(7, 2);
    Brain.Screen.print(RightBackMotor.position(deg));

    // motor encoder positions
    // std::cout <<  
    // leftFrontMotorPosition << " , " <<  leftMiddleMotorPosition << " , " 
    // << leftBackMotorPosition << " , " << rightFrontMotorPosition << " , " << rightMiddleMotorPosition << " , " << rightBackMotorPosition
    // << std::endl; 

    // error
    std::cout << error << std::endl;

    // other checks like integral
    if (error == 0) totalError = 0;

  
    if (abs(error) <= 15) {
      Brain.Screen.print(abs(error));
      break;
    } 
    else {
      // means the error is greater than stop PID or the value set there.
      totalError = 0; // integral = 0
    }

    // calculate motor power
    double lateralMotorPower = ((error * kP) + (derivative * kD) + (totalError * kI)) * multiplier;//* kD and (totalError * kI)
    
    // move the motors - R was rev
    LeftFrontMotor.spin(fwd, lateralMotorPower + leftValue, voltageUnits::volt);//+ turnMotorPower (if turning). L/R for self-correction
    RightFrontMotor.spin(fwd, lateralMotorPower + rightValue, voltageUnits::volt);//- turnMotorPower
    LeftMiddleMotor.spin(fwd, lateralMotorPower + leftValue, voltageUnits::volt);//+ turnMotorPower
    RightMiddleMotor.spin(fwd, lateralMotorPower + rightValue, voltageUnits::volt);//- turnMotorPower
    LeftBackMotor.spin(fwd, lateralMotorPower + leftValue, voltageUnits::volt);//+ turnMotorPower
    RightBackMotor.spin(fwd, lateralMotorPower + rightValue, voltageUnits::volt);//- turnMotorPower

    if (intakeWhile) {
      IntakeMotor.spin(fwd, 12, volt);
    }

    // Brain.Screen.print(error + ", " + averagePosition);
    
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