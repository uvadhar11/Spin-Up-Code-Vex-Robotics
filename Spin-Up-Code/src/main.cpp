/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Umang Vadhar                                              */
/*    Created:      Tue Aug 30 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <string>

#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Exp2                 led           B               
// Optical              optical       19              
// Exp1                 led           A               
// LeftFrontMotor       motor         1               
// LeftMiddleMotor      motor         2               
// LeftBackMotor        motor         3               
// RightFrontMotor      motor         4               
// RightMiddleMotor     motor         5               
// RightBackMotor       motor         6               
// FlywheelMotor        motor         7               
// IntakeMotor          motor         8               
// Inertial             inertial      20              
// LEDG                 led           G               
// Piston               led           C               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;
using namespace std;

#define _USE_MATH_DEFINES

// A global instance of competition
competition Competition;


// define your global instances of motors and other devices here
// vex::controller Controller1 = vex::controller();
// flywheel front
// vex::motor LeftFrontMotor = vex::motor(vex::PORT1, ratio18_1, false);
// vex::motor LeftBackMotor = vex::motor(vex::PORT2, ratio18_1, false);
// vex::motor RightFrontMotor = vex::motor(vex::PORT3, ratio18_1, false);
// vex::motor RightBackMotor = vex::motor(vex::PORT4, ratio18_1, true); // inverted

// vex::motor IntakeMotor = vex::motor(vex::PORT5);
// vex::motor FlywheelMotor = vex::motor(vex::PORT6);
// vex::motor FlywheelMotor2 = vex::motor(vex::PORT7);

// inertial Interial = inertial(PORT9);


// GLOBAL VARIABLES
int selfCorrect = 3;
int stopPID = 30;
int ticks = 15000; // for faster turn speed. So doesn't break everything else if turn doesn't work.
int flywheelPower = 9; // to manage flywheel speed
bool flyjustUpdated = false;
bool expand = false; // variable for allowing expanding up and down.
// bool pistonEnded = false;



/*------------------------------------------------------------------------*/
// FUNCTIONS                                                              */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*------------------------------------------------------------------------*/
double revolutions(double inches) {
  return inches * (3/7) * M_PI * 4.15; // inches * motor ticks * gear ratio * pi * wheel inches
  // inches * 300 * (3/7) * M_PI * 4.05;
}

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


// FLYWHEEL PID
int flywheelPID(){ // using rpm of the flywheel as "distance"
  int desiredRpm = 430;
  //Settings - variables initializations
  double kP = 0.05;
  double kI = 0.000000000001;
  double kD = 0.001;

  //Autonomous Settings
  int error = 0;
  int prevError = 0;
  int derivative;
  int totalError = 0;

  //Reset the motor encoder positions
  // FlywheelMotor.setPosition(0, degrees); // rotational?
  
  while(true) {
    //Get the position of the motors
    int flywheelMotorVelocity = FlywheelMotor.velocity(rpm);
    
    //Potential -> avg pos - desired rpm
    error = flywheelMotorVelocity - desiredRpm;
    
    //Derivative
    derivative = error - prevError;
    
    //Integral - trying to make sure you don't undershoot. So as time goes on when u don't reach target, it increases speed. Integral wind-up when not reaching target over time so when closer to target it might not slow down all the way, which is the problem
    totalError += error;

    // calculate motor power
    double motorPower = (error * kP) + (derivative * kD) + (totalError * kI);//* kD
    
    // move the motor
    FlywheelMotor.spin(fwd, motorPower, voltageUnits::volt);
    
    
    prevError = error;
  
    vex::task::sleep(15);
  
    // if (abs(error) < stopPID) {
    //   Brain.Screen.print(abs(error));
    //   break;
    // }
  }

  //stop the wheels after the while loop is done
  // FlywheelMotor
  return 0;
}

// TBH ALGORITHM
// POST #10 on https://www.chiefdelphi.com/t/paper-take-back-half-shooter-wheel-speed-control/121640/10

// e = S-P;                            // calculate the error;
// Y += G*e;                           // integrate the output;
// if (Y>1) Y=1; else if (Y<0) Y=0;    // clamp the output to 0..+1;
// if (signbit(e)!=signbit(d)){        // if zero crossing,
//   Y = b = 0.5*(Y+b);                // then Take Back Half
//   d = e;}                           // and save the previous error;

// S is the setpoint (target RPM)
// P is the process variable (measured RPM)
// G is the integral gain (the tuning parameter)
// Y is the output command to the motor controller
// e is the error
// d is the previous error
// b is the TBH variable

// Y, d, and b should be initialized.

int TBH() {
  // Define Constants
  double output = 0; // output value to the motor
  double tbh = 0; // tbh tuning constant
  double pI = 0; // integral tuning constant

  double prevError = 0; // previous error
  double targetRpm = 430; // target flywheel RPM

  // Calculate the error: target RPM - calculated RPM
  double error = targetRpm - FlywheelMotor.velocity(rpm);

  // Integrate the output: Integral Tuning Variable * error
  output += pI*error;

  // clamp the output
  if (output > 1) {
    output = 1;
  } else if (output < 0) {
    output = 0;
  }

  // if the error has changed sign since the last loop
  if (signbit(error) != signbit(prevError)) {
    output = tbh = 0.5*(output + tbh);
    prevError = error;
  }

  // spin the flywheel motor
  FlywheelMotor.spin(reverse, output, volt);

  vex::task::sleep(20);

  return 0;
}

int odometry() {
  // Global Variables
  float LeftEncoderPos;
  float RightEncoderPos;
  float MiddleEncoderPos;
  float xPos, yPos, heading;
  // // Function to reset the encoders and set the starting position
  // void resetOdometry() {
  //   leftEncoder = 0;
  //   rightEncoder = 0;
  //   xPos = 0;
  //   yPos = 0;
  //   heading = 0;
  // }

  // store the current encoder values in local variables
  // LeftEncoderPos = LeftEncoder.rotation(rotationUnits::deg);
  // RightEncoderPos = RightEncoder.rotation(rotationUnits::deg);
  // MiddleEncoderPos = MiddleEncoder.rotation(rotationUnits::deg);

  // calculate change in encoder's value since last cycle


  return 0;
}


// Global Variables
// float leftEncoder, rightEncoder;
// float xPos, yPos, heading;

// Function to reset the encoders and set the starting position
// void resetOdometry() {
//   leftEncoder = 0;
//   rightEncoder = 0;
//   xPos = 0;
//   yPos = 0;
//   heading = 0;
// }

// Function to update the odometry using the encoder readings
// void updateOdometry() {
//   // Measure the change in encoder values
//   float leftEncoderDelta = leftEncoder - (LeftBackMotor.rotation(rotationUnits::deg) + LeftFrontMotor.rotation(rotationUnits::deg) + LeftMiddleMotor.rotation(rotationUnits::deg));
//   float rightEncoderDelta = rightEncoder - (RightBackMotor.rotation(rotationUnits::deg) + RightFrontMotor.rotation(rotationUnits::deg) + RightMiddleMotor.rotation(rotationUnits::deg));
  
//   // Update the encoder readings
//   leftEncoder = (LeftBackMotor.rotation(rotationUnits::deg) + LeftFrontMotor.rotation(rotationUnits::deg) + LeftMiddleMotor.rotation(rotationUnits::deg));
//   rightEncoder = (RightBackMotor.rotation(rotationUnits::deg) + RightFrontMotor.rotation(rotationUnits::deg) + RightMiddleMotor.rotation(rotationUnits::deg));
  
//   // Calculate the distance traveled by each wheel
//   float leftDistance = leftEncoderDelta / 360 * 2 * 3.14159 * 4.05;
//   float rightDistance = rightEncoderDelta / 360 * 2 * 3.14159 * 4.05;

//   // Calculate the average distance traveled
//   float distance = (leftDistance + rightDistance) / 2;

//   // Calculate the change in heading
//   float headingDelta = (rightDistance - leftDistance) / 9.5;

//   // Update the heading
//   heading += headingDelta;

//   // Update the position
//   xPos += distance * cos(heading);
//   yPos += distance * sin(heading);

//   // sleep the task
//   vex::task::sleep(20);
// }
// int main() {
//   // Reset the odometry
//   resetOdometry();
//   // Main control loop
//   while(1) {
//     // Update the odometry
//     updateOdometry();
//     // Use the odometry information to control the robot
    
//     // Wait for a short period of time
//     wait(20, timeUnits::msec);
//   }
//   return 0;
// }


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  (NEEDS TO FINISH) or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
int autonSelected = 0; // can set to a default auton if necessary
std::string allianceColor = "NONE"; // track alliance color
 
//  SELECT AUTONOMOUS FUNCTION
// using a structure to keep the contents of the auton buttons in one place
struct autonRects {
 int xCoordinate;
 int yCoordinate;
 // width: left-right and length: up-down
 int rectWidth; // this and below value is a default value for all instances of the stucture.
 int rectLength; // this and above represents the size of a rectangle, only thing that differs is screen position
}; // auton1Rect, auton2Rect, auton3Rect, auton4Rect; // multiple variables storing an instance of this structure
 
int rectWidth = 240; // left-right
int rectLength = 70; // up-down
int autonButtonsStart = 60; // this will be the red/blue button size
 
// use x coordinate for drawing rectangle and NOT for pressing since 2 coordinates are 0!!!
autonRects auton1Rect = {0, 60, rectWidth, rectLength};
autonRects auton2Rect = {240, 60, rectWidth, rectLength};
autonRects auton3Rect = {0, 130, rectWidth, rectLength};
autonRects auton4Rect = {240, 130, rectWidth, rectLength};
 
// get default buttons (red + blue) and no auton selected showing up on the screen
void initalizeAutonSelector() {
 Brain.Screen.setFillColor(red);
 Brain.Screen.drawRectangle(auton1Rect.xCoordinate, auton1Rect.yCoordinate, auton1Rect.rectWidth, auton1Rect.rectLength); // screen is 480 by 240. Paramters: x coord, y coord, width, height
 Brain.Screen.setCursor(5,7); // row (up/down for vex brains), column (left/right) -> different numbers on row/cols -> to see, click on question mark near setCursor in the command sidebar under Looks
 Brain.Screen.print("AUTON 1");
 
 Brain.Screen.setFillColor(red);
 Brain.Screen.drawRectangle(auton2Rect.xCoordinate, auton2Rect.yCoordinate, auton2Rect.rectWidth, auton2Rect.rectLength); // x coord, y coord, width, height
 Brain.Screen.setCursor(5,34);
 Brain.Screen.print("AUTON 2");
 
 Brain.Screen.setFillColor(blue);
 Brain.Screen.drawRectangle(auton3Rect.xCoordinate, auton3Rect.yCoordinate, auton3Rect.rectWidth, auton3Rect.rectLength);
 Brain.Screen.setCursor(9,7);
 Brain.Screen.print("AUTON 3");
 
 Brain.Screen.setFillColor(blue);
 Brain.Screen.drawRectangle(auton4Rect.xCoordinate, auton4Rect.yCoordinate, auton4Rect.rectWidth, auton4Rect.rectLength);
 Brain.Screen.setCursor(9,34);
 Brain.Screen.print("AUTON 4");
 
 Brain.Screen.setFillColor(transparent);
 
 // only print no auton selected if the current auton program selected is 0 (autonSelected == 0)
 if (autonSelected == 0) {
   Brain.Screen.setCursor(11,3);
   Brain.Screen.print("AUTON SELECTED: NONE");
 }
 
}
 
// ALLIANCE COLOR SELECTION
int colorWidth = 80; //left-right
int colorLength = 40; //up-down
 
struct colorRects {
 int xCoordinate;
 int yCoordinate;
 int rectWidth;
 int rectLength;
};
 
colorRects redButton = {200, 10, colorWidth, colorLength};
colorRects blueButton = {220 + colorWidth, 10, colorWidth, colorLength};
 
// Function for Alliance Color Selection
void initalizeAllianceColorSelector() {
 Brain.Screen.setFillColor(transparent);
 Brain.Screen.setCursor(2,3);
 Brain.Screen.print("ALLIANCE COLOR: ");
 
 // draw red alliance color button
 Brain.Screen.setFillColor(red);
 Brain.Screen.drawRectangle(redButton.xCoordinate, redButton.yCoordinate, redButton.rectWidth, redButton.rectLength); // screen is 480 by 240. Paramters: x coord, y coord, width, height
 Brain.Screen.setCursor(2,23); // row (up/down for vex brains), column (left/right) -> different numbers on row/cols -> to see, click on question mark near setCursor in the command sidebar under Looks
 Brain.Screen.print("RED");
 
 // draw blue alliance color button
 Brain.Screen.setFillColor(blue);
 Brain.Screen.drawRectangle(blueButton.xCoordinate, blueButton.yCoordinate, blueButton.rectWidth, blueButton.rectLength); // screen is 480 by 240. Paramters: x coord, y coord, width, height
 Brain.Screen.setCursor(2,33); // row (up/down for vex brains), column (left/right) -> different numbers on row/cols -> to see, click on question mark near setCursor in the command sidebar under Looks
 Brain.Screen.print("BLUE");
 
 // print that no alliance color is selected + set fill to transparent ONLY if no auton is selected
 // otherwise it will flash between the selected one and none because this function is called to reset
 // the other rectangle's color to red/blue to eliminate the chance of having multiple green rectangles
 if (!(allianceColor == "RED" || allianceColor == "BLUE")) {  // if allianceColor isnt red/blue...
   Brain.Screen.setFillColor(transparent);
   Brain.Screen.setCursor(11, 28);
   Brain.Screen.print("COLOR: NONE");
 }
  // return 0;
};

// Auton Screen Selector Function
void autonScreenSelector () {
  if (Brain.Screen.pressing()) { // if the screen is touched. x and y position() allows to get the x and y of the touch on the screen
     
     // if statements for which auton is selected based on which rectangle is pressed (where press is)
     // use x coordinate for drawing rectangle and NOT for pressing since 2 coordinates are 0!!!
     if (Brain.Screen.xPosition() < 240 && (Brain.Screen.yPosition() > auton1Rect.yCoordinate && Brain.Screen.yPosition() < auton1Rect.yCoordinate + auton1Rect.rectLength)) {
       autonSelected = 1;
     }
     else if (Brain.Screen.xPosition() > 240 && (Brain.Screen.yPosition() > auton2Rect.yCoordinate && Brain.Screen.yPosition() < auton2Rect.yCoordinate + auton2Rect.rectLength)) {
       autonSelected = 2;
     }
     else if (Brain.Screen.xPosition() < 240 && (Brain.Screen.yPosition() > auton3Rect.yCoordinate && Brain.Screen.yPosition() < auton3Rect.yCoordinate + auton3Rect.rectLength)) {
       autonSelected = 3;
     }
     else if (Brain.Screen.xPosition() > 240 && (Brain.Screen.yPosition() > auton4Rect.yCoordinate && Brain.Screen.yPosition() < auton4Rect.yCoordinate + auton4Rect.rectLength)) {
       autonSelected = 4;
     }
 
     // printing the current auton route selected
     Brain.Screen.setFillColor(transparent); // set fill color to transparent cuz it could be green from below if statements
 
     // if statement so if the auton is not 0 (there is an auton), then auton + auton # selected is printed to the brain (instead of none)
     if (autonSelected != 0) {
       // print AUTON instead of NONE if an auton is selected
       Brain.Screen.setCursor(11, 19);
       Brain.Screen.print("AUTON");
 
       // print auton program number to the brain next to AUTON and a space after AUTON
       Brain.Screen.setCursor(11, 25);
       Brain.Screen.print(autonSelected);
     }

     // change the selected auton's rectangle color to green
     if (autonSelected == 1) {
       // reset the other auton routes to their color then change this auton route rectangle to green
       // so multiple rectangles might not be green!!!
       initalizeAutonSelector(); // function that will do the above task
       wait(0.1, sec); // wait time so the rectangles won't flash a different color as well. So computer has time to fill the other rectangles in.
       
       Brain.Screen.setFillColor(green);
       Brain.Screen.drawRectangle(auton1Rect.xCoordinate, auton1Rect.yCoordinate, auton1Rect.rectWidth, auton1Rect.rectLength);
       Brain.Screen.setCursor(5,7);
       Brain.Screen.print("AUTON 1");
     }

     else if (autonSelected == 2) {
       initalizeAutonSelector();
       wait(0.1, sec);
       Brain.Screen.setFillColor(green);
       Brain.Screen.drawRectangle(auton2Rect.xCoordinate, auton2Rect.yCoordinate, auton2Rect.rectWidth, auton2Rect.rectLength); // x coord, y coord, width, height
       Brain.Screen.setCursor(5,34);
       Brain.Screen.print("AUTON 2");
     }

     else if (autonSelected == 3) {
       initalizeAutonSelector();
       wait(0.1, sec);
       Brain.Screen.setFillColor(green);
       Brain.Screen.drawRectangle(auton3Rect.xCoordinate, auton3Rect.yCoordinate, auton3Rect.rectWidth, auton3Rect.rectLength);
       Brain.Screen.setCursor(9,7);
       Brain.Screen.print("AUTON 3");
     }
     
     else if (autonSelected == 4) {
       initalizeAutonSelector();
       wait(0.1, sec);
       Brain.Screen.setFillColor(green);
       Brain.Screen.drawRectangle(auton4Rect.xCoordinate, auton4Rect.yCoordinate, auton4Rect.rectWidth, auton4Rect.rectLength);
       Brain.Screen.setCursor(9,34);
       Brain.Screen.print("AUTON 4");
     }
 
     // alliance color selection + green rectangle
     if (Brain.Screen.xPosition() > redButton.xCoordinate && Brain.Screen.xPosition() < redButton.xCoordinate + redButton.rectWidth && Brain.Screen.yPosition() < 50 && Brain.Screen.yPosition() > 10) {
       allianceColor = "RED";
 
       // reset other rectangles/buttons
       initalizeAllianceColorSelector();
       wait(0.1, sec);
 
       // fill in green for selected alliance color
       Brain.Screen.setFillColor(green);
       Brain.Screen.drawRectangle(redButton.xCoordinate, redButton.yCoordinate, redButton.rectWidth, redButton.rectLength);
      
       // write the "red" text in the button cuz otherwise the green fill will cover it up
       // (don't need to set fill color because the shape will be green anyways)
       Brain.Screen.setCursor(2, 23);
       Brain.Screen.print("RED");
 
       // fill transparent color so doesn't fill green or something + write color selected
       Brain.Screen.setFillColor(transparent);
      
       // say which alliance color is selected
       Brain.Screen.setCursor(11, 35);
       Brain.Screen.print("RED");
 
       // get rid of the "E" from NONE from the default set text for no alliance color selected
       Brain.Screen.setCursor(11, 38);
       Brain.Screen.print(" ");
     }
     else if (Brain.Screen.xPosition() > blueButton.xCoordinate && Brain.Screen.xPosition() < blueButton.xCoordinate + blueButton.rectWidth && Brain.Screen.yPosition() < 50 && Brain.Screen.yPosition() > 10) {
       allianceColor = "BLUE";
 
       // reset other rectangles/buttons
       initalizeAllianceColorSelector();
       wait(0.1, sec);
 
       // fill in green for selected alliance color
       Brain.Screen.setFillColor(green);
       Brain.Screen.drawRectangle(blueButton.xCoordinate, blueButton.yCoordinate, blueButton.rectWidth, blueButton.rectLength);
      
       // write the "blue" text in the button cuz otherwise the green fill will cover it up
       // (don't need to set fill color because the shape will be green anyways)
       Brain.Screen.setCursor(2, 33);
       Brain.Screen.print("BLUE");
 
       // fill transparent color so doesn't fill green or something + write color selected
       Brain.Screen.setFillColor(transparent);
 
       // say which alliance color is selected
       Brain.Screen.setCursor(11, 35);
       Brain.Screen.print("BLUE");
     }
 
 
     wait(0.00001, seconds); // a small wait for the infinite loop
   }
}

// roller spinning function


void pre_auton(void) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  // CALIBRATE the gyro and other sensors and stuff (also add as needed)
  Inertial.calibrate();

  // AUTON SCREEN SELECTOR STUFF 
  // initalizeAutonSelector(); // set the start rectangle conditions
  // initalizeAllianceColorSelector(); // set the start alliance color selection conditions

  // Brain.Screen.pressed(autonScreenSelector);
  autonSelected = 3;
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  // basic autonomous code for rollers SIDE STARTING ON ROLLER
  if (autonSelected == 1) {
    // LeftFrontMotor.spin(reverse, 4, voltageUnits::volt);//+ turnMotorPower (if turning). L/R for self-correction
    // RightFrontMotor.spin(fwd, 4, voltageUnits::volt);//- turnMotorPower
    // LeftBackMotor.spin(reverse, 4, voltageUnits::volt);//+ turnMotorPower
    // RightBackMotor.spin(reverse, 4, voltageUnits::volt);
    // IntakeMotor.spin(fwd, 8, voltageUnits::volt);

    // wait(0.25, sec);

    // LeftFrontMotor.stop();
    // LeftBackMotor.stop();
    // RightBackMotor.stop();
    // RightFrontMotor.stop();
    // IntakeMotor.stop();

    // move back
    // LeftFrontMotor.spin(reverse, 12, voltageUnits::volt);//+ turnMotorPower (if turning). L/R for self-correction
    // RightFrontMotor.spin(reverse, 12, voltageUnits::volt);//- turnMotorPower
    // LeftMiddleMotor.spin(reverse, 12, voltageUnits::volt);
    // RightMiddleMotor.spin(reverse, 12, voltageUnits::volt);
    // LeftBackMotor.spin(reverse, 12, voltageUnits::volt);//+ turnMotorPower
    // RightBackMotor.spin(reverse, 12, voltageUnits::volt);

    // trying to do it with degrees
    // LeftFrontMotor.spinFor(fwd, 500, rotationUnits::deg);//+ turnMotorPower (if turning). L/R for self-correction
    // RightFrontMotor.spinFor(reverse, 500, rotationUnits::deg);//- turnMotorPower
    // LeftMiddleMotor.spinFor(fwd, 500, rotationUnits::deg);
    // RightMiddleMotor.spinFor(reverse, 500, rotationUnits::deg);
    // LeftBackMotor.spinFor(fwd, 500, rotationUnits::deg);//+ turnMotorPower
    // RightBackMotor.spinFor(reverse, 500, rotationUnits::deg);

    // wait(0.1, sec);

    // LeftFrontMotor.stop();
    // RightFrontMotor.stop();
    // LeftMiddleMotor.stop();
    // RightMiddleMotor.stop();
    // LeftBackMotor.stop();
    // RightBackMotor.stop();


    // spin rollers
    // while (!(Optical.color() == vex::color::red)) {
    //   IntakeMotor.spin(fwd, 12, voltageUnits::volt);
    // }

    // TEST
    if (allianceColor == "RED") {
      while ((Optical.color() == vex::color::blue)) {
        IntakeMotor.spin(fwd, 12, volt);
        LeftFrontMotor.spin(fwd, 12, volt);
        RightFrontMotor.spin(reverse, 12, volt);
        LeftMiddleMotor.spin(fwd, 12, volt);
        RightMiddleMotor.spin(reverse, 12, volt);
        LeftBackMotor.spin(fwd, 12, volt);
        RightBackMotor.spin(reverse, 12, volt);
      }
    }
    else if (allianceColor == "BLUE") {
      while ((Optical.color() == vex::color::red)) {
        IntakeMotor.spin(fwd, 12, voltageUnits::volt);
        LeftFrontMotor.spin(fwd, 12, volt);
        RightFrontMotor.spin(reverse, 12, volt);
        LeftMiddleMotor.spin(fwd, 12, volt);
        RightMiddleMotor.spin(reverse, 12, volt);
        LeftBackMotor.spin(fwd, 12, volt);
        RightBackMotor.spin(reverse, 12, volt);
      }
    }

    // CODE RN

    // move back
    // LeftFrontMotor.spin(fwd, 12, volt);
    // RightFrontMotor.spin(reverse, 12, volt);
    // LeftMiddleMotor.spin(fwd, 12, volt);
    // RightMiddleMotor.spin(reverse, 12, volt);
    // LeftBackMotor.spin(fwd, 12, volt);
    // RightBackMotor.spin(reverse, 12, volt);

    // // spin rollers
    // IntakeMotor.spin(fwd, 12, volt);
    // wait(0.2, sec);
    // IntakeMotor.stop();


    // LeftFrontMotor.rotateFor(fwd, 200, deg);
    // RightFrontMotor.rotateFor(reverse, 200, deg);
    // LeftMiddleMotor.rotateFor(fwd, 200, deg);
    // RightMiddleMotor.rotateFor(reverse, 200, deg);
    // LeftBackMotor.rotateFor(fwd, 200, deg);
    // RightBackMotor.rotateFor(reverse, 200, deg);

    // IntakeMotor.spin(fwd, 12, volt);
    // wait(0.2, sec);
    // IntakeMotor.stop();
  }

  // basic autonomous code for rollers - other color than auton 1
  else if (autonSelected == 2) { // SIDE WITH NO ROLLER STARTING
    // Brain.Screen.print("Auton 2");
    // move back
    // LeftFrontMotor.spin(reverse, 12, voltageUnits::volt);//+ turnMotorPower (if turning). L/R for self-correction
    // RightFrontMotor.spin(reverse, 12, voltageUnits::volt);//- turnMotorPower
    // LeftMiddleMotor.spin(reverse, 12, voltageUnits::volt);
    // RightMiddleMotor.spin(reverse, 12, voltageUnits::volt);
    // LeftBackMotor.spin(reverse, 12, voltageUnits::volt);//+ turnMotorPower
    // RightBackMotor.spin(reverse, 12, voltageUnits::volt);
    // // IntakeMotor.spin(reverse, 8, voltageUnits::volt);

    // wait(0.1, sec);

    // LeftFrontMotor.stop();
    // RightFrontMotor.stop();
    // LeftMiddleMotor.stop();
    // RightMiddleMotor.stop();
    // LeftBackMotor.stop();
    // RightBackMotor.stop();
  
    // IntakeMotor.stop();

    // if (allianceColor == "RED") {
    //     while (!(Optical.color() == vex::color::blue)) {
    //       IntakeMotor.spin(fwd, 12, voltageUnits::volt);
    //     }
    //   }
    // else if (allianceColor == "BLUE") {
    //   while (!(Optical.color() == vex::color::red)) {
    //     IntakeMotor.spin(fwd, 12, voltageUnits::volt);
    //   }
    // }
    // 
    // while (!(Optical.color() == vex::color::blue)) {
    //   IntakeMotor.spin(fwd, 12, voltageUnits::volt);
    // }

    // AUTON

    // drive forward
    // drivePID(300, 1);

    // // turn 90 degrees to rollers
    // turnPID(90, 1);

    // // spin the rollers
    // if (allianceColor == "RED") {
    //   while ((Optical.color() == vex::color::blue)) {
    //     IntakeMotor.spin(fwd, 12, voltageUnits::volt);
    //   }
    // }
    // else if (allianceColor == "BLUE") {
    //   while ((Optical.color() == vex::color::red)) {
    //     IntakeMotor.spin(fwd, 12, voltageUnits::volt);
    //   }
    // }

    drivePID(400, 1);
    // expand
    for (int i = 0; i < 10; i++) {
      // expand
      Exp1.off();
      Exp2.off();

      wait(0.3, sec);
      // piston back up
      Exp1.on();
      Exp2.on();
      wait(0.3, sec);
    }

    // other stuff

  } 
  else if (autonSelected == 3) {
    // Brain.Screen.print("Auton 3");
    Brain.Screen.print("hi");
    // LeftFrontMotor.spinFor(reverse, 3000, rotationUnits::deg);//+ turnMotorPower (if turning). L/R for self-correction
    // RightFrontMotor.spinFor(reverse, 3000, rotationUnits::deg);//- turnMotorPower
    // LeftMiddleMotor.spinFor(reverse, 3000, rotationUnits::deg);
    // RightMiddleMotor.spinFor(reverse, 3000, rotationUnits::deg);
    // LeftBackMotor.spinFor(reverse, 3000, rotationUnits::deg);//+ turnMotorPower
    // RightBackMotor.spinFor(reverse, 3000, rotationUnits::deg);

    // LeftFrontMotor.spin(reverse, 12, volt);
    // RightFrontMotor.spin(fwd, 12, volt);
    // LeftMiddleMotor.spin(reverse, 12, volt);
    // RightMiddleMotor.spin(fwd, 12, volt);
    // LeftBackMotor.spin(reverse, 12, volt);
    // RightBackMotor.spin(fwd, 12, volt);

    // wait(0.2, sec);

    // LeftFrontMotor.stop();
    // RightFrontMotor.stop();
    // LeftMiddleMotor.stop();
    // RightMiddleMotor.stop();
    // LeftBackMotor.stop();
    // RightBackMotor.stop();

    // rip auton shoots in opponent's high goal, can spool fly up less, wait less, etc. but not really reliable unless under
    // FlywheelMotor.spin(reverse, 12, volt);
    // wait(2, sec);
    // IntakeMotor.spin(reverse, 9, volt);

    // drivePID(200);
    // turnPID(90, 1);

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
  else if (autonSelected == 4) {
    Brain.Screen.print("SKILLS");

    // rotate before expansion
    //turnPID(45, 1);

    // move to middle of field
    // drivePID(3000);


    // // wait for expansion
    // wait(50, sec);

    // // expansion
    // Exp1.off();
    // wait(1, sec);
    // Exp2.off();

    // // drive back to a corner
    // drivePID(-3000);


    // testing driving
    // drivePID(600);
    // turnPID(-90, 1);


    // expansion skills

    // rollers

    // turn and move

    // drive forward a little bit
    // drivePID(60);

    // expand
    // Exp1.off();
    // Exp2.off();

    // drive back a little bit
    // drivePID(-25);

    // SKILLS
    // LeftFrontMotor.spin(reverse, 12, volt);
    // RightFrontMotor.spin(fwd, 12, volt);
    // LeftMiddleMotor.spin(reverse, 12, volt);
    // RightMiddleMotor.spin(fwd, 12, volt);
    // LeftBackMotor.spin(reverse, 12, volt);
    // RightBackMotor.spin(fwd, 12, volt);

    // wait(0.2, sec);

    // LeftFrontMotor.stop();
    // RightFrontMotor.stop();
    // LeftMiddleMotor.stop();
    // RightMiddleMotor.stop();
    // LeftBackMotor.stop();
    // RightBackMotor.stop();

    // Exp1.off();
    // Exp2.off();



    // NEW PROG SKILLS
    
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
    wait(0.3, sec);
    IntakeMotor.stop();
    FlywheelMotor.stop();

    // move forward away from rollers a bit
    drivePID(100, 1);

    // turn in line with the white line
    turnPID(45, 1);

    // drive fwd
    drivePID(650, 1);

    // shoot discs
    turnPID(-7, 1);

    // spin flywheel motor
    FlywheelMotor.spin(reverse, 12, volt);

    Brain.Screen.clearScreen();

    // // once hits 270 ish then spin
    while (FlywheelMotor.velocity(rpm) > -260) {
      // piston
      // Piston.on();
      Brain.Screen.setCursor(5, 2);
      Brain.Screen.print(FlywheelMotor.velocity(rpm));
    }

    IntakeMotor.spin(reverse, 12, volt);

    wait(3.8, sec);
    IntakeMotor.stop();
    FlywheelMotor.stop();

    // // firing 2 discs
    // wait(0.1, sec);
    // Piston.off();
    // Piston.on();
    // wait(0.1, sec);
    // Piston.off();

    // // turn intake facing 3 stack
    turnPID(-130, 1); // 315

    // // forward and intake
    // IntakeMotor.spin(fwd, 12, volt);

    // drivePID(-650, 0.10, true); // attempt for PID + intake - didn't really work
    // but got one disc but then it stopped since that's what PID does.


    LeftFrontMotor.spin(reverse, 2.5, volt);
    RightFrontMotor.spin(reverse, 2.5, volt);
    LeftMiddleMotor.spin(reverse, 2.5, volt);
    RightMiddleMotor.spin(reverse, 2.5, volt);
    LeftBackMotor.spin(reverse, 2.5, volt);
    RightBackMotor.spin(reverse, 2.5, volt);
    IntakeMotor.spin(fwd, 12, volt);



    wait(4.23, sec);

    // IntakeMotor.spin(reverse, 12, volt);
    // wait(0.375, sec);

    LeftFrontMotor.stop();
    RightFrontMotor.stop();
    LeftMiddleMotor.stop();
    RightMiddleMotor.stop();
    LeftBackMotor.stop();
    RightBackMotor.stop();
    IntakeMotor.stop();

    turnPID(-20, 1);

    // shoot
    // // once hits 270 ish then spin
    while (FlywheelMotor.velocity(rpm) > -270) {
      // piston
      // Piston.on();
      Brain.Screen.setCursor(5, 2);
      Brain.Screen.print(FlywheelMotor.velocity(rpm));
    }

    IntakeMotor.spin(reverse, 12, volt);

    wait(5.5, sec);
    IntakeMotor.stop();
    FlywheelMotor.stop();


    // turn back
    turnPID(50, 1);

    // go back
    drivePID(1370, 1);

    // expand
    for (int i = 0; i < 5; i++) {
      // expand
      Exp1.off();
      Exp2.off();

      wait(0.3, sec);
      // piston back up
      Exp1.on();
      Exp2.on();
      wait(0.3, sec);
    }

  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  // int autoRollers = true;
  // VARIABLES
    double turnImportance = 1; // for changing the turn speed faster I think
    double speed = 1; // changing speed

  while (true) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
  
    // DRIVETRAIN CODE
    double turnVal = Controller1.Axis1.position();
    double forwardVal = Controller1.Axis3.position();
    double turnVolts = (turnVal * 0.12 * speed); // multiplying the -1 so it goes the other way since it was inverted.
    double forwardVolts = (forwardVal * 0.12 * (1 - (fabs(turnVolts)/12.0) * turnImportance) * speed);

    // LeftFrontMotor.spin(fwd, forwardVolts + turnVolts, volt);
    // RightFrontMotor.spin(fwd, forwardVolts - turnVolts, volt);
    // LeftMiddleMotor.spin(fwd, forwardVolts + turnVolts, volt);
    // RightMiddleMotor.spin(fwd, forwardVolts - turnVolts, volt);
    // LeftBackMotor.spin(fwd, forwardVolts + turnVolts, volt);
    // RightBackMotor.spin(fwd, forwardVolts - turnVolts, volt);

    // switched from above lines since forward stick did turns and vice versa.
    LeftFrontMotor.spin(fwd, forwardVolts + turnVolts, volt);
    RightFrontMotor.spin(fwd, forwardVolts - turnVolts, volt);
    LeftMiddleMotor.spin(fwd, forwardVolts + turnVolts, volt);
    RightMiddleMotor.spin(fwd, forwardVolts - turnVolts, volt);
    LeftBackMotor.spin(fwd, forwardVolts + turnVolts, volt);
    RightBackMotor.spin(fwd, forwardVolts - turnVolts, volt);

    
    // BUTTONS
    
    // INTAKE (L1/R1)
    if (Controller1.ButtonL1.pressing()) {
      IntakeMotor.spin(fwd, 12, volt);
      // RollerMotor.spin(fwd, 12, volt);
    } else if (Controller1.ButtonR2.pressing()) 
    { // indexer
      IntakeMotor.spin(reverse, 12, volt);
      // RollerMotor.spin(reverse, 12, volt);
    } else {
      IntakeMotor.stop();
      // RollerMotor.stop();
    }

    // SHOOTING - B
    if (Controller1.ButtonR1.pressing()) {
      // shoot stuff then once its shot back, might wanna reset or something so ye
      FlywheelMotor.spin(reverse, flywheelPower, voltageUnits::volt);
      // FlywheelMotor2.spin(fwd, 12, voltageUnits::volt);
    } 
    // else if (Controller1.ButtonUp.pressing()) {
    //   FlywheelMotor.spin(fwd, 2, volt);
    // } 
    else {
      FlywheelMotor.stop();
      // FlywheelMotor2.stop();
    }

    // if (Controller1.ButtonL1.pressing()) {
    //   if (pistonExtended) {
    //     Piston.off();
    //   } else {
    //     Piston.on();
    //   }
    // }

    // if (Controller1.ButtonDown.pressing()) {
    //   if (flywheelPower == 8) flywheelPower = 12;
    //   if (flywheelPower == 12) flywheelPower = 8;
    // }

    // Left arrow - expansion
    // if (Controller1.ButtonLeft.pressing()) {
    //   // Expansion
    //   ExpansionPneumatics.off();
    // }

    // X - manual rollers (left arrow = manual override for turret + LED shows state)
    // if (Controller1.ButtonX.pressing()) {
    //   RollerMotor.spin(fwd, 12, volt);
    // } else {
    //   RollerMotor.stop();
    // }

    // AUTO-ROLLERS - Button X
    // allianceColor = "BLUE";
    // allianceColor = "RED";
    // if (Controller1.ButtonX.pressing()) {
    //   if (allianceColor == "RED") {
    //      while ((Optical.color() == vex::color::blue)) {
    //        IntakeMotor.spin(fwd, 12, voltageUnits::volt);
    //      }
    //    }
    //    else if (allianceColor == "BLUE") {
    //      while ((Optical.color() == vex::color::red)) {
    //        IntakeMotor.spin(fwd, 12, voltageUnits::volt);
    //      }
    //    }
    // }
      // if (autoRollers) {
        // allianceColor = "RED";
        // if (allianceColor == "RED") { // red allianceColor == "RED"
        //   while ((Optical.color() == vex::color::blue)) {
        //     IntakeMotor.spin(fwd, 12, voltageUnits::volt);
        //   }
        // }
        // else if (allianceColor == "BLUE") { // blue
        //   while ((Optical.color() == vex::color::red)) {
        //     IntakeMotor.spin(fwd, 12, voltageUnits::volt);
        //   }
        // }

        // testing auton // red allianceColor == "RED"
          // while ((Optical.color() == vex::color::blue)) {
          //   IntakeMotor.spin(fwd, 12, voltageUnits::volt);
          // }
          // while (Optical.color() == vex::color::red) {
          //   IntakeMotor.spin(fwd, 12, volt);
          // }
        
        // Brain.Screen.print(Optical.color());

      // }
      // else if (autoRollers == false) {
      //   IntakeMotor.spin(fwd, 12, voltageUnits::volt);
      // }
    // }

    // Disable auto-rollers - Button Right (might replace with button up if do turret)
    // if (Controller1.ButtonRight.pressing()) {
    //   autoRollers = !autoRollers;
    // } 

    // Y - parking brake if we have one
    // if (Controller1.ButtonY.pressing()) {
      // pnemuatic parking break if we have something like that (like makes robot off ground maybe or something.
    // }
    // expansion
    if (Controller1.ButtonY.pressing()) {
      // Exp1.off();
      // Exp2.off();

      if (expand == false) {
        Exp1.off();
        Exp2.off();
        expand = true;
      } else {
        Exp1.on();
        Exp2.on();
        expand = false;
      }
    }

    // L2/R2 - turret (down arrow = manual override for turret + LED shows state)

    // other code HERE
    
    // test printing to terminal
    // std::cout <<  
    // FlywheelMotor.velocity(rpm) << " , " <<  FlywheelMotor.torque(Nm) << " , " 
    // << FlywheelMotor.current() << " , " << FlywheelMotor.voltage(volt)
    // << std::endl; 

    // update flywheel power
    if (Controller1.ButtonDown.pressing()) {
      if (flywheelPower == 12) {
        flywheelPower = 10;
      }
      else if (flywheelPower == 10) {
        flywheelPower = 9;
      }
      else if (flywheelPower == 9) {
        flywheelPower = 7;
      }
      else if (flywheelPower == 7) {
        flywheelPower = 12;
      }
    }

    // manage printing flywheel power to brain
    // Controller1.Screen.clearScreen();
    // Controller1.Screen.setCursor(1,1);
    // Controller1.Screen.print("Battery: ");
    // Controller1.Screen.setCursor(1, 10);
    // Controller1.Screen.print(Brain.Battery.capacity()); // gets the percentage left of the battery

    // // rollers
    // Controller1.Screen.setCursor(2,1);
    // Controller1.Screen.print("Rollers: ");
    // Controller1.Screen.setCursor(2,10);
    // Controller1.Screen.print("A");

    // flywheel
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("Flywheel: ");
    Controller1.Screen.setCursor(3,11);
    Controller1.Screen.print(flywheelPower);

    // flywheel velocity
    Controller1.Screen.setCursor(3, 14);
    Controller1.Screen.print(FlywheelMotor.velocity(rpm));

    if (flywheelPower != 12 && flywheelPower!= 10) {
      Controller1.Screen.setCursor(3, 12);
      Controller1.Screen.print(" ");
    }

    // slowmode
    // if (Controller1.ButtonUp.pressing()) {
    //   if (speed != 1) {
    //     speed = 0.5;
    //   }
    //   else {
    //     speed = 1;
    //   }
    // }

    if (Controller1.ButtonRight.pressing()) {
      if (speed == 1) speed = 0.25;
      else speed = 1;
    }

    // vex::task t1(flywheelPID);
    // flywheelPID();

    // run TBH task
    // vex::task flywheelTBH(TBH);
    // do we need to start the TBH task?


    // auto vibrate flywheel when around 220 rpm
    if (FlywheelMotor.velocity(rpm) <= -220 && FlywheelMotor.velocity(rpm) >= -240) {
      Controller1.rumble(rumbleShort);
      // led on
      // LEDG.on();
    } 
    // Controller1.rumble(rumbleShort);

    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
