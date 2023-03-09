#include "vex.h"

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