#include "vex.h"

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