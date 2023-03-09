#include "vex.h"

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