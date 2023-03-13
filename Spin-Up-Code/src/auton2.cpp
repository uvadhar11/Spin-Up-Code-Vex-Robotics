#include "vex.h"
using namespace std;
#include <iostream>
#include "linking.h"

void auton2() {
  // simple drive and expand skills
   drivePID(400, 1);
    // expand
    for (int i = 0; i < 2; i++) {
      // expand
      Exp1.off();
      Exp2.off();

      wait(0.3, sec);
      // piston back up
      Exp1.on();
      Exp2.on();
      wait(0.3, sec);
    }


  // other rollers and ye

}