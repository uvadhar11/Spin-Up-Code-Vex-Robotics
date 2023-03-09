// files to link

// Autons
void auton1();
void auton2();
void auton3();
void auton4();


// Function Files Being Used
void drivePID(int desiredValue, double multiplier);
void drivePID(int desiredValue, double multiplier, bool intakeWhile);
void turnPID(int desiredValue, double multiplier);


// Other functions not currently being used
int flywheelPID();
int TBH();
int odometry();