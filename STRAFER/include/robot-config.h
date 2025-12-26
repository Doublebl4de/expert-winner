using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor leftFront;
extern motor rightFront;
extern motor leftBack;
extern motor rightBack;
extern controller Controller1;
extern inertial bigInertial;
extern motor center1;
extern motor center2;
extern motor lowTake;
extern motor highTake;
extern digital_out midScore;
extern digital_out dscore;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
