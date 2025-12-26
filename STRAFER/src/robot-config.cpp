#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftFront = motor(PORT20, ratio6_1, true);
motor rightFront = motor(PORT18, ratio6_1, false);
motor leftBack = motor(PORT19, ratio6_1, true);
motor rightBack = motor(PORT17, ratio6_1, false);
controller Controller1 = controller(primary);
inertial bigInertial = inertial(PORT1);
motor center1 = motor(PORT14, ratio18_1, true);
motor center2 = motor(PORT15, ratio18_1, false);
motor lowTake = motor(PORT11, ratio6_1, false);
motor highTake = motor(PORT12, ratio6_1, false);
digital_out midScore = digital_out(Brain.ThreeWirePort.A);
digital_out dscore = digital_out(Brain.ThreeWirePort.D);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
