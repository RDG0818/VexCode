#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RightFrontMotor = motor(PORT1, ratio18_1, true);
motor RightBackMotor = motor(PORT2, ratio18_1, true);
motor LeftBackMotor = motor(PORT3, ratio18_1, false);
motor LeftFrontMotor = motor(PORT4, ratio18_1, false);
inertial Inertia = inertial(PORT20);
controller Controller1 = controller(primary);
motor Intake = motor(PORT5, ratio36_1, true);
motor Flywheel = motor(PORT6, ratio6_1, true);
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);
motor Expansion = motor(PORT7, ratio18_1, true);
motor TubeTurner = motor(PORT8, ratio36_1, false);

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