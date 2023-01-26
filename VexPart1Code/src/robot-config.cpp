#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
inertial Inertia = inertial(PORT10);
controller Controller1 = controller(primary);
motor Intake = motor(PORT11, ratio6_1, true);
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);
digital_out DigitalOutB = digital_out(Brain.ThreeWirePort.B);
motor Catapult = motor(PORT20, ratio36_1, false);
gps GPS7 = gps(PORT7, 0.00, 0.00, mm, 180);
motor RightMiddleMotor = motor(PORT19, ratio18_1, false);
motor LeftMiddleMotor = motor(PORT5, ratio18_1, true);
motor RightFrontMotor = motor(PORT1, ratio18_1, true);
motor LeftFrontMotor = motor(PORT3, ratio18_1, false);
motor RightBackMotor = motor(PORT17, ratio18_1, true);
motor LeftBackMotor = motor(PORT18, ratio18_1, false);
limit LimitSwitchC = limit(Brain.ThreeWirePort.C);

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