#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RightMotorsMotorA = motor(PORT1, ratio18_1, false);
motor RightMotorsMotorB = motor(PORT2, ratio18_1, false);
motor_group RightMotors = motor_group(RightMotorsMotorA, RightMotorsMotorB);
motor LeftMotorsMotorA = motor(PORT3, ratio18_1, true);
motor LeftMotorsMotorB = motor(PORT4, ratio18_1, true);
motor_group LeftMotors = motor_group(LeftMotorsMotorA, LeftMotorsMotorB);
inertial Inertia = inertial(PORT20);
controller Controller1 = controller(primary);
motor Intake = motor(PORT5, ratio36_1, true);
motor Flywheel = motor(PORT6, ratio6_1, true);
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);
motor TubeTurner = motor(PORT8, ratio36_1, false);
digital_out DigitalOutB = digital_out(Brain.ThreeWirePort.B);

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