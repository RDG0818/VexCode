using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group RightMotors;
extern motor_group LeftMotors;
extern inertial Inertia;
extern controller Controller1;
extern motor Intake;
extern motor Flywheel;
extern digital_out DigitalOutA;
extern motor TubeTurner;
extern digital_out DigitalOutB;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );