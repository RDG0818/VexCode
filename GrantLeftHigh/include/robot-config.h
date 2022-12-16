using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RightFrontMotor;
extern motor RightBackMotor;
extern motor LeftBackMotor;
extern motor LeftFrontMotor;
extern inertial Inertia;
extern controller Controller1;
extern digital_out DigitalOutA;
extern motor Intake;
extern motor Flywheel;
extern motor Expansion;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );