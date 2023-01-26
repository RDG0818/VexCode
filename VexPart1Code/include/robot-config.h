using namespace vex;

extern brain Brain;

// VEXcode devices
extern inertial Inertia;
extern controller Controller1;
extern motor Intake;
extern digital_out DigitalOutA;
extern digital_out DigitalOutB;
extern motor Catapult;
extern gps GPS7;
extern motor RightMiddleMotor;
extern motor LeftMiddleMotor;
extern motor RightFrontMotor;
extern motor LeftFrontMotor;
extern motor RightBackMotor;
extern motor LeftBackMotor;
extern limit LimitSwitchC;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );