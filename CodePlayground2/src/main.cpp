// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightMotors          motor_group   1, 2            
// LeftMotors           motor_group   3, 4            
// Inertia              inertial      20              
// Controller1          controller                    
// Intake               motor         5               
// Flywheel             motor         6               
// DigitalOutA          digital_out   A               
// TubeTurner           motor         8               
// DigitalOutB          digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
using namespace vex;

// A global instance of competition
competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Inertia.calibrate();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                             Global Functions                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void SetSpeed (double SpeedPercent) {
  RightMotors.setVelocity(SpeedPercent, percent);
  LeftMotors.setVelocity(SpeedPercent, percent);
}

void LineDrive (double Distance, int x) {
  SetSpeed(x);
  RightMotors.spinFor(forward, Distance*28.33, degrees, false);
  LeftMotors.spinFor(forward, Distance*28.33, degrees);
}

void resetSensor (){
  LeftMotors.setPosition(0, degrees);
  RightMotors.setPosition(0, degrees);
}

void Stop () {
  RightMotors.stop();
  LeftMotors.stop();
}


void TurnRight(double x) {
while (Inertia.rotation(degrees)<x){
  RightMotors.spin(reverse);
  LeftMotors.spin(forward);
  wait(0.02, seconds);
}
RightMotors.stop();
LeftMotors.stop();
wait(0.2, seconds);
if (Inertia.rotation(degrees)>x+1) {
  SetSpeed(15);
  RightMotors.spinFor(forward, (Inertia.rotation(degrees)-x)*3.82, degrees, false);
  LeftMotors.spinFor(reverse, (Inertia.rotation(degrees)-x)*3.82, degrees, false);
}
}

void TurnLeft(double x) {
while (Inertia.rotation(degrees)>x){
  RightMotors.spin(forward);
  LeftMotors.spin(reverse);
  wait(0.02, seconds);
}
RightMotors.stop();
LeftMotors.stop();
wait(0.2, seconds);
if (Inertia.rotation(degrees)<x-1) {
  SetSpeed(15);
  RightMotors.spinFor(reverse, (x-Inertia.rotation(degrees))*3.82, degrees, false);
  LeftMotors.spinFor(forward, (x-Inertia.rotation(degrees))*3.82, degrees, false);
}
}

void TurnPID (double Setpoint) {
  // Proportional Constants
  // Tune the constants, when to start integral, and when to stop integral/PID
  double integral = 0;
  double prevError = Setpoint;
  double kP = 5;
  double kI = 0.1;
  double kD = 0.2;
  int x = 0;
  resetSensor();
  while (x<20){
    double CurrentPosition = Inertia.rotation(degrees);
    double error = Setpoint - CurrentPosition;
    double derivative = error - prevError;
    prevError = error;
    integral = integral + error;
    // The next line is when to stop the PID/Integral
    if (-0.1<error<0.1){
      x=x+1;
      integral = 0;
      // The next line is when to start the integral
    } else if (error>90){
      integral = 0;
    }
    SetSpeed(error*kP + integral*kI + derivative*kD);
    RightMotors.spin(forward);
    LeftMotors.spin(forward);
    // Adds timer for amount of time close to 0 error
    wait(10, msec);
  }
}

void LateralPID (double Setpoint) {
  // Proportional Constants
  // Tune the constants, when to start integral, and when to stop integral/PID
  double integral = 0;
  double prevError = Setpoint;
  double kP = 5;
  double kI = 0.1;
  double kD = 0.2;
  int x = 0;
  resetSensor();
  while (x<20){
    double CurrentPosition = (RightMotors.position(turns)+LeftMotors.position(turns))*12.56636;
    double error = Setpoint - CurrentPosition;
    double derivative = error - prevError;
    prevError = error;
    integral = integral + error;
    // The next line is when to stop the PID/Integral
    if (-0.1<error<0.1){
      x=x+1;
      integral = 0;
      // The next line is when to start the integral
    } else if (error>24){
      integral = 0;
    }
    SetSpeed(error*kP + integral*kI + derivative*kD);
    RightMotors.spin(forward);
    LeftMotors.spin(forward);
    // Adds timer for amount of time close to 0 error
    wait(10, msec);
  }
}

bool FlywheelOn = true;
int DesiredVelocity = 3000;
int FlywheelAlgorithm(){
  //Tune Proportional Constants
  double kP = 0.1;
  int output = 0;
  int prevError = -1;
  int tbh = DesiredVelocity;
  while (FlywheelOn){
    int ActualVelocity = (Flywheel.velocity(rpm))*6;
    int error = DesiredVelocity-ActualVelocity;
    output = output + error*kP;
    if (signbit(error) != signbit(prevError)){
      output = 0.5 * (output + tbh);           
      tbh = output;                             
      prevError = error; 
    }
    Flywheel.spin(forward, output/300.0, voltageUnits::volt);
    vex::task::sleep (15);
  }
  return 1;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
task Mytask = task(FlywheelAlgorithm);
RightMotors.setStopping(coast);
LeftMotors.setStopping(coast);
Flywheel.setVelocity(60, percent);
Flywheel.spin(forward);
wait(1000, msec);
DigitalOutA.set(true);
wait(500, msec);
DigitalOutA.set(false);
wait(500, msec);
LineDrive(.5, 25);
LineDrive(-.5, 25);
DigitalOutA.set(true);
wait(500, msec);
DigitalOutA.set(false);
wait(500, msec);
Flywheel.stop();
LineDrive(26, 25);
TurnRight(82);
LineDrive(4.5, 25);
TubeTurner.spinFor(reverse, 260, degrees);
LineDrive(-1.5, 25);
TurnRight(200);
Intake.setVelocity(100, percent);
Intake.spin(forward);
LineDrive(72, 45);
TurnLeft(150);
FlywheelOn = false;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  FlywheelOn = true;
  while (true) {
    RightMotors.setStopping(coast);
    LeftMotors.setStopping(coast);
    Flywheel.setStopping(coast);
    Intake.setVelocity(100, percent);
    TubeTurner.setMaxTorque(100, percent);

    float LeftMotorSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
    float RightMotorSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();

    LeftMotors.setVelocity(pow(LeftMotorSpeed/100.0, 3)*100, percent);
    RightMotors.setVelocity(pow(RightMotorSpeed/100.0, 3)*100, percent);

    RightMotors.spin(forward);
    LeftMotors.spin(forward);

    if (Controller1.ButtonL2.pressing()){
      DesiredVelocity = 3600;
    } else {
      DesiredVelocity = 3000;
    }

    if (Controller1.ButtonDown.pressing() && Controller1.ButtonL2.pressing()) {
      DigitalOutA.set(true);
      DigitalOutB.set(true);
    } 

    if (Controller1.ButtonR2.pressing()){
      Intake.spin(forward);
    } else if (Controller1.ButtonR1.pressing()){
      Intake.spin(reverse);
    } else {
      Intake.stop();
    }

    if (Controller1.ButtonUp.pressing()){
      TubeTurner.spin(reverse);
    } else {
      Intake.stop();
    }
    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }

}
