/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightFrontMotor      motor         1               
// RightBackMotor       motor         2               
// LeftBackMotor        motor         3               
// LeftFrontMotor       motor         4               
// Inertia              inertial      20              
// Controller1          controller                    
// DigitalOutA          digital_out   A               
// Intake               motor         5               
// Flywheel             motor         6               
// Expansion            motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void SetSpeedDriving (double SpeedPercent) {
RightFrontMotor.setVelocity(SpeedPercent, percent);
RightBackMotor.setVelocity(SpeedPercent, percent);
LeftFrontMotor.setVelocity(SpeedPercent, percent);
LeftBackMotor.setVelocity(SpeedPercent, percent);
}

void TurnRightHeading (double Heading, double Speed) {
  RightFrontMotor.setVelocity(Speed, percent);
  RightBackMotor.setVelocity(Speed, percent);
  LeftFrontMotor.setVelocity(Speed, percent);
  LeftBackMotor.setVelocity(Speed, percent);
  if (Heading-Inertia.heading(degrees)>1){
RightFrontMotor.spinFor(reverse, 3.8*(Heading-Inertia.heading(degrees)), degrees, false);
RightBackMotor.spinFor(reverse, 3.8*(Heading-Inertia.heading(degrees)), degrees, false);
LeftFrontMotor.spinFor(forward, 3.8*(Heading-Inertia.heading(degrees)), degrees, false);
LeftBackMotor.spinFor(forward, 3.8*(Heading-Inertia.heading(degrees)), degrees);
  }
  else{
    RightFrontMotor.spinFor(reverse, 3.8*((Heading-Inertia.heading(degrees))+360), degrees, false);
RightBackMotor.spinFor(reverse, 3.8*((Heading-Inertia.heading(degrees))+360), degrees, false);
LeftFrontMotor.spinFor(forward, 3.8*((Heading-Inertia.heading(degrees))+360), degrees, false);
LeftBackMotor.spinFor(forward, 3.8*((Heading-Inertia.heading(degrees))+360), degrees);
  }
}

void TurnLeftHeading (double Heading, double Speed) {
  RightFrontMotor.setVelocity(Speed, percent);
  RightBackMotor.setVelocity(Speed, percent);
  LeftFrontMotor.setVelocity(Speed, percent);
  LeftBackMotor.setVelocity(Speed, percent);
  if (Inertia.heading(degrees) - Heading>1){
RightFrontMotor.spinFor(forward, 3.8*(Inertia.heading(degrees) - Heading), degrees, false);
RightBackMotor.spinFor(forward, 3.8*(Inertia.heading(degrees) - Heading), degrees, false);
LeftFrontMotor.spinFor(reverse, 3.8*(Inertia.heading(degrees) - Heading), degrees, false);
LeftBackMotor.spinFor(reverse, 3.8*(Inertia.heading(degrees) - Heading), degrees);
  }
  else{
  RightFrontMotor.spinFor(forward, 3.8*((Inertia.heading(degrees) - Heading)+360), degrees, false);
RightBackMotor.spinFor(forward, 3.8*((Inertia.heading(degrees) - Heading)+360), degrees, false);
LeftFrontMotor.spinFor(reverse, 3.8*((Inertia.heading(degrees) - Heading)+360), degrees, false);
LeftBackMotor.spinFor(reverse, 3.8*((Inertia.heading(degrees) - Heading)+360), degrees);
  }
}

void TurnRight(double x) {
while (Inertia.rotation(degrees)<x){
  RightFrontMotor.spin(reverse);
  RightBackMotor.spin(reverse);
  LeftFrontMotor.spin(forward);
  LeftBackMotor.spin(forward);
  wait(0.02, seconds);
}
RightFrontMotor.stop();
LeftFrontMotor.stop();
RightBackMotor.stop();
LeftBackMotor.stop();
wait(0.2, seconds);
if (Inertia.rotation(degrees)>x+1) {
  SetSpeedDriving(15);
  RightFrontMotor.spinFor(forward, (Inertia.rotation(degrees)-x)*3.82, degrees, false);
  RightBackMotor.spinFor(forward, (Inertia.rotation(degrees)-x)*3.82, degrees, false);
  LeftFrontMotor.spinFor(reverse, (Inertia.rotation(degrees)-x)*3.82, degrees, false);
  LeftBackMotor.spinFor(reverse, (Inertia.rotation(degrees)-x)*3.82, degrees);
}
}

void TurnLeft(double x) {
while (Inertia.rotation(degrees)>x){
  RightFrontMotor.spin(forward);
  RightBackMotor.spin(forward);
  LeftFrontMotor.spin(reverse);
  LeftBackMotor.spin(reverse);
  wait(0.02, seconds);
}
RightFrontMotor.stop();
LeftFrontMotor.stop();
RightBackMotor.stop();
LeftBackMotor.stop();
wait(0.2, seconds);
if (Inertia.rotation(degrees)<x-1) {
  SetSpeedDriving(15);
  RightFrontMotor.spinFor(reverse, (x-Inertia.rotation(degrees))*3.82, degrees, false);
  RightBackMotor.spinFor(reverse, (x-Inertia.rotation(degrees))*3.82, degrees, false);
  LeftFrontMotor.spinFor(forward, (x-Inertia.rotation(degrees))*3.82, degrees, false);
  LeftBackMotor.spinFor(forward, (x-Inertia.rotation(degrees))*3.82, degrees);
}
}

void resetSensor (){
    LeftBackMotor.setPosition(0, degrees);
    RightBackMotor.setPosition(0, degrees);
    LeftFrontMotor.setPosition(0, degrees);
    RightFrontMotor.setPosition(0, degrees);
  }


void Stop () {
      RightFrontMotor.stop();
      LeftFrontMotor.stop();
      RightBackMotor.stop();
      LeftBackMotor.stop();
}

double StartingSpeed = 0;

void Acceleration (double Distance, double MaxSpeed) {
  resetSensor();
    while (RightFrontMotor.position(degrees)<Distance*28.33){
      StartingSpeed += 2;
      if (StartingSpeed>MaxSpeed){
        StartingSpeed=MaxSpeed;
      }
      RightFrontMotor.setVelocity(StartingSpeed, percent);
      LeftFrontMotor.setVelocity(StartingSpeed, percent);
      RightBackMotor.setVelocity(StartingSpeed, percent);
      LeftBackMotor.setVelocity(StartingSpeed, percent);
      RightFrontMotor.spin(forward);
      LeftFrontMotor.spin(forward);
      RightBackMotor.spin(forward);
      LeftBackMotor.spin(forward);
      wait(.02, seconds);
    }
    Stop();
}

void AccelerationBackwards (double Distance, double MaxSpeed) {
  resetSensor();
    while (RightFrontMotor.position(degrees)>-1*Distance*28.33){
      StartingSpeed += 1;
      if (StartingSpeed>MaxSpeed){
        StartingSpeed=MaxSpeed;
      }
      RightFrontMotor.setVelocity(StartingSpeed, percent);
      LeftFrontMotor.setVelocity(StartingSpeed, percent);
      RightBackMotor.setVelocity(StartingSpeed, percent);
      LeftBackMotor.setVelocity(StartingSpeed, percent);
      RightFrontMotor.spin(reverse);
      LeftFrontMotor.spin(reverse);
      RightBackMotor.spin(reverse);
      LeftBackMotor.spin(reverse);
      wait(.02, seconds);
    }
    Stop();
}

void DriveForward (double Distance, int x) {
        RightFrontMotor.setVelocity(x, percent);
      LeftFrontMotor.setVelocity(x, percent);
      RightBackMotor.setVelocity(x, percent);
      LeftBackMotor.setVelocity(x, percent);
  RightFrontMotor.spinFor(forward, Distance*28.33, degrees, false);
      LeftFrontMotor.spinFor(forward, Distance*28.33, degrees, false);
      RightBackMotor.spinFor(forward, Distance*28.33, degrees, false);
      LeftBackMotor.spinFor(forward, Distance*28.33, degrees);
}
double y = 25;


// Should always be followed by stopping all motors.
// Should always be preceded by double StartingSpeed = 0;
void autonomous(void) {
RightFrontMotor.setStopping(coast);
RightBackMotor.setStopping(coast);
LeftFrontMotor.setStopping(coast);
LeftBackMotor.setStopping(coast);
Intake.setVelocity(100, percent);
DriveForward (2, 25);
Intake.spinFor(reverse, 270, degrees);
DriveForward (-3, 25);
RightFrontMotor.spinFor(forward, 35, degrees, false);
RightBackMotor.spinFor(forward, 35, degrees, false);
LeftFrontMotor.spinFor(reverse, 35, degrees, false);
LeftFrontMotor.spinFor(reverse, 35, degrees);
wait (100, msec);
DriveForward(-3, 20);
Flywheel.setStopping(coast);
Flywheel.setVelocity(95, percent);
Flywheel.spin(forward);
wait(3, seconds);
DigitalOutA.set(true);
wait(500, msec);
DigitalOutA.set(false);
wait(500, msec);
DigitalOutA.set(true);
wait(500, msec);
DigitalOutA.set(false);
wait(500, msec);
Flywheel.stop();
TurnLeftHeading(252,25);
wait(100, msec);
Intake.setVelocity(100, percent);
DriveForward(30, 60);
wait(300, msec);
DriveForward(-4, 30);
Intake.spin(forward);
DriveForward(18, 30);
TurnRightHeading (318, 30);

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (true) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    // Flywheel initial values. 
    RightFrontMotor.setStopping(coast);
RightBackMotor.setStopping(coast);
LeftFrontMotor.setStopping(coast);
LeftBackMotor.setStopping(coast);
Flywheel.setStopping(coast);
Expansion.setStopping(hold);
  float LeftMotorSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
  float RightMotorSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();

 if (LeftMotorSpeed>0) {
    LeftBackMotor.setVelocity(pow(LeftMotorSpeed/100, 2)*100, percent);
    LeftFrontMotor.setVelocity(pow(LeftMotorSpeed/100, 2)*100, percent);
  } else {
    LeftBackMotor.setVelocity(pow(LeftMotorSpeed/100, 2)*-100, percent);
    LeftFrontMotor.setVelocity(pow(LeftMotorSpeed/100, 2)*-100, percent);
  }

 
 if (RightMotorSpeed>0) {
    RightBackMotor.setVelocity(pow(RightMotorSpeed/100, 2)*100, percent);
    RightFrontMotor.setVelocity(pow(RightMotorSpeed/100, 2)*100, percent);
  } else {
    RightBackMotor.setVelocity(pow(RightMotorSpeed/100, 2)*-100, percent);
    RightFrontMotor.setVelocity(pow(RightMotorSpeed/100, 2)*-100, percent); 
  }



  RightFrontMotor.spin(forward);
  RightBackMotor.spin(forward);
  LeftBackMotor.spin(forward);
  LeftFrontMotor.spin(forward);

if (Controller1.ButtonL1.pressing()){
  Flywheel.setVelocity(85, percent);
  Flywheel.spin(forward);

} else if (Controller1.ButtonL2.pressing()){
  Flywheel.setVelocity(100, percent);
    Flywheel.spin(forward);
} else {
  Flywheel.stop();
}

if (Controller1.ButtonX.pressing()) {
DigitalOutA.set(true);
} 
else {
  DigitalOutA.set(false);
}

if (Controller1.ButtonUp.pressing()){
  Expansion.setVelocity(100, percent);
  Expansion.spin(forward);
} else {
  Expansion.stop();
}

if (Controller1.ButtonR2.pressing()){
  Intake.setVelocity(100, percent);
  Intake.spin(forward);
} else if (Controller1.ButtonR1.pressing()){
  Intake.setVelocity(50, percent);
Intake.spin(reverse);
} else {
  Intake.stop();}
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
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
