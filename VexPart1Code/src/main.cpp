// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Inertia              inertial      10              
// Controller1          controller                    
// Intake               motor         11              
// DigitalOutA          digital_out   A               
// DigitalOutB          digital_out   B               
// Catapult             motor         20              
// GPS7                 gps           7               
// RightMiddleMotor     motor         19              
// LeftMiddleMotor      motor         5               
// RightFrontMotor      motor         1               
// LeftFrontMotor       motor         3               
// RightBackMotor       motor         17              
// LeftBackMotor        motor         18              
// LimitSwitchC         limit         C               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
#include "cmath"
using namespace vex;

// A global instance of competition
competition Competition;

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
  RightFrontMotor.setVelocity(SpeedPercent, percent);
  RightMiddleMotor.setVelocity(SpeedPercent, percent);
  RightBackMotor.setVelocity(SpeedPercent, percent);
  LeftFrontMotor.setVelocity(SpeedPercent, percent);
  LeftMiddleMotor.setVelocity(SpeedPercent, percent);
  LeftBackMotor.setVelocity(SpeedPercent, percent);
}

void MotorTimeOut (int x){
  RightFrontMotor.setTimeout(x, seconds);
  RightMiddleMotor.setTimeout(x, seconds);
  RightBackMotor.setTimeout(x, seconds);
  LeftFrontMotor.setTimeout(x, seconds);
  LeftMiddleMotor.setTimeout(x, seconds);
  LeftBackMotor.setTimeout(x, seconds);
}

void SetSpeedVoltage (double SpeedPercent) {
  RightFrontMotor.setVelocity(SpeedPercent, percent);
  RightMiddleMotor.setVelocity(SpeedPercent, percent);
  RightBackMotor.setVelocity(SpeedPercent, percent);
  LeftFrontMotor.setVelocity(SpeedPercent, percent);
  LeftMiddleMotor.setVelocity(SpeedPercent, percent);
  LeftBackMotor.setVelocity(SpeedPercent, percent);
}

void Drive (double x, double y) {
    SetSpeed (x);
    RightFrontMotor.spinFor(forward, y/12.56, turns, false);
    RightMiddleMotor.spinFor(forward, y/12.56, turns, false);
    RightBackMotor.spinFor(forward, y/12.56, turns, false);
    LeftFrontMotor.spinFor(forward, y/12.56, turns, false);
    LeftMiddleMotor.spinFor(forward, y/12.56, turns, false);
    LeftBackMotor.spinFor(forward, y/12.56, turns);
}

void resetSensor (){
  RightFrontMotor.setPosition(0, degrees);
  RightMiddleMotor.setPosition(0, degrees);
  RightBackMotor.setPosition(0, degrees);
  LeftFrontMotor.setPosition(0, degrees);
  LeftMiddleMotor.setPosition(0, degrees);
  LeftBackMotor.setPosition(0, degrees);
}

void Stop () {
  RightFrontMotor.stop();
  RightMiddleMotor.stop();
  RightBackMotor.stop();
  LeftFrontMotor.stop();
  LeftMiddleMotor.stop();
  LeftBackMotor.stop();
}

void SmallTurnRight(double y){
  double x = 0;
  while (y>Inertia.rotation()){
    x=x+0.3;
    SetSpeed(x);
    RightFrontMotor.spin(reverse);
    RightMiddleMotor.spin(reverse);
    RightBackMotor.spin(reverse);
    LeftFrontMotor.spin(forward);
    LeftMiddleMotor.spin(forward);
    LeftBackMotor.spin(forward);
    wait(30, msec);
  }
  Stop();
}

void TurnPID (double Setpoint) {
  resetSensor();
  double kP = .12;
  double rightspeed = 0;
  double leftspeed = 0;
  double velCap = 0;
  double acc = 0.2;
  bool PIDon = true;
  while (PIDon){
    double error = Setpoint - Inertia.rotation();
    if (error>-7 && error<7){
      PIDon = false;
    }
    velCap = velCap + acc;
    if (velCap>4){
       velCap = 4;
    }

      rightspeed = -1*error*kP;
      leftspeed = error*kP;
      if (rightspeed<0 && rightspeed>-0.2){
        rightspeed = -0.2;
        leftspeed = 0.2;
      }
      if (leftspeed<0 && leftspeed>-0.2){
        leftspeed = -0.2;
        rightspeed = 0.2;
      }
      if (leftspeed>velCap){
        leftspeed = velCap;
        rightspeed = -1*velCap;
      } else if (rightspeed>velCap){
        rightspeed = velCap;
        leftspeed = -1*velCap;
      }

    RightFrontMotor.spin(forward, rightspeed, voltageUnits::volt);
    RightMiddleMotor.spin(forward, rightspeed, voltageUnits::volt);
    RightBackMotor.spin(forward, rightspeed, voltageUnits::volt);
    LeftFrontMotor.spin(forward, leftspeed, voltageUnits::volt);
    LeftMiddleMotor.spin(forward, leftspeed, voltageUnits::volt);
    LeftBackMotor.spin(forward, leftspeed, voltageUnits::volt);
    wait(10, msec);
  }
  Stop();
}

void LateralPID (double Setpoint) {
  resetSensor();
  double kP = .9;
  double speed = 0;
  double velCap = 0;
  double acc = 0.2;
  bool PIDon = true;
  while (PIDon){
    double error = Setpoint - (RightBackMotor.position(turns))*13.33;
    if (error>-0.8 && error<0.8){
      PIDon = false;
    }
    speed = error*kP;
    velCap = velCap + acc;
    if (velCap>6){
      velCap = 6;
    }
    if (Setpoint > 0 && speed > velCap){
      speed = velCap;
    } else if (Setpoint < 0 && speed < -1*velCap){
     speed =  -1*velCap;
    }
    RightFrontMotor.spin(forward, speed, voltageUnits::volt);
    RightMiddleMotor.spin(forward, speed, voltageUnits::volt);
    RightBackMotor.spin(forward, speed, voltageUnits::volt);
    LeftFrontMotor.spin(forward, speed, voltageUnits::volt);
    LeftMiddleMotor.spin(forward, speed, voltageUnits::volt);
    LeftBackMotor.spin(forward, speed, voltageUnits::volt);
    wait(10, msec);
  }
  Stop();
}

void CatapultPosition() {
  Catapult.spinFor(forward, 180, degrees);
  Catapult.spin(forward);
  bool x = true;
  while (x){
    if (LimitSwitchC.pressing()){
      Catapult.stop();
      x = false;
  }
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
//Start of Part 1: two rollers, 3 in high goal
Catapult.setVelocity(100, percent);
Intake.setVelocity(50, percent);
MotorTimeOut(3);
Drive(15, -5);
Intake.spinFor(forward, -450, degrees);
MotorTimeOut(60);
LateralPID(4);
TurnPID(125);
Intake.setVelocity(100, percent);
Intake.spin(forward);
LateralPID(-24);
wait(0.6, seconds);
Intake.stop();
TurnPID(85);
Drive(20.5, -9.5);
wait(0.3, seconds);
Intake.setVelocity(50, percent);
Intake.spinFor(forward, -450, degrees);
LateralPID(3);
TurnPID(-4);
LateralPID(72);
SmallTurnRight(10);
wait(0.3, seconds);
CatapultPosition();
//Start of Part 2: two rollers, 6 in high goal
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (true) {
    Catapult.setVelocity(100, percent);
    RightFrontMotor.setStopping(coast);
    RightMiddleMotor.setStopping(coast);
    RightBackMotor.setStopping(coast);
    LeftFrontMotor.setStopping(coast);
    LeftMiddleMotor.setStopping(coast);
    LeftBackMotor.setStopping(coast);

    float LeftMotorSpeed = Controller1.Axis3.position() + Controller1.Axis1.position()*.8;
    float RightMotorSpeed = Controller1.Axis3.position() - Controller1.Axis1.position()*.8;

    if (LeftMotorSpeed>=0){
    LeftFrontMotor.spin(forward, pow(LeftMotorSpeed/100.0, 2)*8.4, voltageUnits::volt);
    LeftMiddleMotor.spin(forward, pow(LeftMotorSpeed/100.0, 2)*8.4, voltageUnits::volt);
    LeftBackMotor.spin(forward, pow(LeftMotorSpeed/100.0, 2)*8.4, voltageUnits::volt);
    } else if (LeftMotorSpeed<0){
    LeftFrontMotor.spin(forward, pow(LeftMotorSpeed/100.0, 2)*-8.4, voltageUnits::volt);
    LeftMiddleMotor.spin(forward, pow(LeftMotorSpeed/100.0, 2)*-8.4, voltageUnits::volt);
    LeftBackMotor.spin(forward, pow(LeftMotorSpeed/100.0, 2)*-8.4, voltageUnits::volt);
    }

    if (RightMotorSpeed>=0){
    RightFrontMotor.spin(forward, pow(RightMotorSpeed/100.0, 2)*8.4, voltageUnits::volt);
    RightMiddleMotor.spin(forward, pow(RightMotorSpeed/100.0, 2)*8.4, voltageUnits::volt);
    RightBackMotor.spin(forward, pow(RightMotorSpeed/100.0, 2)*8.4, voltageUnits::volt);
    } else if (RightMotorSpeed<0){
    RightFrontMotor.spin(forward, pow(RightMotorSpeed/100.0, 2)*-8.4, voltageUnits::volt);
    RightMiddleMotor.spin(forward, pow(RightMotorSpeed/100.0, 2)*-8.4, voltageUnits::volt);
    RightBackMotor.spin(forward, pow(RightMotorSpeed/100.0, 2)*-8.4, voltageUnits::volt);
    }
  


    if (Controller1.ButtonL1.pressing()){
      Intake.setVelocity(100,percent);
      Intake.spin(forward);
    } else if (Controller1.ButtonL2.pressing()){
      Intake.setVelocity(50, percent);
      Intake.spin(reverse);
    } else {
      Intake.stop();
    }

    Catapult.spin(forward);
    if (LimitSwitchC.pressing()){
      Catapult.stop();
    }

    if (Controller1.ButtonR1.pressing()){
      Catapult.spinFor(forward, 45, degrees);
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
