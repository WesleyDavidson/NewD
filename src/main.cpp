/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       there                                                     */
/*    Created:      7/27/2025, 11:36:37 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "Useful.h"

using namespace vex;

// A global instance of competition
competition Competition;

controller Controller1(primary);
brain Brain1;

motor frontLeft(PORT6, ratio6_1, true);    //FrontLeft Motor
motor middleLeft(PORT7, ratio6_1, false);   //MiddleLeft Motor
motor backLeft(PORT8, ratio6_1, true);    //BackLeft Motor
motor frontRight(PORT1, ratio6_1, false);   //FrontRight Motor
motor middleRight(PORT2, ratio6_1, true);    //MiddleRight Motor
motor backRight(PORT3, ratio6_1, false);   //BackRight Motor
motor bottomIntake(PORT5, ratio6_1, true);   //Bottom Intake Motor
motor topIntake(PORT4, ratio6_1, true);   //Top Intake Motor

digital_out TM = digital_out(Brain1.ThreeWirePort.H);   //3-Wire Tounge Mech Pistons

inertial Inertial = inertial (PORT11);

//booleans
bool BY_old = false;
bool BY = false;
bool toggle = false;
bool R1_old = false;
bool toggle2 = false;
bool toggle3 = false;
bool R2 = false;
bool R2_old = false;


int J1, J2;
int ABSJ1, ABSJ2;



// void Drive(float inches, float speed){
//   frontLeft.spin()
// }


//void Drive(float deg, )
//PID Drive Function from Useful.cpp
/*void Drive(float inches, float speed){
    float DPR = (360/(M_PI*2.75))*(4.0/3.0);        //DistancePerRotation

    float TargetDistance = inches * DPR;
    float KP = .5;    //Overshoot smaller Undershoot Bigger
    float KI = .01;
    float KD = .01;   
   float Error;
     frontLeft.resetPosition();
     frontRight.resetPosition();
   do {
    float WheelPos = (frontRight.position(vex::degrees) + frontLeft.position(vex::degrees))/(2);
    Error = TargetDistance - WheelPos;
    float Output = 5.001;//KP * Error;

  //  if (Output > speed) Output = speed;
  //  if (Output < -speed) Output = speed;


    frontLeft.spin(forward, Output, pct);  
    middleLeft.spin(forward, Output, pct);
    backLeft.spin(forward, Output, pct);
    frontRight.spin(forward, Output, pct);
    middleRight.spin(forward, Output, pct);
    backRight.spin(forward, Output, pct);
   }while (fabs(Error)>1);
   stopMotors();
}*/







void Drive1(float inches, float speed) {
    float TargetDistance = inches; //Units are Inches
    float DPI = (360 / (M_PI * 2.75)) * (4.0 / 3.0);  // Degrees Per Inch = Degrees per inch of travel
    float TargetDegrees = TargetDistance * DPI;  // Target distance based on the input
    
    // PID constants
    float KP = 1.0;  // Proportional gain
    float KI = .01; // Integral gain (if needed)
    float KD = .01; // Derivative gain (if needed)
    double SlewRate = .5;
    double PrevOutput = 0.0;
    double LimitedOutput;


    float lastError = 0;
    float integral = 0;

    frontLeft.resetPosition();
    frontRight.resetPosition();

    float Error = TargetDegrees;
    float Output;

    // PID control loop
  do {
  float WheelPos = (frontRight.position(vex::degrees) + frontLeft.position(vex::degrees)) / 2;
  Error = TargetDegrees - WheelPos;  // Calculate the error
  integral += Error;  // Integral term (accumulated error over time)
  float derivative = Error - lastError;  // Derivative term (change in error)
        
  // Calculate the PID output
  Output = (KP * Error) + (KI * integral) + (KD * derivative);
    
 // Limit the speed to the max speed
 //if (Output > speed) Output = speed;
// if (Output < -speed) Output = -speed;

if (fabs(Output - PrevOutput) > SlewRate) {
 if (Output > PrevOutput) {
  LimitedOutput = PrevOutput + SlewRate;
 } else {
  LimitedOutput = PrevOutput - SlewRate;
 }
} else {
  LimitedOutput = Output;
}



 // Set motor speeds based on the output of the PID controller
  frontLeft.spin(forward, LimitedOutput, pct);
  middleLeft.spin(forward, LimitedOutput, pct);
  backLeft.spin(forward, LimitedOutput, pct);
  frontRight.spin(forward, LimitedOutput, pct);
  middleRight.spin(forward, LimitedOutput, pct);
  backRight.spin(forward, LimitedOutput, pct);

  lastError = Error;  // Save the error for the next loop iteration
  PrevOutput = LimitedOutput;
   wait(20, msec);  // Wait a short amount to prevent overload
} while (fabs(Error) > 10);  // Continue until the error is small enough (robot reaches the target position)

  stopMotors();  // Stop the motors once the target is reached
}


//Works, wheelies, no PID
/*void DriveSimple(float inches, float speed) {
    float DPR = (360.0 / (M_PI * 2.75)) * (4.0 / 3.0);  // DistancePerRotation
    float TargetDistance = inches * DPR;  // Target distance based on the input

    // Reset motor positions
    frontLeft.resetPosition();
    frontRight.resetPosition();

    // Simple motor drive without PID
    float TargetPos = TargetDistance; 
    
    while (fabs(frontLeft.position(degrees)) < TargetPos) {
        frontLeft.spin(forward, speed, pct);
        middleLeft.spin(forward, speed, pct);
        backLeft.spin(forward, speed, pct);
        frontRight.spin(forward, speed, pct);
        middleRight.spin(forward, speed, pct);
        backRight.spin(forward, speed, pct);
        wait(20, msec); // Small delay for better control
    }

    stopMotors();
}
*/





//PID Turn Function
/*void Turn(float degrees, float speed){
    Inertial.resetRotation();
    float KP = .1;
    float Error;
    do{
        float Inertialrotation = Inertial.rotation(deg);
        Error = degrees - Inertialrotation;
        float Output = Error * KP;
        if(Output > speed) Output = speed;
        if (Output < -speed) Output = -speed;

        frontLeft.spin(forward, Output, pct);
        middleLeft.spin(forward, Output, pct);
        backLeft.spin(forward, Output, pct);
        frontRight.spin(reverse, Output, pct);
        middleRight.spin(reverse, Output, pct);
        backRight.spin(reverse, Output, pct);
        
} while (fabs(Error)>1);
  stopMotors();*/
//}

void stopMotors() {      //Stop Drive Motors Command
middleLeft.setStopping(brakeType::brake);
backLeft.setStopping(brakeType::brake);
frontRight.setStopping(brakeType::brake);
middleRight.setStopping(brakeType::brake);
backRight.setStopping(brakeType::brake);

frontLeft.stop();
middleLeft.stop();
backLeft.stop();
frontRight.stop();
middleRight.stop();
backRight.stop();
}

void stopIntake() {
bottomIntake.setStopping(brakeType::brake);   //Stop bottom Intake Motor Command
topIntake.setStopping(brakeType::brake);   //Stop top Intake Motor Command
bottomIntake.stop();
topIntake.stop();
}

int deadzone = 5;

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
  TM.set(true);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//*---------------------------------------------------------------------------*/
//*                                                                           */
//*                              Autonomous Task                              */
//*                                                                           */
//*  This task is used to control your robot during the autonomous phase of   */
//*  a VEX Competition.                                                       */
//*                                                                           */
//*  You must modify the code to add your own robot specific commands here.   */
//*---------------------------------------------------------------------------*/

void autonomous(void) {
//DriveSimple(12, 50);
Drive1(48, 25);
}

void drive(int J1, int J2){
  frontLeft.spin(forward, J2 + J1, pct);  // Left side motors
  middleLeft.spin(forward, J2 + J1, pct);
  backLeft.spin(forward, J2 + J1, pct);

  frontRight.spin(forward, J2 - J1, pct);  // Right side motors
  middleRight.spin(forward, J2 - J1, pct);
  backRight.spin(forward, J2 - J1, pct);
}

void fIntake (bool toggle2) {
  if (toggle2 == true){
    bottomIntake.spin(reverse, 75, pct);
    topIntake.spin(forward, 75, pct);

  }else{ //Else turn ramp and intake off
    bottomIntake.stop();
    topIntake.stop();

  }

}

void rIntake (bool toggle3) {
  if (toggle3 == true){
    bottomIntake.spin(forward, 75, pct);
    topIntake.spin(reverse, 75, pct);

  }else{ //Else turn ramp and intake off
    bottomIntake.stop();
    topIntake.stop();

  }

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
  while (1) {

bool R1 = Controller1.ButtonR1.pressing();
bool R2 = Controller1.ButtonR2.pressing();
bool BY = Controller1.ButtonY.pressing();
int J1 = Controller1.Axis1.position(pct);  // Left/Right joystick (X-axis)
int J2 = Controller1.Axis3.position(pct);  // Forward/Backward joystick (Y-axis)

drive(J1, J2);


 //Start Deadzone

ABSJ1 = J1;
ABSJ2 = J2;
if (J1 < 0) ABSJ1 = -J1;
if (J2 < 0) ABSJ2 = -J2;

if (ABSJ1 < deadzone) J1 = 0;
if (ABSJ2 < deadzone) J2 = 0;

//End Deadzone



//Drive Train loop
if ((J1 == 0) && (J2 == 0)) {   //If J1 and J2 = 0, stop motors
  stopMotors();
} else {
  drive(J1, J2);
}



//Tongue Mech Command
if (BY && !BY_old){   //Activates Tounge Mech pistons when BY changes
toggle = !toggle;
TM.set(toggle);
} 
BY_old = BY;


//Intake & Ramp Control
// if (R1 && !R1_old){
// toggle2 = !toggle2;
// R1_old = R1;
// }



if(R1 && !R1_old){
toggle2 = 1-toggle2;
  if (toggle2){
  bottomIntake.spin(forward, 75, pct);
  topIntake.spin(forward, 75, pct);
  }
  if (!toggle2){ 
  bottomIntake.stop();
  topIntake.stop();
          
}
      
}
R1_old=R1;



if(R2 && !R2_old){
toggle3 = 1-toggle3;
  if (toggle3){
  bottomIntake.spin(reverse, 75, pct);
  topIntake.spin(reverse, 75, pct);
  }
  else{
  bottomIntake.stop(brakeType::hold);
  topIntake.stop(brakeType::hold);
  }

      
}
R2_old = R2;



//Old Intake

// if (toggle2 == true) {
// Intake.spin(reverse, 75, pct);
// IntakeB.spin(forward, 75, pct);
//   } else {
//     Intake.stop(brake); //Stops Intake Motor
//   }


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
