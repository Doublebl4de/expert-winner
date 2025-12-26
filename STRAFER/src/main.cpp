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
// leftFront            motor         20              
// rightFront           motor         18              
// leftBack             motor         19              
// rightBack            motor         17              
// Controller1          controller                    
// bigInertial          inertial      1               
// center1              motor         14              
// center2              motor         15              
// lowTake              motor         11              
// highTake             motor         12              
// midScore             digital_out   A               
// dscore               digital_out   D               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

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
  bigInertial.calibrate();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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


void drive(double left,double right){
  leftFront.spin(forward, left*.12, volt);
  leftBack.spin(forward, left*.12, volt);
  rightFront.spin(forward, right*.12, volt);
  rightBack.spin(forward, right*.12, volt);
}
void strafe(double center){
  
  center1.spin(forward,center*.12, volt);
  center2.spin(forward,center*.12, volt);
}
double max(float val1, float val2){
  if (abs(val1) >= abs(val2)){
    return val1;
  }else if(abs(val2) > abs(val1)){
    return val2;
  }
  return val1;
}
double failSafe(float val){
  if (abs(val) < 16){
    return 16*abs(val)/val;
  }else{
    return val;
  }
}
double clip_num(double input, double max, double min){
  if (input>max){
    return max;
  }
  else if (input < min) {
    return min;
  }
  return input;
}
void pint(float a){
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(a);
}
double pi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651328230664709384460955058223172;
void go(double dist, double power){
  leftBack.setPosition(0,degrees);
  rightBack.setPosition(0,degrees);
  // 3/4 gear ratio, wheel circumference is 2.75 inch * pi
  double totalDegrees = (dist*360*(60))/((3.25/2)*pi*2*36); //CHECKED
  //2*(3.25/2)*pi
  //36/60 
  //for every 360*5/3
  //2.43*
  double proportionalControllerConstant = 0.1;//abs(100/dist)
  int time = 0;
  while (true){
    double left_error = totalDegrees - leftBack.position(degrees);
    double right_error = totalDegrees - rightBack.position(degrees);
    /*
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.clearLine();
    Controller1.Screen.print(leftBack.position(turns));
    Controller1.Screen.print("    ");
    Controller1.Screen.print(left_error);
    Controller1.Screen.newLine();
    Controller1.Screen.clearLine();
    Controller1.Screen.print(rightBack.position(turns));
    Controller1.Screen.print("    ");
    Controller1.Screen.print(right_error);
    */
    
    double left_output = clip_num(failSafe(left_error*proportionalControllerConstant),power,-power);
    double right_output = clip_num(failSafe(right_error*proportionalControllerConstant),power,-power);
    drive(left_output,right_output);
    

    if ((abs(left_error) < 5 and abs(right_error) < 5)){
      drive(0,0); //REDUNDANT
      time ++;
    }
    if (time >= 10){
      drive(0,0);
      break;
    }
    double left_average = (leftBack.velocity(percent));
    double right_average = (rightBack.velocity(percent));
    if (abs(left_average) < 20 and abs(right_average) < 20){
      time ++;
      pint(90);
    }
    wait(0.009, seconds); //IDK
    
    /*
    if (time == 5){
      drive(0,0);
      break;
    }
    */
  }
}
void turn(double target, double power){
  int time = 0;
  double proportionalControllerConstant = 0.9;
  while(true){
    double error = target - bigInertial.rotation();
    int output = clip_num(failSafe(error * proportionalControllerConstant), power, -power);
    drive(output,-output);
    if(abs(leftBack.velocity(percent)) < 0.2 and abs(rightBack.velocity(percent)) < 0.2){
      time += 0;
    } else {
      time = 0;
    }
    if (abs(error) <= 0.1){
      time += 10;
    }else{
      time = 0;
    }
    if (time >= 70){
      drive(0,0);
      bigInertial.setRotation(0, degrees);

      break;
    }
    /*
    Brain.Screen.clearScreen();
    Brain.Screen.clearLine();
    Brain.Screen.print(error);

    Brain.Screen.print(bigInertial.rotation());
    */
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(error);
    Controller1.Screen.print("    ");
    Controller1.Screen.print(output);
    Controller1.Screen.newLine();
    Controller1.Screen.print(bigInertial.rotation());
    Controller1.Screen.newLine();
    Controller1.Screen.print(center1.position(degrees));
    wait(0.001,seconds);
  }
}
void side(double dist,double power){
  center1.setPosition(0,degrees);
  center2.setPosition(0,degrees);
  // 3/4 gear ratio, wheel circumference is 2.75 inch * pi
  double totalDegrees = (dist*360)/((3.25/2)*pi*2); //CHECKED
  //2*(3.25/2)*pi
  //36/60 
  //for every 360*5/3
  //2.43*
  double proportionalControllerConstant = 0.25;//abs(100/dist)
  int time = 0;
  while (true){
    double error = totalDegrees - center1.position(degrees);
    
    
    
    double output = clip_num(failSafe(error*proportionalControllerConstant),power,-power);
    strafe(output);
    

    if ((abs(error) < 15)){
      strafe(0); //REDUNDANT
      time ++;
    }
    if (time >= 5){
      strafe(0);
      break;
    }
    double average = (center1.velocity(percent)+center2.velocity(percent))/2;
    if (abs(average) < 20){
      time ++;
     //IDK
    }
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(error);
    wait(0.001, seconds);
    /*
    if (time == 5){
      drive(0,0);
      break;
    }
    */
  }
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


void autonomous(void) {
  //side(24,100);
  
  side(8.75,100);
  
  lowTake.spin(forward,-100,percent);
  highTake.spin(forward,100,percent);
  go(30,100);
  pint(1);
  go(9,30);
  pint(2);
  wait(100,msec);
  go(9,30);
  pint(3);
  wait(100,msec);
  highTake.spin(forward,50,percent);
  go(-24,100);
  highTake.spin(forward,0,percent);
  side(25,100);
  turn(180,100);
  go(-16,100);

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}
//ThisIsPascelCasing thisIsCamelCasing
bool midPressed = false;
bool mid = false;
bool descorePressed = false;
bool descore = false;
float zone1 = 0; 
float zone4 = 0;
float zone3 = 0;
float zone2 = 0;
float mag = 0;
float R1Toggle = 1;
bool R1Pressing = false;
float verticalDeadZone = 20;
void usercontrol(void) {
  // User control code here, inside the loop
  bigInertial.calibrate();
  midScore.set(false);
  highTake.spin(forward,R1Toggle*100,percent);
  while (1) {
    /*
    if (abs(Controller1.Axis3.position()) > verticalDeadZone){
      mag = (abs(Controller1.Axis3.position()))/Controller1.Axis3.position();
      zone3 = mag*(abs(Controller1.Axis3.position())-verticalDeadZone)/(100-verticalDeadZone);
    }else{
      zone3 = 0;
    }
    if (abs(Controller1.Axis2.position()) > verticalDeadZone){
      mag = (abs(Controller1.Axis2.position()))/Controller1.Axis2.position();
      zone2 = mag*(abs(Controller1.Axis2.position())-verticalDeadZone)/(100-verticalDeadZone);
    }else{
      zone2 = 0;
    }
    */
    drive(Controller1.Axis3.value(), Controller1.Axis2.value());
    /*
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(zone3);
    Controller1.Screen.newLine();
    Controller1.Screen.print(zone2);
    */
    /*
    if (abs(Controller1.Axis1.position()) > 50){
      zone1 = Controller1.Axis1.position();
    }else{
      zone1 = 0;
    }
    if (abs(Controller1.Axis4.position()) > 50){
      zone4 = Controller1.Axis4.position();
    }else{
      zone4 = 0;
    }
    */
    strafe(Controller1.Axis1.position()+Controller1.Axis4.position());
    if (Controller1.ButtonL2.pressing()){
      lowTake.spin(forward,100,percent);
    }else if(Controller1.ButtonL1.pressing()){
      lowTake.spin(forward,-100,percent);
    }else{
      lowTake.spin(forward,0,percent);
    }
    /*
    if (Controller1.ButtonR2.pressing()){
      highTake.spin(forward,100,percent);
    }else if(Controller1.ButtonR1.pressing()){
      highTake.spin(forward,-100,percent);
      
    }else{
      highTake.spin(forward,0,percent);
    }
    */
    if (Controller1.ButtonR1.pressing() and R1Pressing == false){
      R1Pressing = true;
      R1Toggle = R1Toggle * -1;
      
    }else if(Controller1.ButtonR1.pressing() == false){
      R1Pressing = false;
    }
    highTake.spin(forward,100*R1Toggle,percent);
    if (Controller1.ButtonB.pressing()){
      //strafe(100);
    }else if(Controller1.ButtonDown.pressing()){
      //strafe(-100);
    }else{
      //strafe(0);
    }
    if (Controller1.ButtonUp.pressing() and midPressed == false){
      mid = not mid;
      midScore.set(mid);
      midPressed = true;
    }else if (not Controller1.ButtonUp.pressing()){
      midPressed = false;
    }
    if (Controller1.ButtonR2.pressing() and descorePressed == false){
      descore = not descore;
      dscore.set(descore);
      descorePressed = true;
    }else if (not Controller1.ButtonR2.pressing()){
      descorePressed = false;
    }
    wait(0.0025,sec);

  }
    
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    
    //wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
