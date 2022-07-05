#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;
pros::Controller Controller1 = pros::Controller(CONTROLLER_MASTER); 
pros::Motor Intake(21, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); 
pros::Motor LifterFrontLeft(1, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES); 
pros::Motor LifterFrontRight(10, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES); 
pros::ADIDigitalOut Pneumatics1('A'); 
pros::ADIDigitalOut Pneumatics2('C');
pros::Motor LifterBack(20, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES); 
pros::Motor DriveTrainLF(13, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor DriveTrainLB(14, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES); 
pros::Motor DriveTrainRF(17, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);  
pros::Motor DriveTrainRB(18, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES); 
pros::Imu InertialSensor(11);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    Pneumatics1.set_value(false);
    Pneumatics2.set_value(false);
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DriveTrainLF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DriveTrainLB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DriveTrainRF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DriveTrainRB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    LifterBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LifterFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LifterFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    InertialSensor.reset();
    
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void inertialsensorPID (double angle){
  double threshold;
  if(angle <= 0.0) {
    threshold = 1;
  } else {
    threshold = 0.5;
  }
  double error = angle - InertialSensor.get_rotation();
  double integral;
  double derivative;
  double prevError;
  double kp = 0.98;
  double ki = 0;
  double kd = 1;

  while(fabs(error) > threshold){
    error = angle - InertialSensor.get_rotation();
    integral = integral + error;
    if(error == 0 || fabs(error) >= angle){
      integral = 0; 
    }
    derivative = error - prevError;
    prevError = error;
    double p = error * kp;
    double i = integral * ki;
    double d = derivative * kd; 
    
    double vel = p + i + d;

    DriveTrainRF.move_velocity(vel); 
    DriveTrainLF.move_velocity(vel);
    DriveTrainRB.move_velocity(vel);
    DriveTrainLB.move_velocity(vel);

    pros::delay(10);
  }
}


void autonomous() {
   std::shared_ptr<ChassisController> chassisForward =
    ChassisControllerBuilder()
        .withMotors(13, 14, -17, -18)
        .withDimensions(AbstractMotor::gearset::green, {{4_in, 14.5_in}, imev5GreenTPR})
        .build();
    std::shared_ptr<ChassisController> chassisBackward =
    ChassisControllerBuilder()
        .withMotors(-13, -14, 17, 18)
        .withDimensions(AbstractMotor::gearset::green, {{4_in, 14.5_in}, imev5GreenTPR})
        .build();
  Pneumatics1.set_value(true);
  Pneumatics2.set_value(true);
  chassisBackward->moveDistance(4_in);
  void waitUntilSettled();
  LifterBack.move_absolute(30, 70); 
  void waitUntilSettled();
  inertialsensorPID(135); 
  void waitUntilSettled();
  chassisForward->moveDistance(30_in);
  Pneumatics1.set_value(false);
  Pneumatics2.set_value(false);
  void waitUntilSettled();
  chassisForward->moveDistance(45_in);
  LifterFrontLeft.move_absolute(100, 70);
  LifterFrontRight.move_absolute(100, 70);
  void waitUntilSettled();
  chassisForward->moveDistance(4_in);
  Pneumatics1.set_value(true);
  Pneumatics2.set_value(true);
  void waitUntilSettled();
  chassisBackward->moveDistance(7_in);
  void waitUntilSettled();
  inertialsensorPID(200); 
  void waitUntilSettled();
  LifterBack.move_absolute(-30, 70);
  void waitUntilSettled();
  chassisForward->moveDistance(40_in);
  void waitUntilSettled();
  Pneumatics1.set_value(false);
  Pneumatics2.set_value(false);
  void waitUntilSettled();
  chassisBackward->moveDistance(40_in);
  void waitUntilSettled();
  inertialsensorPID(160);
  void waitUntilSettled();
  chassisForward->moveDistance(7_in);
  void waitUntilSettled();
  inertialsensorPID(270); 
  void waitUntilSettled();
  chassisForward->moveDistance(50_in);
  void waitUntilSettled();
  Pneumatics1.set_value(false);
  Pneumatics2.set_value(false);
  void waitUntilSettled();
  chassisBackward->moveDistance(50_in);
  void waitUntilSettled();
  inertialsensorPID(90); 
  void waitUntilSettled();
  chassisForward->moveDistance(7_in);
  void waitUntilSettled();
  LifterFrontLeft.move_absolute(100, 70);
  LifterFrontRight.move_absolute(100, 70);
  void waitUntilSettled();
  Pneumatics1.set_value(true);
  Pneumatics2.set_value(true);
  //Make sure to lower LifterFront other thatn that we good
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
//Seperate Functions for Each of the Mototrs 

void doDrive() {
  int DriveTrainRightVelocity = 0;
  int DriveTrainLeftVelocity = 0;
    DriveTrainRightVelocity = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) - Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    DriveTrainLeftVelocity = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    if (DriveTrainRightVelocity > 127) {
      DriveTrainRightVelocity = 127;
    } else if (DriveTrainRightVelocity < -127) {
      DriveTrainRightVelocity = -127;
    }
    if (DriveTrainLeftVelocity > 127) {
      DriveTrainLeftVelocity = 127;
    } else if (DriveTrainLeftVelocity < -127) {
      DriveTrainLeftVelocity = -127;
    }
    DriveTrainRB = DriveTrainRightVelocity;
    DriveTrainRF = DriveTrainRightVelocity; 
    DriveTrainLF = DriveTrainLeftVelocity;
    DriveTrainLB = DriveTrainLeftVelocity;
}
void doIntake() {
   int intakeVelocity = 0;
     if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_X) && Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
       intakeVelocity = 0;
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
       intakeVelocity = 127;
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
       intakeVelocity = -127;
     } else {
       intakeVelocity = 0;
     }
     Intake = intakeVelocity;
 }
 void doLifterFront() {
   int lifterFrontVelocity = 0;
     if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
       lifterFrontVelocity = 0;
       LifterFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       LifterFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
       lifterFrontVelocity = 127;
       LifterFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
       LifterFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
       lifterFrontVelocity = -127;
       LifterFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
       LifterFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
     } else {
       lifterFrontVelocity = 0;
       LifterFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       LifterFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
     }
     LifterFrontRight = lifterFrontVelocity;
     LifterFrontLeft = lifterFrontVelocity;
     
 }
 void doLifterBack() {
   int lifterBackVelocity = 0;
     if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
       lifterBackVelocity = 0;
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
       lifterBackVelocity = 127;
     } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
       lifterBackVelocity = -127;
     } else {
       lifterBackVelocity = 0;
     }
     LifterBack = lifterBackVelocity;
 }
 void doPneumatics() {
     if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
       Pneumatics1.set_value(true);
       Pneumatics2.set_value(true);
     }
     if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
       Pneumatics1.set_value(false);
       Pneumatics2.set_value(false);
     }
 }


//Main Driver Control 
void opcontrol() {
  DriveTrainLF.move_velocity(200); 
  DriveTrainLB.move_velocity(200); 
  DriveTrainRF.move_velocity(200); 
  DriveTrainRB.move_velocity(200); 

  while (true) {
    doDrive();
    doIntake();
    doLifterBack();
    doLifterFront();
    doPneumatics();
    pros::delay(10);
  }
}