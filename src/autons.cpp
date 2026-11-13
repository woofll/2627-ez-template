#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // chassis.drive_imu_scaler_set(1.05);
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(12.5, 0.0, 100.0);         // Fwd/rev constants, DONE
  chassis.pid_heading_constants_set(15.0, 0.0, 45.0);       // Heading constants, DONE 
  chassis.pid_turn_constants_set(3.6, 0.125, 20.0, 15.0);     // Turning constants, DONE
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // 
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // 
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // 

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}


///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .


void toggleOnly(){
  toggle();

  chassis.pid_drive_set(-24, 120);
}


void leftZone(){
    global::cascadeLeft.set_zero_position(0);
  global::cascadeRight.set_zero_position(0);
  global::cascade.set_zero_position(0);

//TOGGLE****************************************************************************************8
chassis.pid_drive_set(-3, 127);
 chassis.pid_wait_quick_chain();

chassis.pid_drive_set(5, 127);
 chassis.pid_wait_quick_chain();

chassis.pid_drive_set(-3, 127); 
 chassis.pid_wait_quick_chain();
 
chassis.pid_drive_set(4.1, 127);
  global::wrist.extend();
 chassis.pid_wait_quick_chain();
 //TOGGLE****************************************************************************************8

 //FIRST PIN****************************************************************************************8

  global::cascade.move_absolute(700, 127);
  chassis.pid_drive_set(-14.5, 120);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(90, 100);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-12, 120);
  chassis.pid_wait_quick();
   global::cascade.move_absolute(0, 127);
    chassis.pid_drive_set(-1, 100);
  chassis.pid_wait_quick_chain();
  
  global::claw.extend();

    chassis.drive_angle_set(90);
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);    // Set the current position

  chassis.pid_drive_set(10, 110);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(35, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-15, 120);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6, 20);
  chassis.pid_wait_quick_chain();
  global::cascade.move_absolute(700, 120);
  global::claw.retract();
  
  chassis.pid_drive_set(-18, 10);
  chassis.pid_wait_quick();
  global::cascade.move_absolute(2300, 127);

chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, -100, -65);
chassis.pid_wait_quick();

chassis.pid_drive_set(-14, 90);
global::cascade.move_absolute(700, 100);
chassis.pid_wait_quick();
global::claw.extend();
pros::delay(100);

  chassis.drive_angle_set(90);
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);    // Set the current position

//FIRST PIN****************************************************************************************8

//SECOND PIN****************************************************************************************8

global::wrist.extend();
global::claw.extend();

  chassis.pid_drive_set(10, 110);
  chassis.pid_wait_quick();
  global::cascade.move_absolute(0, 127);

  chassis.pid_turn_set(35, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-15, 120);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6, 25);
  chassis.pid_wait_quick_chain();
  global::claw.retract();
  global::cascade.move_absolute(400, 120);
  chassis.pid_drive_set(-18, 10);
  chassis.pid_wait_quick();
  global::cascade.move_absolute(2300, 127);

chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, -100, -65);
chassis.pid_wait_quick();

chassis.pid_drive_set(-14, 60);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-7, 60);
global::cascade.move_relative(-1200, 100);
chassis.pid_wait_quick();
global::claw.extend();
pros::delay(300);
chassis.pid_drive_set(-14, 60);
chassis.pid_wait_quick_chain();
//SECOND PIN****************************************************************************************8

}

void leftCross(){

}

void rightZone(){
  toggle();
  global::wrist.retract();
  global::cascade.move_relative(1500, 120);
  pros::delay(500);
  chassis.pid_drive_set(-16, 100);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(-15, 90);
  chassis.pid_wait();
  
  global::cascade.move_relative(-1650, 127);
  pros::delay(1000);
  global::claw.extend();
  pros::delay(1000);
  chassis.pid_drive_set(8, 100);
  chassis.pid_wait();
}

void rightCross(){

}




void fwd24(){
  chassis.pid_drive_set(24, 120);
  chassis.pid_wait();
}

void rev24(){
  chassis.pid_drive_set(-24, 120);
  chassis.pid_wait();
}

void left90(){
  chassis.pid_turn_set(-90, 120);
  chassis.pid_wait();
}

void right90(){
  chassis.pid_turn_set(90, 120);
  chassis.pid_wait();
}


void skills(){

// Phase 1 - Toggle
chassis.pid_drive_set(-8, 120);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(13, 127);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(-8, 120); 
chassis.pid_wait_quick_chain();
 
chassis.pid_drive_set(13, 127);
chassis.pid_wait_quick_chain();

// Phase 2 - Score First Pin

// Phase 3 - Score second cuppin

// Phase 4 - Score third cuppin 

// Phase 5 - Twice matchload

// Phase 6 - Score pin on second alliance

// Phase 7 - Three matchloads

// Phase 8 - Second Toggle

// Phase 9 - Park in the middle
}