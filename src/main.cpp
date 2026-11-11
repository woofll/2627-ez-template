#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-13, -17},    // Left Chassis Ports (negative port will reverse it!)
    {15, 18},  // Right Chassis Ports (negative port will reverse it!)

    5,      // IMU Port
    2.67,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (paking these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(x, 2, x);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(20, 2, 0);   // This tracking wheel is parallel to the drive wheels


/************************************************************************************************************************** */
const int CLAWnumStates = 5; 
int CLAWstates[CLAWnumStates] = {0, 100000, 200000, 300000, 400000}; //
int CLAWcurrState = 0;
int CLAWtarget = 0;

void CLAWnextState() {
  CLAWcurrState++;

  if (CLAWcurrState >= CLAWnumStates) {
    CLAWcurrState = 0;
  }

  CLAWtarget = CLAWstates[CLAWcurrState];
}
void CLAWbackState() {
  CLAWcurrState--;

  if (CLAWcurrState < 0) {
    CLAWcurrState = CLAWnumStates - 1;
  }

  CLAWtarget = CLAWstates[CLAWcurrState];
}

void CLAWzeroState() {
  CLAWcurrState = 0;
  CLAWtarget = CLAWstates[CLAWcurrState];
}

// void CLAWliftControl() {
//   double kp = 0.1;
//   double error = CLAWtarget - (global::cascadeRotation.get_position());
//   double velocity = kp * error;
//   if (velocity > 127) velocity = 127;
//   if (velocity < -127) velocity = -127;
//   global::cascade.move(velocity); 
// }
/************************************************************************************************************************** */


void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(2);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(10.0, 10.0);  // Defaults for curve. If using tank, only the first parameter is used.

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    {"RIGHT ONE PIN!", rightZone},    
    {"LEFT ONE PIN!", leftZone},      
    {"only toggle!", toggleOnly},
  });

  // Initialize chassis and auton selector
  chassis.initialize(); 
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  
  global::cascade.set_zero_position(0);
  global::cascadeRotation.set_position(0);

  // pros::Task liftControlTask([]{
  //       while (true) {
  //           CLAWliftControl();
  //           pros::delay(10);
  //       }
  //   });

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code.
 */
void autonomous() {
  chassis.pid_targets_reset();            // Resets PID targets to 0
  chassis.drive_imu_reset();              // Reset gyro position to 0
  chassis.drive_sensor_reset();             // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());            
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  
  }
  ez::screen_print(tracker_value + tracker_width, line);  
}

/**
 * Ez screen task
 */
void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                           "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                           "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  

          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol
 */
void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    chassis.pid_tuner_iterate();
  }
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code.
 */
void opcontrol() {

chassis.drive_brake_set(MOTOR_BRAKE_COAST);

double clawCloseTime = 0; // Set time that claw closes
double clawOpenTime = 0; // Set time that claw opens
double cascadeStateTime = 0;
bool clawDisEnabled = true; // Distance sensor is enabled

bool wristPos = true; // True = wrist is UP
bool clawPos = false; // False = claw is CLOSED

while (true) {
  ez_template_extras();

  // chassis.opcontrol_tank(); // Tank control
  chassis.opcontrol_arcade_standard(ez::SPLIT); // Standard split arcade
  global::cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  double cascadePos = global::cascadeRotation.get_position();
  double clawDis = global::clawDistance.get();
  double curTime = pros::millis();

  // pros::lcd::set_text(5, std::to_string(chassis.drive_sensor_left()));
  // pros::lcd::set_text(5, std::to_string(clawCloseTime));
  // pros::lcd::set_text(6, std::to_string(clawDisEnabled));
  pros::lcd::set_text(5, std::to_string(CLAWcurrState));
  pros::lcd::set_text(6, std::to_string(CLAWtarget));
  pros::lcd::set_text(7, std::to_string(global::cascadeRotation.get_position()));
  // pros::lcd::set_text(6, std::to_string(chassis.drive_sensor_right()));
  // pros::lcd::set_text(7, std::to_string(chassis.odom_theta_get()));

  if (master.get_digital_new_press(DIGITAL_UP)){ // UP pressed, raise/lower wrist
    if (wristPos == true){ // Wrist = down, so raise claw
      global::wrist.extend();
      wristPos = false;
    } else if (wristPos == false){ // Wrist = up, so lower claw
      global::wrist.retract();
      wristPos = true;
    }
  }

  if (master.get_digital_new_press(DIGITAL_L1)){ // L1 pressed, so claw opens/closes
    if (clawPos == false){ // Claw = closed, so OPEN
      global::claw.extend();
      clawPos = true; // Claw is open

      clawDisEnabled = false; // Distance sensor disabled
      clawOpenTime = pros::millis(); // Update when claw is opened

    } else if (clawPos == true){ // Claw = open, so CLOSE
      global::claw.retract();
      global::cascade.move_relative(800, 127); // Raise cascade to bring cuppin off floor
      clawPos = false; // Claw is closed
      clawCloseTime = pros::millis(); // Update when claw is closed
    }
  }

  if (master.get_digital(DIGITAL_R1)){ // Cascade UP
    global::cascade.move_velocity(200);
    // CLAWtarget = global::cascadeRotation.get_position();
    // global::cascadeLeft.move_velocity(200);
  } else if (master.get_digital(DIGITAL_R2) && cascadePos > 3300 ){ // Cascade DOWN
    global::cascade.move_velocity(-200);
    // CLAWtarget = global::cascadeRotation.get_position();
    // global::cascadeRight.move_velocity(200);
  } else {
    if ((curTime - clawCloseTime > 500) || (curTime - cascadeStateTime > 1000)) {

        // CLAWliftControl();
        global::cascade.brake();

    }
    // else {
    //     global::cascade.brake();
  }

  // if (master.get_digital(DIGITAL_A)){ // Intake forward
  //   global::intake.move_velocity(600);
  // } else if (master.get_digital(DIGITAL_B)) { // Intake reverse
  //   global::intake.move_velocity(-600);
  // } else {
  //   global::intake.brake();
  // }

  if ((clawDisEnabled == true) && (clawDis > 40) && (clawDis < 65) && (clawPos == true)){
    global::cascade.move_relative(800, 127);
    global::claw.retract(); // CLAW CLOSES
    clawCloseTime = pros::millis(); // Updates the time that claw closes
    clawDisEnabled = false; // Disables distance sensor
    clawPos = false; // Claw is closed
  }

  if ((clawDisEnabled == false) && (clawPos == true) && (curTime - clawOpenTime > 2000)){ // Re-enables claw distance sensor after 3 seconds after claw first opens
    clawDisEnabled = true;
  }


  // cascade pid states
  // if (master.get_digital_new_press(DIGITAL_X)){ 

  //     CLAWnextState(); // Go UP one state
  //     cascadeStateTime = pros::millis();

  //   } else if (master.get_digital_new_press(DIGITAL_L2)){ // go to the zero position

  //     CLAWzeroState(); // Go to ZERO state
  //     cascadeStateTime = pros::millis();

  //   } else if (master.get_digital_new_press(DIGITAL_B)){ // go back a state

  //     CLAWbackState(); // Go BACK one state
  //     cascadeStateTime = pros::millis();

  //   }

    // if (master.get_digital(DIGITAL_DOWN) && master.get_digital(DIGITAL_LEFT)){
    //   // fwd24();
    //   toggleOnly();
    // }
    //    if (master.get_digital(DIGITAL_DOWN) && master.get_digital(DIGITAL_RIGHT)){
    //   // fwd24();
    //   // leftZone();
    //   rightZone();
    // }
       // if (master.get_digital_new_press(DIGITAL_L1) && master.get_digital_new_press(DIGITAL_R1)){
    //   leftZone();
    //   toggleOnly();
    // }

    pros::delay(ez::util::DELAY_TIME);  
  }
}

