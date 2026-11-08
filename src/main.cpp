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
int CLAWstates[CLAWnumStates] = {0, 50, 100, 200, 400}; // 50 = 20deg, 100 = 40 deg; 2.5 = 1deg
int CLAWcurrState = 0;
int CLAWtarget = 0;

void CLAWnextState() {
  CLAWcurrState += 1;
  if (CLAWcurrState == 5) {
    CLAWcurrState = 0;
  }

  CLAWtarget = CLAWstates[CLAWcurrState];
}
void CLAWbackState() {
  CLAWcurrState -= 1;
  if (CLAWcurrState == 0) {
    CLAWcurrState = 5;
  }

  CLAWtarget = CLAWstates[CLAWcurrState];
}

void CLAWliftControl() {
  double kp = 0.72;
  double error = CLAWtarget - (global::testMotor.get_position());
  double velocity = kp * error;
  global::testMotor.move(velocity); 
}
/************************************************************************************************************************** */


void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(2);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(4.0, 4.0);  // Defaults for curve. If using tank, only the first parameter is used.

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"only toggle!", toggleOnly},
  });

  // Initialize chassis and auton selector
  chassis.initialize(); 
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

  global::cascadeLeft.set_zero_position(0);
  global::cascadeRight.set_zero_position(0);
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
  
  double clawTime = 0;
  bool clawDisEnabled = true;
  double clampPressed = 0;
  bool testBool = false;
  double macroDown = 0;

    bool wristPos = true; // False = vertical
    bool clawPos = false; // False = closed

  
  while (true) {
    ez_template_extras();   

    // chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    global::cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    double cascadePos = global::cascade.get_position();
    double clawDis = global::clawDistance.get();



        // pros::lcd::set_text(5, std::to_string(chassis.drive_sensor_left()));
        pros::lcd::set_text(5, std::to_string(cascadePos));
        pros::lcd::set_text(6, std::to_string(clawDisEnabled));
    // pros::lcd::set_text(6, std::to_string(chassis.drive_sensor_right()));
    //     pros::lcd::set_text(7, std::to_string(chassis.odom_theta_get()));
  



  if (master.get_digital_new_press(DIGITAL_UP)){ // R1 pressed, open/close
    if (wristPos == true){ // claw = closed, so open claw
      global::wrist.extend();
      wristPos = false;
    } else if (wristPos == false){ // claw = open, so close claw
      global::wrist.retract();
      wristPos = true;
    }
  } 

  if (master.get_digital_new_press(DIGITAL_L1)){ // R2 pressed, hori/vert
    if (clawPos == false){ // claw = open
      global::claw.extend();
      clawPos = true;
    } else if (clawPos == true){ // claw = closed
      global::claw.retract();
      global::cascade.move_relative(500, 127);
      clawPos = false;
      clawDisEnabled = false;
      clawTime = pros::millis();
      clampPressed = pros::millis();
    }
  } 

  if (master.get_digital(DIGITAL_R1)){ // Cascade UP
      global::cascade.move_velocity(200);
    } else if (master.get_digital(DIGITAL_R2)){// Cascade DOWN
      global::cascade.move_velocity(-150);
    } else {
      if ((pros::millis() - clampPressed > 450) || (pros::millis() - macroDown > 1500) ){
      global::cascade.brake(); 
      }
    }

  if (master.get_digital_new_press(DIGITAL_Y)) {
    global::cascade.move_absolute(100, 200);
    macroDown = pros::millis();
}

  if (master.get_digital(DIGITAL_A)){ // Intake fwd/rev
      global::intake.move_velocity(600);
  } else if (master.get_digital(DIGITAL_B)) {
      global::intake.move_velocity(-600);
  } else {
      global::intake.brake();
  }

  if (!clawDisEnabled && (pros::millis() - clawTime > 1000)){
    clawDisEnabled = true;
  }

  if (clawDisEnabled && clawDis > 40 && clawDis < 65 && clawPos == true){
    global::cascade.move_relative(500, 127);
    global::claw.retract(); // CLAW CLOSES
    clawTime = pros::millis();
    clampPressed = pros::millis();
    clawPos = false;
  }

  

  // if (master.get_digital_new_press(DIGITAL_X)){ // cascade pid states
  //     CLAWnextState();
  //   } else if (master.get_digital_new_press(DIGITAL_Y)){ // go to the zero position
  //     CLAWcurrState = 0;
  //     CLAWtarget = CLAWstates[CLAWcurrState];
  //   } else if (master.get_digital_new_press(DIGITAL_B)){ // go back a state
  //     CLAWbackState();
  //   }

    // if (master.get_digital(DIGITAL_DOWN) && master.get_digital(DIGITAL_LEFT)){
    //   // fwd24();
    //   toggleOnly();
    // }
    //    if (master.get_digital(DIGITAL_DOWN) && master.get_digital(DIGITAL_RIGHT)){
    //   // fwd24();
    //   leftZone();
    // }
       // if (master.get_digital_new_press(DIGITAL_L1) && master.get_digital_new_press(DIGITAL_R1)){
    //   leftZone();
    //   toggleOnly();
    // }

    pros::delay(ez::util::DELAY_TIME);  
  }
}
