#include "main.h"
#include <memory>

auto ODOM_WHEEL_DIAMETER = 2.75_in;
auto ODOM_WHEEL_TRACK = 7_in;

auto MIDDLE_ODOM_DISTANCE = 1_in;
auto MIDDLE_ODOM_DIAMETER = 2.75_in;

int RIGHT_FRONT_DRIVE = -1;
int RIGHT_BACK_DRIVE = -2;
int LEFT_FRONT_DRIVE = 3;
int LEFT_BACK_DRIVE = 4;

int INTAKE = 5;
int INTAKE2 = 6;

int FLYWHEEL = 7;
int FLYWHEEL2 = 8;

char INDEXER = 'A';
char EXPANSION = 'B';

char LEFT_ODOM_TOP = 'C';
char LEFT_ODOM_BOTTOM = 'D';

char RIGHT_ODOM_TOP = 'E';
char RIGHT_ODOM_BOTTOM = 'F';

char BACK_ODOM_TOP = 'G';
char BACK_ODOM_BOTTOM = 'H';

pros::ADIDigitalOut indexer = pros::ADIDigitalOut(INDEXER);
pros::ADIDigitalOut expansion = pros::ADIDigitalOut(EXPANSION);

ControllerButton autonomousButton = ControllerButton(ControllerDigital::Y);

ControllerButton intakeInButton(ControllerDigital::L1);
ControllerButton intakeOutButton(ControllerDigital::L2);

ControllerButton flywheelButton(ControllerDigital::R1);
ControllerButton indexerButton(ControllerDigital::R2);

ControllerButton flywheelUpButton(ControllerDigital::up);
ControllerButton flywheelDownButton(ControllerDigital::down);

ControllerButton expansionButton(ControllerDigital::X);

std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
        .withMotors(LEFT_FRONT_DRIVE, LEFT_BACK_DRIVE, RIGHT_FRONT_DRIVE,
                    RIGHT_BACK_DRIVE)
        .withGains({0.001, 0, 0.0001}, {0.001, 0, 0.0001}, {0.001, 0, 0.0001})
        .withDimensions(AbstractMotor::gearset::green,
                        {{ODOM_WHEEL_DIAMETER, ODOM_WHEEL_TRACK,
                          MIDDLE_ODOM_DISTANCE, MIDDLE_ODOM_DIAMETER},
                         quadEncoderTPR})
        .withSensors(ADIEncoder{LEFT_ODOM_TOP, LEFT_ODOM_BOTTOM},
                     ADIEncoder{RIGHT_ODOM_TOP, RIGHT_ODOM_BOTTOM, true},
                     ADIEncoder{BACK_ODOM_TOP, BACK_ODOM_BOTTOM, false})
        .withOdometry()
        .buildOdometry();

std::shared_ptr<ChassisController> drive =
    ChassisControllerBuilder()
        .withMotors(LEFT_FRONT_DRIVE, LEFT_BACK_DRIVE, RIGHT_FRONT_DRIVE,
                    RIGHT_BACK_DRIVE)
        .withDimensions({AbstractMotor::gearset::green, (80.0 / 60.0)},
                        {{4_in, 11.5_in}, imev5GreenTPR})
        .build();

auto intake = AsyncVelControllerBuilder()
                  .withMotor(INTAKE)
                  .withGearset(AbstractMotor::gearset::green)
                  .build();

auto intake2 = AsyncVelControllerBuilder()
                   .withMotor(INTAKE2)
                   .withGearset(AbstractMotor::gearset::green)
                   .build();

auto flywheel = AsyncVelControllerBuilder()
                    .withMotor(FLYWHEEL)
                    .withGearset(AbstractMotor::gearset::green)
                    .build();

auto flywheel2 = AsyncVelControllerBuilder()
                     .withMotor(FLYWHEEL2)
                     .withGearset(AbstractMotor::gearset::green)
                     .build();

auto slowDrive = AsyncMotionProfileControllerBuilder()
                     .withLimits({0.1, 0.3, 5.0})
                     .withOutput(chassis)
                     .buildMotionProfileController();

auto medDrive = AsyncMotionProfileControllerBuilder()
                    .withLimits({0.9, 0.3, 5.0})
                    .withOutput(chassis)
                    .buildMotionProfileController();

auto fastDrive = AsyncMotionProfileControllerBuilder()
                     .withLimits({0.8, 0.5, 5.0})
                     .withOutput(chassis)
                     .buildMotionProfileController();

auto fasterDrive = AsyncMotionProfileControllerBuilder()
                       .withLimits({3.3, 2.7, 10.0})
                       .withOutput(chassis)
                       .buildMotionProfileController();

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "hey umunugusuku");
  pros::lcd::set_text(2, "mode: initialize");
  pros::lcd::set_text(
      3, "battery: " + std::to_string(pros::battery::get_current()) + " mA @ " +
             std::to_string(pros::battery::get_capacity()) + " mAh" + " (" +
             std::to_string(pros::battery::get_temperature()) + " C)");

  pros::lcd::set_text(
      4, "flywheel: " + std::to_string(flywheel->getTarget() * 25) + " RPM");
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
void autonomous() {
  // set the state to zero
  chassis->setState({0_in, 0_in, 0_deg});
  // turn 45 degrees and drive approximately 1.4 ft
  chassis->driveToPoint({1_ft, 1_ft});
  // turn approximately 45 degrees to end up at 90 degrees
  chassis->turnToAngle(90_deg);
  // turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north
  // of the robot
  chassis->turnToPoint({5_ft, 0_ft});
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
void opcontrol() {
  Controller controller;

  int flywheelSpeed = 50;

  while (true) {
    pros::lcd::set_text(
        3, "battery: " + std::to_string(pros::battery::get_current()) +
               " mA @ " + std::to_string(pros::battery::get_capacity()) +
               " mAh" + " (" +
               std::to_string(pros::battery::get_temperature()) + " C)");

    pros::lcd::set_text(
        4, "flywheel: " + std::to_string(flywheel->getTarget() * 25) + " RPM");

    drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                            controller.getAnalog(ControllerAnalog::rightY));

    if (intakeOutButton.isPressed()) {
      intake->setTarget(-100);
      intake2->setTarget(-100);
    } else if (intakeInButton.isPressed()) {
      intake->setTarget(100);
      intake2->setTarget(100);
    }

    if (flywheelUpButton.changedToPressed() && flywheelSpeed < 100 - 20) {
      flywheelSpeed += 20;
    } else if (flywheelDownButton.changedToPressed() &&
               flywheelSpeed > 0 + 20) {
      flywheelSpeed -= 20;
    }

    if (flywheelButton.isPressed() && flywheelSpeed != 0) {
      flywheel->setTarget(flywheelSpeed);
      flywheel2->setTarget(flywheelSpeed);
    } else {
      flywheel->setTarget(0);
      flywheel2->setTarget(0);
    }

    if (indexerButton.changedToPressed()) {
      indexer.set_value(1);
    } else if (indexerButton.changedToReleased()) {
      indexer.set_value(0);
    }

    if (expansionButton.changedToPressed()) {
      expansion.set_value(1);
    } else if (expansionButton.changedToReleased()) {
      expansion.set_value(0);
    }

    pros::delay(10);
  }
}
