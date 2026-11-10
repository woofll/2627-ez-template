#include "main.h"

namespace global{
//pros::Motor motorName(portNumber, motorType);
//pros::sensorType sensorName (portNumber);
//pros::adi::Pneumatics pneumaticsName('portLetter', state(true/false));


// pros::Motor testMotor(1, pros::v5::MotorGearset::rpm_600);
pros::Motor intake(9, pros::v5::MotorGearset::rpm_600);
pros::Motor cascadeLeft(8, pros::v5::MotorGearset::rpm_200);//NOT CORRECT
pros::Motor cascadeRight(-10, pros::v5::MotorGearset::rpm_200);
pros::Distance clawDistance(19);
pros::MotorGroup cascade({8, -10});
pros::adi::Pneumatics claw('A', false);
pros::adi::Pneumatics wrist('B', true);

}