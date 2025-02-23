#include "definitions.h"
inline void rollerControl() {
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    lift.move_voltage(-12000);
    Intake1.move_voltage(-12000);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    lift.move_voltage(12000);
    Intake1.move_voltage(12000);
  }
  else {
    lift.move(0);
    Intake1.move(0);
  }
}

