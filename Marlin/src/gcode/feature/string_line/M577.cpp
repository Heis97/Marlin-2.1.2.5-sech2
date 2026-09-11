#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M577() {
  #ifdef PRIMARY_PLATE
  if (parser.seen('V') && parser.seen('I')) 
  {
    motors.servo_counter_work_max[parser.intval('I')] = (long)parser.intval('V');
  } 



  #endif
}
