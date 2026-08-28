//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M589() {
    if (parser.seen('X')) 
    {
        motors.home_delta(parser.floatval('X'));
    }
    if (parser.seen('Y')) 
    {
        motors.home_delta_calibr(parser.floatval('Y'));
    }
   /* if (parser.seen('A'))  motors.ring_buf_en = parser.intval('A');
     if (parser.seen('B'))  motors.ring_buf_lookup = parser.intval('B');
     if (parser.seen('C'))  motors.ring_buf_end= parser.intval('C');
     if (parser.seen('D')) motors.ring_buf_counter = parser.intval('D'); 
    if (parser.seen('F')) motors.ring_buf_all_counter_write = parser.intval('F');*/
}
