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

    if (parser.seen('Z') && parser.seen('E')) 
    {
        motors.move_delta_z(parser.floatval('Z'), parser.intval('E'));
    }
}
