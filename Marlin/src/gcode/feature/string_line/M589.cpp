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

    if (parser.seen('S')) 
    {
        #ifdef KINEMATIK
        motors.ring_buf_en = false;
        #endif
        for(int i=0; i<8;i++)
        {
            motors._steps[i] = 0;
        }
    }

    if (parser.seen('A')) 
    {
        motors.delta_tcp_serach_x = parser.intval('A');

    }
    if (parser.seen('B')) 
    {
        motors.delta_tcp_serach_y = parser.intval('B');

    }
    if (parser.seen('C')) 
    {
        motors.delta_tcp_serach_z = parser.intval('C');

    }

    if (parser.seen('P')) 
    {
        motors.wait_time_counter_max = parser.longval('P');
        if(motors.wait_time_counter_max >0)
        {
            motors.wait_time_counter = 0;
            motors.wait_time_counter_en = 1;
            motors.wait_time_counter_done = 0;
        }
        else
        {
            motors.wait_time_counter_en = 0;
        }

    }
}
