//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M588() {
    if (parser.seen('X') && parser.seen('Y') && parser.seen('Z') && parser.seen('E') && parser.seen('W')) 
    {
        motors.ring_buf_all_counter_write_max = motors.ring_buf_lookup + motors.ring_buf_all_counter_write;
        
        if(motors.ring_buf_all_counter_write>= motors.ring_buf_all_counter_write_max) return;
        int cur_ind_ring = motors.ring_buf_all_counter_write%RING_BUF_NUM;
        motors.ring_buf_x[cur_ind_ring] = parser.longval('X');
        motors.ring_buf_y[cur_ind_ring] = parser.longval('Y');
        motors.ring_buf_z[cur_ind_ring] = parser.longval('Z');
        motors.ring_buf_e[cur_ind_ring] = parser.longval('E');
        motors.ring_buf_time[cur_ind_ring] = parser.longval('W');

        motors.ring_buf_all_counter_write++;
    }

    if (parser.seen('A'))  motors.ring_buf_en = parser.intval('A');
     if (parser.seen('B'))  motors.ring_buf_lookup = parser.intval('B');
     if (parser.seen('C'))  motors.ring_buf_end= parser.intval('C');
     if (parser.seen('D')) motors.ring_buf_counter = parser.intval('D');

}
