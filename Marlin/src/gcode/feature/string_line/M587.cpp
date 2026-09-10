//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M587() {
    #ifdef PRIMARY_PLATE
    if (parser.seen('I') && parser.seen('S')) { Serial.print(":M587 "); Serial.println(parser.floatval('S')); motors.step(parser.floatval('S'),parser.byteval('I')); }
    if (parser.seen('I') && parser.seen('P')) {   motors.gotopos(parser.floatval('P'),parser.byteval('I')); }//motors.setVel(0,parser.byteval('I'));  
    if (parser.seen('I') && parser.seen('M'))  motors.setVel(parser.floatval('M'),parser.byteval('I')); 
    if (parser.seen('I') && parser.seen('H'))  motors.home_axis(parser.byteval('I')); 
    if (parser.seen('I') && parser.seen('D'))  motors.set_motor_dir(parser.intval('D'),parser.byteval('I')); 

    if (parser.seen('F'))  { motors.sleep_all();}
    if (parser.seen('W'))  { motors.wake_up_all();}


    if (parser.seen('I') && parser.seen('A')) {  motors.setAcs(parser.floatval('A'),parser.byteval('I'));  }
    if (parser.seen('I') && parser.seen('V')) {  motors.setVelDest(parser.floatval('V'),parser.byteval('I'));  }
    if (parser.seen('I') && parser.seen('E')) {  motors.end_inv[parser.byteval('I')] = parser.intval('E');}
    if (parser.seen('I') && parser.seen('R')) {  motors.steps_pr_mm[parser.byteval('I')] = parser.floatval('R');}
    if (parser.seen('I') && parser.seen('K')) {  motors.home_dir_sdp[parser.byteval('I')] = parser.intval('K');}    
    if (parser.seen('I') && parser.seen('B')) {  motors.home_pos[parser.byteval('I')] = parser.longval('B');}

    if (parser.seen('U'))  { motors.sleep_string(string_manager.motors_tens);}

    #endif
}
