#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"


void GcodeSuite::M581() {


 if (parser.seen('A') && parser.seen('I')) 
  {

        
    if(parser.intval('I')==0)  hal.set_pwm_duty(pin_t(FAN0_PIN), parser.intval('A'));
    if(parser.intval('I')==1)  hal.set_pwm_duty(pin_t(FAN1_PIN), parser.intval('A'));
    if(parser.intval('I')==2)  hal.set_pwm_duty(pin_t(FAN2_PIN), parser.intval('A'));
    if(parser.intval('I')==3)  hal.set_pwm_duty(pin_t(FAN3_PIN), parser.intval('A'));
    if(parser.intval('I')==4)  hal.set_pwm_duty(pin_t(FAN4_PIN), parser.intval('A'));
    if(parser.intval('I')==6)  hal.set_pwm_duty(pin_t(FAN5_PIN), parser.intval('A'));

  } 

}
