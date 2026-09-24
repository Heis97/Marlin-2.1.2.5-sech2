/*
StepDirDriverPos.h - библиотека управления STEP/DIR драйвером шагового двигателя

*/

#include "StepDirDriverPos.h"



int step_pins[AXIS_NUM] {X_STEP_PIN,  Y_STEP_PIN,  Z_STEP_PIN,  I_STEP_PIN,  J_STEP_PIN,  K_STEP_PIN,  U_STEP_PIN,  E0_STEP_PIN  };
int dir_pins [AXIS_NUM] {X_DIR_PIN,   Y_DIR_PIN,   Z_DIR_PIN,   I_DIR_PIN,   J_DIR_PIN,   K_DIR_PIN,   U_DIR_PIN,   E0_DIR_PIN   };
int en_pins  [AXIS_NUM] {X_ENABLE_PIN,Y_ENABLE_PIN,Z_ENABLE_PIN,I_ENABLE_PIN,J_ENABLE_PIN,K_ENABLE_PIN,U_ENABLE_PIN,E0_ENABLE_PIN};
int stop_pins[AXIS_NUM] {X_DIAG_PIN,  Y_DIAG_PIN,  Z_DIAG_PIN,  I_DIAG_PIN,  J_DIAG_PIN,  K_DIAG_PIN,  U_DIAG_PIN,  E0_DIAG_PIN  };


int servo_pins[SERVO_NUM]{SERVO_C1_PIN ,SERVO_C2_PIN };

volatile long servo_counter_work[SERVO_NUM]{0,0};
volatile long servo_counter_work_max[SERVO_NUM]{120,120};
volatile long servo_counter_20ms[SERVO_NUM]{0,0};
volatile long servo_counter_20ms_max[SERVO_NUM]{2000,2000}; //20ms
    



int _endstop_val[AXIS_NUM]{0,0,0,0,0,0,0,0 }; 
int end_inv[AXIS_NUM]{0, 0, 0, 1, 1, 1, 1, 0 }; 
long home_pos[AXIS_NUM]{32000,  32000,  32000,  0,  0,  0,  0,  0 }; 

int home_dir_sdp[AXIS_NUM] {1,  1,  1,  1,  1,  1,  1,  1 };
int motor_dir[AXIS_NUM] {1,  1,  1,  1,  1,  1,  1,  1 };
//int steps_pr_mm[AXIS_NUM] { 800, 160, 240,240, 240, 240, 120, 100  };
float steps_pr_mm[AXIS_NUM] { 80, 80, 80,80, 80, 80, 40, 80  };


float steps_pr_mm_orig[AXIS_NUM] { 80, 80, 80,94, 94, 94, 47, 100  };
float steps_pr_mm_k[AXIS_NUM] { 1, 1, 1, 1, 1, 1, 1, 1};




volatile bool do_step[AXIS_NUM]{false,false,false,false,false,false,false,false}; 
volatile bool _homing_need[AXIS_NUM]{false,false,false,false,false,false,false,false};     
volatile bool _homed[AXIS_NUM]{false,false,false,false,false,false,false,false};     



volatile int _vibro[AXIS_NUM]{0,  0,  0,  1,  1,  1,  1,  1 }; 
volatile int vibro_ampl[AXIS_NUM] {15,  15,  15,  10,  10,  10,  10,  10  };
volatile int vibro_counter[AXIS_NUM] {1,  1,  1,  1,  1,  1,  1,  1  };
volatile int cur_dir[AXIS_NUM] {1,  1,  1,  1,  1,  1,  1,  1  };
//#endif
int count_handl = 0;
#define DIV_VEL_ZERO 100000
StepDirDriverPos motors(step_pins, dir_pins, en_pins, stop_pins);

//#define DEBUG_STEP_DIR
#define DEBUG_STEP_DIR_TARGET 5
#define COUNT_HAND_END 300
//---------------------------- конструктор -----------------------------------
StepDirDriverPos::StepDirDriverPos (int* pinStep, int* pinDir, int* pinEn, int* pinStop) {
 // Serial.println("StepDirDriverPos::StepDirDriverPos");
  for (byte i=0; i<AXIS_NUM;i++)
  {

    _pinStep[i] = pinStep[i];
    _pinDir [i] = pinDir[i];
    _pinEn  [i] = pinEn[i];
    _pinStop[i] = pinStop[i];
    SET_OUTPUT(_pinStep[i]); WRITE(_pinStep[i], LOW);
    SET_OUTPUT(_pinDir[i]); WRITE(_pinDir[i], LOW);
    SET_OUTPUT(_pinEn[i]); WRITE(_pinDir[i], LOW);
     
    SET_INPUT(_pinStop[i]);
    _steps[i] = 0;
    //_fixStop = false;
    _divider[i] = DIV_VEL_ZERO;
    _divider_sub[i] = 0;
    _dividerCount[i] = 0;   
    _pos[i] = 0;
    
    do_step[i] = false;
    _homing_need[i] = false;


    setVelDest(2.1,i);
    setAcs(0.5f,i);
 
    _vel_prev[i] = 0;
    vibro_ampl[i] = 15;    
  }

  vibro_ampl[E0_AXIS] = 30;
  sleep(E_AXIS);
  pin_stop_delta_calibr = E0_DIAG_PIN;

  for(int i=0; i<SERVO_NUM;i++)
  {
    SET_OUTPUT(servo_pins[i]); 
    servo_counter_20ms_max[i] = 2000;
    servo_counter_work_max[i] = 120;
  }
}

long counter_steps = 0;
//------------------------------- управление коммутацией фаз
// метод должен вызываться регулярно с максимальной частотой коммутации фаз
void  StepDirDriverPos::control(byte num) {
   //Serial.println(num);



  if(do_step[num]) { 
  
  WRITE(_pinStep[num], LOW);
    do_step[num] = false; 
    //Serial.println("do_step false");
    if(num==0)
    {

      //debug_count  = -1;
    }
    
  };
  // делитель частоты коммутации
  if ( _steps[num] == 0 ) return;
  //двигатель не остановлен
  _dividerCount[num]++;  
  if ( _dividerCount[num] < _divider[num] ) return;  
  else 
  { 
    _dividerCount[num]= 0;
    if(_dividerCount_sub[num] > _divider_sub[num])
    {
      _dividerCount[num]= 1;
    };
    _dividerCount_sub[num]++;
    if(_dividerCount_sub[num]==100)
    {
      _dividerCount_sub[num] = 0;
    }


    /*if(_dividerCount_sub[num] > _divider_sub[num])
    {
      _dividerCount[num]= 1;
    };
    _dividerCount_sub[num]++;
    if(_dividerCount_sub[num]==100)
    {
      _dividerCount_sub[num] = 0;
    }*/
  };

  if ( _steps[num] != 0 ) {
    //Serial.println("do_step true");
    //debug_count  = ring_buf_cur_count;
    WRITE(_pinStep[num], HIGH); 
    do_step[num] = true;    
    
    if(num==0)
    {
     // debug_count  = 1;

    }
  }   

  if (_steps[num] > 0) 
  { _steps[num]--; _pos[num]++; } // вращение против часовой стрелки
  else
  { _steps[num]++; _pos[num]--; }// вращение по часовой стрелке           
       
}
long prev_count = 0;
long prev_time = 0;
float prev_x = 0;
float prev_y = 0;
float prev_z = 0;
float prev_e = 0;
bool buf_work = false;
void  StepDirDriverPos::ring_buf_control() 
{
  buf_work = true;
  int ring_buf_cur = ring_buf_counter%RING_BUF_NUM;//current command all and in ring
  
  if(ring_buf_cur_count>=ring_buf_time[ring_buf_cur] - prev_count )
  {
    prev_count = ring_buf_time[ring_buf_cur];
    ring_buf_cur_count = 0;
    //when next command
       
    gotopos(ring_buf_x[ring_buf_cur], X_AXIS );
    gotopos(ring_buf_y[ring_buf_cur], Y_AXIS );
    gotopos(ring_buf_z[ring_buf_cur], Z_AXIS );
    gotopos(ring_buf_e[ring_buf_cur], E_AXIS );

  
    long abs_time = ring_buf_time[ring_buf_cur];
    long cur_time = abs_time-prev_time;
    prev_time = abs_time;

    long abs_x = ring_buf_x[ring_buf_cur];
    long cur_x = abs_x - prev_x;
    prev_x = abs_x;

    if(cur_x==0) cur_x = 1;
    setDiv(abs((float)cur_time/(float)cur_x ), X_AXIS);
    _dividerCount[X_AXIS] = _divider[X_AXIS];

    long abs_y = ring_buf_y[ring_buf_cur];
    long cur_y = abs_y - prev_y;
    prev_y = abs_y;

    if(cur_y==0) cur_y = 1;
    setDiv(abs((float)cur_time/(float)cur_y) , Y_AXIS);
    _dividerCount[Y_AXIS] = _divider[Y_AXIS];
//---------------------------------------------------------------
    long abs_z = ring_buf_z[ring_buf_cur];
    long cur_z = abs_z - prev_z;
    prev_z = abs_z;

    if(cur_z==0) cur_z = 1;
    setDiv(abs((float)cur_time/(float)cur_z) , Z_AXIS);
    _dividerCount[Z_AXIS] = _divider[Z_AXIS];
//---------------------------------------------------------------
    long abs_e = ring_buf_e[ring_buf_cur];
    long cur_e = abs_e - prev_e;
    prev_e = abs_e;

    if(cur_e==0) cur_e = 1;
    setDiv(abs((float)cur_time/(float)cur_e) , E_AXIS);
    _dividerCount[E_AXIS] = _divider[E_AXIS];



    //-------------------
    ring_buf_counter++;

  }
  ring_buf_cur_count++;
  
}

long control_counter = 0;
bool servo_counter_work_en[SERVO_NUM] {false,false};
void  StepDirDriverPos::control_servo(byte num)
{
  servo_counter_20ms[num]++;
  if(servo_counter_20ms[num]>servo_counter_20ms_max[num])
  {
    servo_counter_work_en[num] = true;
    servo_counter_20ms[num] = 0L;
    WRITE(servo_pins[num],1);
  }

  if(servo_counter_work_en[num])
  {
    servo_counter_work[num]++;
    if(servo_counter_work[num]>servo_counter_work_max[num])
    {
      WRITE(servo_pins[num],0);
      servo_counter_work_en[num] = false;
      servo_counter_work[num] = 0L;
    }
  }
} 

long debug_prev_pos;
long debug_prev_count;

void  StepDirDriverPos::control() {
  //for (byte i=AXIS_NUM-1; i>0;i--){ control(i); }

  
#ifdef KINEMATIK
  if(ring_buf_en && ring_buf_counter<ring_buf_end) ring_buf_control();

  if(!ring_buf_en && buf_work)
  {
    buf_work = false;
    _steps[0] = 0;
    _steps[1] = 0;
    _steps[2] = 0;
    _steps[7] = 0;
  }
  #endif
  control_counter++;

  control(7);
  control(6);
  control(5);
  control(4);
  control(3);
  control(2);
  control(1);
  control(0);

  control_servo(0);
  control_servo(1);
  

  if( control_counter%10000 ==0)
  {
    debug_count =(long)( 1000000.0f/(float)(_pos[0]-debug_prev_pos));

      debug_prev_pos = _pos[0];
      debug_prev_count = control_counter;
  }
}
//------------------------------- запуск вращения
// инициирует поворот двигателя на заданное число шагов
void  StepDirDriverPos::step(long steps, byte num) { 

  if(steps==0 ) 
  {
    _steps[num]= 0; 
    _vel[num] = 0;
    _vel_prev[num] = 0;
    return;
  }
  
  if(num==7) WRITE(_pinEn[num], LOW);
  if ( SIGN(steps)*motor_dir[num] < 0 )//*motor_dir[num]
   {
    WRITE(_pinDir[num], LOW);
    cur_dir[num] = 0;
  }
  else 
  {
    WRITE(_pinDir[num], HIGH);
    cur_dir[num] = 1;
  }
  _steps[num]= (volatile long)steps;
}

void  StepDirDriverPos::step(float dist, byte num) {

  long d = dist_to_steps(dist,num);  
  step(d,num); 
}

void StepDirDriverPos::wake_up(byte num)
{
  WRITE(_pinEn[num], LOW);
}

void StepDirDriverPos::wake_up_all()
{
  for (byte i=0; i<AXIS_NUM;i++) wake_up(i);
}

void StepDirDriverPos::sleep(byte num)
{
  WRITE(_pinEn[num], HIGH);
}

void StepDirDriverPos::sleep_all()
{
  for (byte i=0; i<AXIS_NUM;i++) sleep(i);
}

void StepDirDriverPos::sleep_string(byte motors_tens[5])
{
  for (byte i=0; i<AXIS_NUM;i++) sleep(motors_tens[i]);
}

void  StepDirDriverPos::gotopos(long int koord, byte num) {
   

	  step(koord -_pos[num],num);
}


void  StepDirDriverPos::gotopos(float pos, byte num) {
	  gotopos(dist_to_steps(pos,num),num);
}
//------------------------------ режим коммутации фаз и остановки
//void  StepDirDriverPos::setMode(byte stepMode, boolean fixStop)  {  
//  _fixStop = fixStop;
//}

//------------------------------ установка делителя частоты для коммутации фаз
void StepDirDriverPos::setPos(long int pos, byte num)  {
  _pos[num] = pos;
} 

void StepDirDriverPos::setPos(float pos, byte num)  {
   setPos(dist_to_steps(pos,num),num);
} 


void StepDirDriverPos::setDivider(long int divider, byte num)  {
  _divider[num]  = divider; 
}

void StepDirDriverPos::set_cur_k(float k, byte num)  
{
  if(k<0) k = 0.00001f;
  steps_pr_mm_k[num] = k;
  steps_pr_mm[num] =  steps_pr_mm_k[num]*steps_pr_mm_orig[num];
}


void StepDirDriverPos::setVel(volatile float vel, byte num)  
{
  volatile float vel_steps =(volatile float)(vel*steps_pr_mm[num]);

  if(vel<0.0001) _vel_dest[num] = 0.0001;
  else           _vel_dest[num] = vel_steps;

  #ifdef DEBUG_STEP_DIR
  if(num==DEBUG_STEP_DIR_TARGET)
  {
    Serial.print("set vel ");
    Serial.println((float)_vel_dest[num]);
  }

  #endif
  
  setVelIntern(_vel_dest[num],num);
}

void StepDirDriverPos::setDiv(volatile float div, byte num)  
{
  double integerPart;
  double fractionalPart;

  fractionalPart = modf(double(div), &integerPart);

  _divider[num] = (long)integerPart;
  _divider_sub[num] = (int)(fractionalPart*100);
}

void  StepDirDriverPos::setVelIntern(volatile float vel, byte num)
{
  volatile float _vel_ch = vel;
  if(_vel_ch==0) return;
  double div = (104616.18f/_vel_ch);//FREQ_MOTORS
   #ifdef DEBUG_STEP_DIR
  if(num==DEBUG_STEP_DIR_TARGET && count_handl == COUNT_HAND_END-1)
  {
    Serial.print("vel ");
    Serial.print((float)vel);
    Serial.print("div ");
    Serial.print(div);
    Serial.print("_vel_dest ");
    Serial.println((float)_vel_dest[DEBUG_STEP_DIR_TARGET]);
  }
  #endif
  
  if(div<2) div = 2;
  _vel[num] = _vel_ch;
  setDiv(div,num);
  //_divider[num]  = (volatile long)div; 
};

void  StepDirDriverPos::setAcs(float acs, byte num)
{
  _acs[num] =  acs*steps_pr_mm[num]/1000.0f;
};

void StepDirDriverPos::setVelDest(volatile float vel, byte num)
{
  _time_ch_vel_prev[num] = micros();
  _vel_prev[num] = _vel[num];
  _vel_dest[num] =  vel*steps_pr_mm[num];
  #ifdef DEBUG_STEP_DIR  
  Serial.print("vel ");
  Serial.print((float)_vel[num]);
  Serial.print(";vel_dest ");
  Serial.println((float)_vel_dest[num]);
  #endif

};
void StepDirDriverPos::setVelDest_direct(volatile float vel, byte num)
{
  _time_ch_vel_prev[num] = micros();
  _vel_prev[num] = _vel[num];
  _vel_dest[num] =  vel;
  #ifdef DEBUG_STEP_DIR  
  Serial.print("vel ");
  Serial.print((float)_vel[num]);
  Serial.print(";vel_dest ");
  Serial.println((float)_vel_dest[num]);
  #endif

};

void  StepDirDriverPos::move_delta_z(float vel, int en)
{
    if(en == 0) 
    {
        step(0l,X_AXIS);
        step(0l,Y_AXIS);
        step(0l,Z_AXIS);
        return;
    }

    setVelDest(vel,X_AXIS);
    setVelDest(vel,Y_AXIS);
    setVelDest(vel,Z_AXIS);

    if(en>0)
    {
        step(1000000l,X_AXIS);
        step(1000000l,Y_AXIS);
        step(1000000l,Z_AXIS);
    }

    if(en<0)
    {
        step(-1000000l,X_AXIS);
        step(-1000000l,Y_AXIS);
        step(-1000000l,Z_AXIS);
    }

    return;

}
void StepDirDriverPos::set_motor_dir(int dir, byte num)  {

  motor_dir[num] = dir;
}

//----------------------------- чтение оставшихся шагов 
volatile long int* StepDirDriverPos::readSteps()  {
  //long int stp;
  //stp = _steps;
  return _steps;
}

//----------------------------- чтение текущей координаты
volatile long int* StepDirDriverPos::readPos()  {
  //long int poz;
  //poz = _poz;
  return _pos;
}
volatile long  StepDirDriverPos::readPosOne(byte num)  {
  //long int poz;
  //poz = _poz;
  return _pos[num];
}
volatile bool* StepDirDriverPos::readHoming()
{
  return _homing_need;
}

volatile bool StepDirDriverPos::readHoming_one(byte ax)
{
  return _homing_need[ax];
}

byte StepDirDriverPos::readEnd(byte num)
{
  return READ(_pinStop[num]);
}

float StepDirDriverPos::readVelDest(byte num)
{
  return _vel_dest[num];
};

float StepDirDriverPos::readVel(byte num)
{
  return _vel[num];
};
long int StepDirDriverPos::dist_to_steps(float dist, byte num)
{
   return (long int)(dist*steps_pr_mm[num]);
}

void StepDirDriverPos::home_axis(byte num)
{
  _homing_need[num] = true;
  step(home_dir_sdp[num] * 100000L,num);
}

void StepDirDriverPos::home_delta(float div_vel)
{
    setDiv(div_vel,X_AXIS);
    setDiv(div_vel,Y_AXIS);
    setDiv(div_vel,Z_AXIS);

    home_axis(X_AXIS);
    home_axis(Y_AXIS);
    home_axis(Z_AXIS);
}

void StepDirDriverPos::home_delta_calibr(float div_vel)
{
    setDiv(div_vel,X_AXIS);
    setDiv(div_vel,Y_AXIS);
    setDiv(div_vel,Z_AXIS);

    delta_calibr = true;
    step(0L,X_AXIS);
    step(0L,Y_AXIS);
    step(0L,Z_AXIS);
    step(-home_dir_sdp[X_AXIS]*100000L,X_AXIS);
    step(-home_dir_sdp[Y_AXIS]*100000L,Y_AXIS);
    step(-home_dir_sdp[Z_AXIS]*100000L,Z_AXIS);
}
void StepDirDriverPos::home_handler(byte _num)
{

  if(_num==0)
  {
    int end_val = READ(pin_stop_delta_calibr);
   
    if(delta_calibr )
    {
      if(end_val ==0)
      {
        step(0L,X_AXIS);
        step(0L,Y_AXIS);
        step(0L,Z_AXIS);
        delta_calibr = false;
      }
    }
  }
  
  _endstop_val[_num] = READ(_pinStop[_num]);  
  if(end_inv[_num]>0)
  {
    if(_endstop_val[_num]==0) _endstop_val[_num] = 1;
    else _endstop_val[_num] = 0;
  }

  if(!_homing_need[_num]) return;
  
  if(_endstop_val[_num]==1)
  {
    _homing_need[_num] = false;
    _homed[_num] = true;
    step(0L,_num);
    setPos(home_pos[_num] ,_num);
    // step(-home_dir_sdp[_num]*100L,_num);       
  }
  
  
}

void  StepDirDriverPos::home_handler()
{
  for (byte i=0; i<AXIS_NUM;i++){  home_handler(i); }    
}

void  StepDirDriverPos::vel_handler(byte _num)
{
    if(_steps[_num]== 0) {
       _vel[_num]= 0; 
       _vel_prev[_num]= 0; 
       _time_ch_vel_prev[_num] = _time_ch_vel;
       return;
      }
    unsigned long  _dt_time_ch_vel = _time_ch_vel-_time_ch_vel_prev[_num];

   
    
    _vel[_num]= _vel_prev[_num] +_acs[_num]*((volatile float)_dt_time_ch_vel)/1000;


    if(_vel[_num]>=_vel_dest[_num])
    {
      _vel[_num]=_vel_dest[_num];
    }

 
      
    
    setVelIntern(_vel[_num],_num);
    
}


void  StepDirDriverPos::vel_handler()
{
  _time_ch_vel = micros();

  if(ring_buf_en)
  {
    vel_handler(3);
    vel_handler(4);
    vel_handler(5);
    vel_handler(6);
  }
  else
  {
      vel_handler(0);
      vel_handler(1);
      vel_handler(2);
      vel_handler(3);
      vel_handler(4);
      vel_handler(5);
      vel_handler(6);
      vel_handler(7);
  }
}
int counter_idle = 0;
long counter_rest_e = 0;
long counter_rest_e_max = 10000;
bool e_en = false;
void StepDirDriverPos::idle()
{

 if(!ring_buf_en)
 {
    counter_idle++;
    if(counter_idle>100)
    {
      vel_handler();
      counter_idle=0;
    }   

 }
 if(ring_buf_counter>=ring_buf_end) ring_buf_en = false;
  home_handler();

  #ifdef KINEMATIK
  if(_steps[E_AXIS]==0)
  {
    counter_rest_e++;
    if(counter_rest_e>counter_rest_e_max &&e_en )
    {
        WRITE(_pinEn[E_AXIS],1);
        e_en = false;
    }
    
  }
  else
  {
    e_en = true;
    counter_rest_e = 0;
  }
  #endif
}