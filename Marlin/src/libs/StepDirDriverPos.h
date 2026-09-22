

#define AXIS_NUM 8
#define SERVO_NUM 2
#define RING_BUF_NUM 400
#include <Arduino.h>
#include "../inc/MarlinConfig.h"
#include "../module/settings.h"
#include "../module/planner.h"





class StepDirDriverPos {

  public:

      long control_counter;
      StepDirDriverPos(int* pinStep, int* pinDir, int* pinEn, int* pinStop); // конструктор
      void  control();  // управление, метод должен вызываться регулярно с максимальной частотой коммутации фаз.
      void  control_servo(byte num);  
      void  control(byte num); 
      void  ring_buf_control(); 
      void  home_axis(byte num);
      void  home_delta(float div_vel);
      void  home_delta_calibr(float div_vel);
      void  home_handler(byte num);
      void  home_handler();
      void  vel_handler(byte num);
      void  vel_handler();
      void  gotopos(long int koord, byte num);
      void  gotopos(float dist, byte num);
      void  step(long int steps, byte num);  // инициирует поворот двигателя на заданное число шагов
      void  step(float dist, byte num);  // инициирует поворот двигателя на заданное число шагов
      void  setPos(long int pos, byte num); 
      void  setPos(float dist, byte num); 
      void  setDivider(long int divider, byte num);  // установка делителя частоты для коммутации фаз
      void  set_cur_k(float k, byte num);
      void  setVel(float vel, byte num);
      void  setDiv(float div, byte num);
      void  setVelIntern(volatile float vel, byte num);
      void setVelDest(volatile float vel, byte num);
      void setVelDest_direct(volatile float vel, byte num);
      void  setAcs(float acs, byte num);
      void move_delta_z(float vel,int en);
      
      void  set_motor_dir(int dir, byte num);
      volatile long int* readSteps();
      volatile long int* readPos();
      volatile long readPosOne(byte num);
      volatile bool* readHoming();
      volatile bool readHoming_one(byte ax);
      long int dist_to_steps(float dist, byte num);
      byte readEnd(byte num);
      float readVelDest(byte num);
      float readVel(byte num);
      void idle();

      void wake_up(byte num);
      void wake_up_all();
      void sleep(byte num);
      void sleep_all();
      void sleep_string(byte motors_tens[5]);


//DEFAULT_AXIS_STEPS_PER_UNIT


      volatile int _vibro[AXIS_NUM]{ }; 
      volatile int vibro_ampl[AXIS_NUM] { };
      volatile int vibro_counter[AXIS_NUM] {};
      volatile int cur_dir[AXIS_NUM] {};
      
   
    volatile long ring_buf_x[RING_BUF_NUM]{ }; 
    volatile long ring_buf_y[RING_BUF_NUM]{ }; 
    volatile long ring_buf_z[RING_BUF_NUM]{ }; 
    volatile long ring_buf_e[RING_BUF_NUM]{ }; 
    volatile long ring_buf_time[RING_BUF_NUM]{ }; 
    volatile long ring_buf_dividerCount[RING_BUF_NUM]{ }; 
    volatile int ring_buf_dividerCount_sub[RING_BUF_NUM]{ }; 

    volatile long ring_buf_command_counter = 0;
    volatile long ring_buf_counter = 0;
    volatile long ring_buf_lookup = 200;
    volatile long ring_buf_end = 0;
    volatile int ring_buf_cur_count = 0;

    volatile bool delta_calibr = false;

    volatile long debug_count = 0;

    volatile unsigned long ring_buf_all_counter = 0;
    volatile unsigned long ring_buf_all_counter_write = 0;
    volatile unsigned long ring_buf_all_counter_write_max = 0;

    volatile bool ring_buf_en = false;
    volatile bool ring_buf_go = false;

    volatile long _steps[AXIS_NUM]{};// оставшееся число шагов 
    volatile long int _pos[AXIS_NUM]{}; 
      
    // boolean _fixStop[AXIS_NUM];  // признак фиксации положения при остановке
    
    volatile  long  int  _dividerCount[AXIS_NUM]{};  
    volatile  int  _dividerCount_sub[AXIS_NUM]{};  
    byte pin_stop_delta_calibr;
    unsigned long  _time_ch_vel;  
    unsigned long  _time_ch_vel_prev[AXIS_NUM]{}; 

    
    volatile  long  int  _divider[AXIS_NUM]{};  
    volatile  int  _divider_sub[AXIS_NUM]{}; 
    volatile float  _vel[AXIS_NUM]{};  
    volatile float  _vel_dest[AXIS_NUM]{};
    volatile float  _vel_prev[AXIS_NUM]{}; 
    volatile bool _homing_need[AXIS_NUM]{};
    
    int debug_val = 1;

    volatile float  _acs[AXIS_NUM]{};  
    int  _pinStop [AXIS_NUM]{};
    volatile int _endstop_val [AXIS_NUM]{};
    bool  end_inv [AXIS_NUM]{};
    float steps_pr_mm [AXIS_NUM]{};
    int home_dir_sdp[AXIS_NUM] {};
    long home_pos[AXIS_NUM]{};

    
    volatile long servo_counter_20ms[SERVO_NUM]{};
    volatile long servo_counter_work[SERVO_NUM]{};
    volatile long servo_counter_20ms_max[SERVO_NUM]{};
    volatile long servo_counter_work_max[SERVO_NUM]{};


  private:
    int _pinStep[AXIS_NUM]{};
    int  _pinDir[AXIS_NUM]{};
    int  _pinEn [AXIS_NUM]{};
    

      
      //long int koord[AXIS_NUM]{};     
      


      
      

} ;
extern StepDirDriverPos motors;