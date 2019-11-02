#include "kal/kal.h"

#define DEBUG 1
#define ADC_DEBUG 0

//光センサ
#define LIGHT_SENSOR_L 14
#define LIGHT_SENSOR_R 27
int offset = 350;

//wave
kal::wave sin_wave(0.0,30,0.5,SIN);

//robot
#define MOTOR_NUM 2//4
kal::two_wheel_drive brobot;

//differentiator
kal::Diff<double> dtheta_st[MOTOR_NUM];
kal::Diff<double> dtheta_ref[MOTOR_NUM];
kal::Diff<double> dx_ref;

//時間管理//@todo: freeRTOSの導入検討
double t = 0.0;//time
bool timer_flag = 0;

//timer関連
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {  /* this function must be placed in IRAM */
  portENTER_CRITICAL_ISR(&timerMux);
  //control---------------------------------------------------------------------------------------------------------------------------/
  t += Ts;
  timer_flag = 1; 
  //-----------------------------------------------------------------------------------------------------------------------------------/
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  Serial.println("started");
  
  //motor1の設定
  brobot.motor[0].GPIO_setup(GPIO_NUM_4,GPIO_NUM_0);//方向制御ピン設定
  brobot.motor[0].PWM_setup(GPIO_NUM_2,0);//PWMピン設定
  brobot.motor[0].encoder_setup(PCNT_UNIT_0,GPIO_NUM_36,GPIO_NUM_39);//エンコーダカウンタ設定
  brobot.motor[0].set_fb_v_param(0.8,0.0,1.7);
  brobot.motor[0].set_fb_param(30,0.0,5.0);
  brobot.motor[0].set_fb_cc_param(30.0,0.0);

//  //motor2
  brobot.motor[1].GPIO_setup(GPIO_NUM_16,GPIO_NUM_17);//方向制御ピン設定
  brobot.motor[1].PWM_setup(GPIO_NUM_15,0);//PWMピン設定
  brobot.motor[1].encoder_setup(PCNT_UNIT_1,GPIO_NUM_34,GPIO_NUM_35);//エンコーダカウンタ設定
  brobot.motor[1].set_fb_v_param(0.8,0.0,1.7);
  brobot.motor[1].set_fb_param(30,0.0,5.0);
  brobot.motor[1].set_fb_cc_param(30.0,0.0);
//  //motor3
//  motor[2].GPIO_setup(GPIO_NUM_5,GPIO_NUM_21);//方向制御ピン設定
//  motor[2].PWM_setup(GPIO_NUM_13,0);//PWMピン設定
//  motor[2].encoder_setup(PCNT_UNIT_2,GPIO_NUM_32,GPIO_NUM_33);//エンコーダカウンタ設定
//  //motor4
//  motor[3].GPIO_setup(GPIO_NUM_22,GPIO_NUM_23);//方向制御ピン設定
//  motor[3].PWM_setup(GPIO_NUM_12,0);//PWMピン設定
//  motor[3].encoder_setup(PCNT_UNIT_3,GPIO_NUM_25,GPIO_NUM_26);//エンコーダカウンタ設定

  //車体の設定
  //brobot.set_param(56.0/2.0,200.0,3.0/5.0);//バリシャコタン
  brobot.set_param(82.0/2.0,200.0,1.0);

  //timer割り込み設定
  timer = timerBegin(0, 80, true);//プリスケーラ設定
  timerAttachInterrupt(timer, &onTimer, true);//割り込み関数指定
  timerAlarmWrite(timer, (int)(Ts*1000000), true);//Ts[s]ごとに割り込みが入るように設定
  timerAlarmEnable(timer);//有効化
}
void loop() {
  if(timer_flag){//制御周期
    timer_flag = 0;

    //状態取得---------------------------------------------------------------------------//
//    int light_sensor_l = analogRead(LIGHT_SENSOR_L);
//    int light_sensor_r = analogRead(LIGHT_SENSOR_R);
//    int error = light_sensor_l - light_sensor_r + offset;
  
    for(int i=0;i<MOTOR_NUM;i++){
      brobot.motor[i].get_angle(brobot.motor[i].state.q);  
    }
    for(int i=0;i<MOTOR_NUM;i++){
      dtheta_st[i].update(brobot.motor[i].state.q,brobot.motor[i].state.dq);  
    }
    brobot.odmetry_update();
    //------------------------------------------------------------------------------------//
    
    //目標値計算
    sin_wave.update();
    brobot.ref.x = sin_wave.output;
    dx_ref.update(brobot.ref.x,brobot.ref.dx);
  
    //出力計算
    brobot.position_control(brobot.ref.x);
  //  brobot.motor[0].position_control();
  //  brobot.motor[0].drive(brobot.motor[0].output);

#if DEBUG
    for(int i=0;i<MOTOR_NUM;i++){
      Serial.print(brobot.motor[i].ref.q * RAD2DEG);
      Serial.print(",");
      Serial.print(brobot.motor[i].state.q * RAD2DEG);     
      Serial.print(",");
    }
    Serial.println();
#endif
#if ADC_DEBUG
    Serial.print(light_sensor_l);
    Serial.print(",");
    Serial.print(light_sensor_r);     
    Serial.print(",");
    Serial.print(error);     
    Serial.println();
#endif
        
  }//制御周期
  else{//その他の処理
    
  }
}
