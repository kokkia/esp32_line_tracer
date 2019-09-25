#include "kal/kal.h"

#define DEBUG 1
#define ADC_DEBUG 0

//光センサ
#define LIGHT_SENSOR_L 14
#define LIGHT_SENSOR_R 27
int offset = 350;

//motor
#define MOTOR_NUM 4
kal::nxtmotor motor[MOTOR_NUM];//4 motor
//motor control gain
#define KP 30.0
#define KD 5.0
#define KDD 0.05

//reference
kal::wave sin_wave(0.0,PI/3,0.5,TRIANGLE);

//robotdata
kal::RobotData<double> ref[MOTOR_NUM];
kal::RobotData<double> state[MOTOR_NUM];

//differentiator
kal::Diff<double> dtheta_st[MOTOR_NUM];
kal::Diff<double> dtheta_ref[MOTOR_NUM];

double t = 0.0;//time

//timer関連
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {  /* this function must be placed in IRAM */
  portENTER_CRITICAL_ISR(&timerMux);
  //control---------------------------------------------------------------------------------------------------------------------------/
  t += Ts;
  //状態取得
  int light_sensor_l = analogRead(LIGHT_SENSOR_L);
  int light_sensor_r = analogRead(LIGHT_SENSOR_R);
  int error = light_sensor_l - light_sensor_r + offset;
  
  for(int i=0;i<MOTOR_NUM;i++){
    motor[i].get_angle(state[i].theta);  
  }
  for(int i=0;i<MOTOR_NUM;i++){
    dtheta_st[i].update(state[i].theta,state[i].dtheta);  
  }  
  
  //目標値計算
  sin_wave.update();
  for(int i=0;i<MOTOR_NUM;i++){
    ref[i].theta = sin_wave.output;
    dtheta_ref[i].update(ref[i].theta,ref[i].dtheta);
  }
  
  //出力計算
//  for(int i=0;i<MOTOR_NUM;i++){
//    //double u = KP*(ref[i].theta-state[i].theta) + KD * (ref[i].dtheta - state[i].dtheta);
//    motor[i].drive(u);
//  }
  double u = 0.02 * error;
  motor[0].drive(-(3.0 - u));
  motor[1].drive(-(3.0 + u));

#if DEBUG
  for(int i=0;i<MOTOR_NUM;i++){
    Serial.print(ref[i].theta*RAD2DEG);
    Serial.print(",");
//    Serial.print(state[i].theta*RAD2DEG);     
//    Serial.print(",");
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
  //-------------------------------------------------------------------------------------------------------------------------------------/
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  Serial.println("started");
  
  //motor1の設定
  motor[0].GPIO_setup(GPIO_NUM_4,GPIO_NUM_0);//方向制御ピン設定
  motor[0].PWM_setup(GPIO_NUM_2,0);//PWMピン設定
  motor[0].encoder_setup(PCNT_UNIT_0,GPIO_NUM_36,GPIO_NUM_39);//エンコーダカウンタ設定
  //motor2
  motor[1].GPIO_setup(GPIO_NUM_16,GPIO_NUM_17);//方向制御ピン設定
  motor[1].PWM_setup(GPIO_NUM_15,0);//PWMピン設定
  motor[1].encoder_setup(PCNT_UNIT_1,GPIO_NUM_34,GPIO_NUM_35);//エンコーダカウンタ設定
  //motor3
  motor[2].GPIO_setup(GPIO_NUM_5,GPIO_NUM_21);//方向制御ピン設定
  motor[2].PWM_setup(GPIO_NUM_13,0);//PWMピン設定
  motor[2].encoder_setup(PCNT_UNIT_2,GPIO_NUM_32,GPIO_NUM_33);//エンコーダカウンタ設定
  //motor4
  motor[3].GPIO_setup(GPIO_NUM_22,GPIO_NUM_23);//方向制御ピン設定
  motor[3].PWM_setup(GPIO_NUM_12,0);//PWMピン設定
  motor[3].encoder_setup(PCNT_UNIT_3,GPIO_NUM_25,GPIO_NUM_26);//エンコーダカウンタ設定

  //timer割り込み設定
  timer = timerBegin(0, 80, true);//プリスケーラ設定
  timerAttachInterrupt(timer, &onTimer, true);//割り込み関数指定
  timerAlarmWrite(timer, (int)(Ts*1000000), true);//Ts[s]ごとに割り込みが入るように設定
  timerAlarmEnable(timer);//有効化
}
void loop() {
}
