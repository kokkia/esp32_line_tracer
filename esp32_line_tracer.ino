#include "kal/kal.h"
#include<Wire.h>

#define DEBUG 0
#define ADC_DEBUG 1
#define COLOR 0

//光センサ
#define LIGHT_SENSOR_L 14
#define LIGHT_SENSOR_C 27
#define LIGHT_SENSOR_R 4//27
int offset = 350;

//カラーセンサ
#define SDA_PIN_COLOR_1 23//21
#define SCL_PIN_COLOR_1 22//22
#define SDA_PIN_COLOR_2 26//21
#define SCL_PIN_COLOR_2 25//22
TwoWire Wire2( 1 );//2個目のカラーセンサI2C
class color_data{
  public:
  int r;
  int g;
  int b;
  int a;
};

color_data cl1,cl2;

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
  
//  //motor1の設定
//  brobot.motor[0].GPIO_setup(GPIO_NUM_4,GPIO_NUM_0);//方向制御ピン設定
//  brobot.motor[0].PWM_setup(GPIO_NUM_2,0);//PWMピン設定
//  brobot.motor[0].encoder_setup(PCNT_UNIT_0,GPIO_NUM_36,GPIO_NUM_39);//エンコーダカウンタ設定
//  brobot.motor[0].set_fb_v_param(10.0,1.0,0.0);
//  brobot.motor[0].set_fb_param(30,0.0,5.0);
//  brobot.motor[0].set_fb_cc_param(50.0,0.0);
//
////  //motor2
//  brobot.motor[1].GPIO_setup(GPIO_NUM_16,GPIO_NUM_17);//方向制御ピン設定
//  brobot.motor[1].PWM_setup(GPIO_NUM_15,0);//PWMピン設定
//  brobot.motor[1].encoder_setup(PCNT_UNIT_1,GPIO_NUM_34,GPIO_NUM_35);//エンコーダカウンタ設定
//  brobot.motor[1].set_fb_v_param(10.0,1.0,0.0);
//  brobot.motor[1].set_fb_param(30,0.0,5.0);
//  brobot.motor[1].set_fb_cc_param(50.0,0.0);
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

#if COLOR
  Wire.begin(SDA_PIN_COLOR_1, SCL_PIN_COLOR_1); // SDA, SCL
  Wire2.begin(SDA_PIN_COLOR_2, SCL_PIN_COLOR_2); // SDA, SCL
  Wire.beginTransmission(0x2A);
  Wire.write(0x0);
  Wire.write(0x89);
  Wire.endTransmission();
  Wire.beginTransmission(0x2A);
  Wire.write(0x0);
  Wire.write(0x09);
  Wire.endTransmission();
  Wire2.beginTransmission(0x2A);
  Wire2.write(0x0);
  Wire2.write(0x89);
  Wire2.endTransmission();
  Wire2.beginTransmission(0x2A);
  Wire2.write(0x0);
  Wire2.write(0x09);
  Wire2.endTransmission();
#endif

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
    int light_sensor_l = analogRead(LIGHT_SENSOR_L);
    int light_sensor_c = analogRead(LIGHT_SENSOR_C);
    int light_sensor_r = analogRead(LIGHT_SENSOR_R);
    read_color();
    
//    int error = light_sensor_l - light_sensor_r + offset;
  
//    for(int i=0;i<MOTOR_NUM;i++){
//      brobot.motor[i].get_angle(brobot.motor[i].state.q);  
//    }
//    for(int i=0;i<MOTOR_NUM;i++){
//      dtheta_st[i].update(brobot.motor[i].state.q,brobot.motor[i].state.dq);  
//    }
//    brobot.odmetry_update();
//    //------------------------------------------------------------------------------------//
    
//    //目標値計算
//    sin_wave.update();
//    brobot.ref.x = sin_wave.output;
//    dx_ref.update(brobot.ref.x,brobot.ref.dx);
//    //brobot.motor[0].ref.dq = sin_wave.output/30*PI;
//  
//    //出力計算
//    brobot.position_control(brobot.ref.x);
//    //brobot.motor[0].drive(brobot.motor[0].velocity_control());
      
#if DEBUG
    for(int i=0;i<MOTOR_NUM-1;i++){
      Serial.print(brobot.motor[i].ref.q * RAD2DEG);
      Serial.print(",");
      Serial.print(brobot.motor[i].state.q * RAD2DEG);     
      Serial.print(",");
      //Serial.print(brobot.motor[i].output);     
      //Serial.print(",");
      
    }
    Serial.println();
#endif
#if ADC_DEBUG
    Serial.print(light_sensor_l);
    Serial.print(",");
    Serial.print(light_sensor_c);     
    Serial.print(",");
    Serial.print(light_sensor_r);     
    Serial.print(",");
//    Serial.print(error);     
    Serial.println();
#endif
        
  }//制御周期
  else{//その他の処理
    
  }
}


void read_color(void){
  //r:赤 g:緑 b:青 a:赤外
  int r,g,b,a;
  int h,l;
  Wire.beginTransmission(0x2A);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(0x2A,8);
  Serial.print(" 1: ");
  if(Wire.available()){
    //赤
    h = Wire.read();
    l = Wire.read();
    r = h << 8|l;
    Serial.print("r:");
    Serial.print(r);
    cl1.r = r;
    //緑
    h = Wire.read();
    l = Wire.read();
    g = h << 8|l;
    Serial.print(" g:");
    Serial.print(g);
    cl1.g = g;
    //青
    h = Wire.read();
    b = Wire.read();
    r = h << 8|l;
    Serial.print(" b:");
    Serial.print(b);
    cl1.b = b;
    //赤外
    h = Wire.read();
    l = Wire.read();
    a = h << 8|l;
    Serial.print(" a:");
    Serial.print(a);
    cl1.a = a;
    Serial.println("");
  }
  Wire.endTransmission();
  
  Serial.print(" 2: ");
  Wire2.beginTransmission(0x2A);
  Wire2.write(0x03);
  Wire2.endTransmission();
  Wire2.requestFrom(0x2A,8);
  if(Wire2.available()){
    //赤
    h = Wire2.read();
    l = Wire2.read();
    r = h << 8|l;
    Serial.print("r:");
    Serial.print(r);
    //緑
    h = Wire2.read();
    l = Wire2.read();
    g = h << 8|l;
    Serial.print(" g:");
    Serial.print(g);
    //青
    h = Wire2.read();
    b = Wire2.read();
    r = h << 8|l;
    Serial.print(" b:");
    Serial.print(b);
    //赤外
    h = Wire2.read();
    l = Wire2.read();
    a = h << 8|l;
    Serial.print(" a:");
    Serial.print(a);
    Serial.println("");
  }
  Wire2.endTransmission();
}
