/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		IAR 8.3 or MDK 5.24
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-12-19
 * @note		
 ********************************************************************************************************************/
#include "headfile.h"
#include "math.h"
void right_go_backward(uint32 speed){
    if(speed > 500) speed = 500;
    ftm_pwm_duty(ftm0,ftm_ch3,speed); //D0
    ftm_pwm_duty(ftm0,ftm_ch0,0); //D1
}

void right_stop(){
    ftm_pwm_duty(ftm0,ftm_ch3,0); //D0
    ftm_pwm_duty(ftm0,ftm_ch0,0); //D1
}

void right_brake(){
    ftm_pwm_duty(ftm0,ftm_ch3,1000); //D0
    ftm_pwm_duty(ftm0,ftm_ch0,1000); //D1
}

void right_go_forward(uint32 speed){
    if(speed > 500) speed = 500;
    ftm_pwm_duty(ftm0,ftm_ch3,0); //D0
    ftm_pwm_duty(ftm0,ftm_ch0,speed); //D1
}

void left_go_forward(uint32 speed){
     if(speed > 500) speed = 500;
     ftm_pwm_duty(ftm0,ftm_ch2,speed); //D2
     ftm_pwm_duty(ftm0,ftm_ch1,0); //D3
}

void left_go_backward(uint32 speed){
     if(speed > 500) speed = 500;
     ftm_pwm_duty(ftm0,ftm_ch2,0); //D2
     ftm_pwm_duty(ftm0,ftm_ch1,speed); //D3
}

void left_stop(){
     ftm_pwm_duty(ftm0,ftm_ch2,0); //D2
     ftm_pwm_duty(ftm0,ftm_ch1,0); //D3
}

void left_brake(){
     ftm_pwm_duty(ftm0,ftm_ch2,1000); //D2
     ftm_pwm_duty(ftm0,ftm_ch1,1000); //D3
}

void go_forward(uint32 speed){
    left_go_forward(speed);
    right_go_forward(speed);
}

void go_backward(uint32 speed){
    left_go_backward(speed);
    right_go_backward(speed);
}

void stop(){
    left_stop();
    right_stop();
}

void brake(){
    left_brake();
    right_brake();
}

void go_liner(int32 speed){
    if(speed >= 0) go_forward(speed);
    else go_backward(-speed);
}

void go(int32 speed_left, int32 speed_right){
    if(speed_left >= 0) left_go_forward(speed_left);
    else left_go_backward(-speed_left);
    if(speed_right >= 0) right_go_forward(speed_right);
    else right_go_backward(-speed_right);
}

//直立环
const double MECHANICAL_BALANCE = 0;
const double KP_balance = 32 * 0.7;//32;//27.9 * 0.6;
const double KD_balance = 1.2 * 0.7;//2.2 * 0.6;
const int16 DUTY_MAX = 400;
int16 get_balance_duty(double angle, double gyro){
    double err = angle - MECHANICAL_BALANCE;
    int16 duty = err*KP_balance + KD_balance*gyro;
    if(duty < -DUTY_MAX) duty = -DUTY_MAX;
    if(duty > DUTY_MAX)duty = DUTY_MAX;
    return duty;
}

//速度环
#define KP_velocity 39//42//32.5f
float KI_velocity = KP_velocity/500.0f;
const int16 ENCODER_INTEGRAL_MAX = 10000;
float encoder_least, encoder, movement, encoder_integral;
int16 get_velocity_duty(int16 speed_left, int16 speed_right){
  encoder_least = (speed_left + speed_right) - 0;
  encoder *= 0.8f;
  encoder += encoder_least * 0.2f;
  encoder_integral += encoder/10.0f;
  if(encoder_integral >= ENCODER_INTEGRAL_MAX) encoder_integral = ENCODER_INTEGRAL_MAX;
  if(encoder_integral <= -ENCODER_INTEGRAL_MAX) encoder_integral = -ENCODER_INTEGRAL_MAX;
  int16 duty = encoder * KP_velocity + encoder_integral * KI_velocity;
  return duty;
}

//转向环
#define KP_turn 4
int16 get_turn_duty(int16 speed_left, int16 speed_right, double gyro_z){
  double err = gyro_z - 0;
  int16 duty = err * KP_turn;
  return duty;
}

//定义一些全局变量
double angle = 0;
double gyro = 0;
double gyro_z_axis = 0;
int16 duty_left = 0, duty_right = 0, duty_balance = 0, duty_velocity = 0, duty_turn = 0;
int16 speed_left = 0;
int16 speed_right = 0;
char CAR_STATUS = 3; //0:CAR STOP
                     //1:CAR RUN
                     //2:CAR WAITING (进入等待状态，直到小车稳定放正)

void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void exception_detection(){//异常检测}
  if(angle > MECHANICAL_BALANCE + 40 || angle < MECHANICAL_BALANCE - 40){
    //RUN_OR_STOP_FLAG = 0;
    CAR_STATUS = 2;
    encoder_integral = 0;
  }else{
    //RUN_OR_STOP_FLAG = 1;
    CAR_STATUS = 1;
  }
}
void get_attitude(void){//获取姿态}
  Get_Gyro();
  Get_AccData();
  Data_Filter();
  Get_Attitude();
  KalmanFilter(pitch);
  angle = pitch;
  gyro = real_gyro_y;
  gyro_z_axis = real_gyro_z;
}

void car_wait(void){//等待，直到小车稳定放正}
  for(int i=0;i<100;i++){
      get_attitude();
  }
  while(1){
      get_attitude();
      if(angle<MECHANICAL_BALANCE+15&&angle>MECHANICAL_BALANCE-15&&gyro<3&&gyro>-3){
        OLED_Fill(0x00); 
        OLED_P6x8Str(43,0,"DONE.");
        systick_delay_ms(250);
        CAR_STATUS = 1;
        break;
      }else{
        OLED_Fill(0x00);
        OLED_P6x8Str(43,0,"WAITING");
        OLED_P6x8Str(0,2,"ANGLE:");
        OLED_P6x8Str(0,3,"GYRO :");
        OLED_Print_Num1(42,2,angle);
        OLED_Print_Num1(42,3,gyro);
      }
      systick_delay_ms(5);
    }
}

int main(void){
    {//一系列初始化}
      get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
    //uart_init(uart2,9600);
    
    //OLED初始化
    OLED_Init();
    OLED_P6x8Str(0,0,"Starting...");
    systick_delay_ms(250);
    //FTM电机初始化
    ftm_pwm_init(ftm0,ftm_ch3,50,0); //D0
    ftm_pwm_init(ftm0,ftm_ch0,50,0); //D1
    ftm_pwm_init(ftm0,ftm_ch2,50,0); //D2
    ftm_pwm_init(ftm0,ftm_ch1,50,0); //D3
    //FTM正交解码初始化
    ftm_quad_init(ftm1);
    ftm_quad_init(ftm2);
    OLED_P6x8Str(0,2,"FTM Init Done...");
    systick_delay_ms(250);
    //PIT中断开始
    pit_init_ms(pit0,5);
    set_irq_priority(PIT0_IRQn,1);//PIT0中断是MPU的中断
    enable_irq(PIT0_IRQn);
    pit_init_ms(pit1,10);
    set_irq_priority(PIT1_IRQn,2);//PIT1中断是编码器的中断
    enable_irq(PIT1_IRQn);
    EnableInterrupts;
    OLED_P6x8Str(0,3,"PIT Init Done...");
    systick_delay_ms(250);
    //MPU6050初始化
    IIC_init();
    systick_delay_ms(50);
    while(InitMPU6050() != 0x00);
    OLED_P6x8Str(0,4,"MPU Init...");
    systick_delay_ms(250);
    OLED_P6x8Str(0,5,"PLZ KEEP STABLE");
    systick_delay_ms(250);
    MPU6050_Offset();//温漂校零，此时要求保持稳定，不要晃动！
    OLED_P6x8Str(0,6,"MPU Init Done");
    systick_delay_ms(250);
    OLED_P6x8Str(0,7,"Ready");
    systick_delay_ms(250);
    }
    car_wait();
    while(1){//OLED显示}
      if(CAR_STATUS == 1){
        OLED_Fill(0x00);
        OLED_P6x8Str(38,0,"Dashboard");
        OLED_P6x8Str(0,2,"ANGLE:");
        OLED_P6x8Str(0,3,"GYRO :");
        //OLED_P6x8Str(0,4,"DUTY :");
        OLED_P6x8Str(0,5,"SPD-L:");
        OLED_P6x8Str(0,6,"SPD-R:");
        OLED_Print_Num1(42,2,angle*10);
        OLED_Print_Num1(42,3,gyro);
        //OLED_Print_Num1(42,4,duty);
        OLED_Print_Num1(42,5,speed_left);
        OLED_Print_Num1(42,6,speed_right);
      }else if(CAR_STATUS == 0){
        OLED_Fill(0x00);
        OLED_P6x8Str(43,0,"STOPPED");
        OLED_P6x8Str(0,2,"ANGLE:");
        OLED_P6x8Str(0,3,"GYRO :");
        OLED_Print_Num1(42,2,angle);
        OLED_Print_Num1(42,3,gyro);
      }
      systick_delay_ms(50);
    }
}


void PIT0_IRQHandler(void){
      PIT_FlAG_CLR(pit0);
      if(CAR_STATUS==1){//状态正常
        get_attitude();
        duty_balance = get_balance_duty(angle,gyro);
        duty_velocity = get_velocity_duty(speed_left,speed_right);
        duty_turn = get_turn_duty(speed_left, speed_right, gyro_z_axis);
        duty_left = duty_balance + duty_velocity - duty_turn;
        duty_right = duty_balance + duty_velocity + duty_turn;
        go(duty_left, duty_right);
        exception_detection();
      }else{//状态为停止或等待
        if(CAR_STATUS == 0){//停止
          //get_attitude();
          go(0,0);
        }else if(CAR_STATUS == 2){//等待
          go(0,0);
          CAR_STATUS == 3;
          car_wait();
        }
        
      }
}

void PIT1_IRQHandler(void){
  PIT_FlAG_CLR(pit1);
  if(CAR_STATUS == 1){
    speed_right = ftm_quad_get(ftm1);
    ftm_quad_clean(ftm1);
    speed_left = ftm_quad_get(ftm2);
    ftm_quad_clean(ftm2);
  }
}



