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
    ftm_pwm_duty(ftm0,ftm_ch3,0); //D0
    ftm_pwm_duty(ftm0,ftm_ch0,speed); //D1
}

void left_go_forward(uint32 speed){
     ftm_pwm_duty(ftm0,ftm_ch2,speed); //D2
     ftm_pwm_duty(ftm0,ftm_ch1,0); //D3
}

void left_go_backward(uint32 speed){
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

void go(int32 speed){
    if(speed >= 0) go_forward(speed);
    else go_backward(-speed);
}

const double MECHANICAL_BALANCE = 1;
const double KP_balance = 15.5;
const double KD_balance = 2;
const int16 DUTY_MAX = 300;
const int16 DUTY_MIN = 40;
const int16 DUTY_ING = 15;
int16 get_balance_duty(double angle, double gyro){
    double err = angle - MECHANICAL_BALANCE;
    //if(err>=0) err = pow(err,1/2.0);
    //if(err<0) err = -pow(-err,1/2.0);
    int16 duty = err*KP_balance + KD_balance*gyro;
    if(duty <= -DUTY_MAX) duty = -DUTY_MAX;
    if(duty > -DUTY_MAX && duty <= -DUTY_MIN) duty = duty;
    if(duty > -DUTY_MIN && duty <= -DUTY_ING) duty = -DUTY_MIN;
    if(duty > -DUTY_ING && duty <= DUTY_ING) duty = 0;
    if(duty > DUTY_ING && duty <= DUTY_MIN) duty = DUTY_MIN;
    if(duty > DUTY_MIN && duty <= DUTY_MAX) duty = duty;
    if(duty > DUTY_MAX)duty = DUTY_MAX;
    
    return duty;
}

char CLEAR_ENCODER_INTEGRAL_FLAG = 0;
#define KP_velocity 5.0f
float KI_velocity = KP_velocity/200.0f;
const int16 ENCODER_INTEGRAL_MAX = 10000;
int16 get_velocity_duty(int16 speed_left, int16 speed_right){
  static float velocity, encoder_least, encoder, movement, encoder_integral;
  encoder_least = (speed_left + speed_right) - 0;
  encoder *= 0.7f;
  encoder += encoder_least * 0.3f;
  encoder_integral += encoder;
  if(encoder_integral >= ENCODER_INTEGRAL_MAX) encoder_integral = ENCODER_INTEGRAL_MAX;
  if(encoder_integral <= -ENCODER_INTEGRAL_MAX) encoder_integral = -ENCODER_INTEGRAL_MAX;
  velocity = encoder * KP_velocity + encoder_integral * KI_velocity;
  if(CLEAR_ENCODER_INTEGRAL_FLAG == 1){
    CLEAR_ENCODER_INTEGRAL_FLAG = 0;
    encoder_integral = 0;
  }
  return velocity;
}

double angle = 0;
double gyro = 0;
int16 duty = 0 , duty_balance = 0, duty_velocity = 0;
int16 speed_left = 0;
int16 speed_right = 0;
char RUN_OR_STOP_FLAG = 0;

void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void exception_detection(){
  if(angle > MECHANICAL_BALANCE + 40 || angle < MECHANICAL_BALANCE - 40){
    CLEAR_ENCODER_INTEGRAL_FLAG = 1;
    RUN_OR_STOP_FLAG = 0;
  }else{
    RUN_OR_STOP_FLAG = 1;
  }
}
void get_attitude(void){
  Get_Gyro();
  Get_AccData();
  Data_Filter();
  Get_Attitude();
  KalmanFilter(pitch);
  angle = pitch;
  gyro = real_gyro_y;
}

int main(void)
{
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
    OLED_P6x8Str(0,1,"FTM Init Done...");
    systick_delay_ms(250);
    //PIT中断开始
    pit_init_ms(pit0,5);
    set_irq_priority(PIT0_IRQn,1);//PIT0中断是MPU的中断
    enable_irq(PIT0_IRQn);
    pit_init_ms(pit1,50);
    set_irq_priority(PIT1_IRQn,2);//PIT1中断是编码器的中断
    enable_irq(PIT1_IRQn);
    EnableInterrupts;
    OLED_P6x8Str(0,2,"PIT Init Done...");
    systick_delay_ms(250);
    //MPU6050初始化
    IIC_init();
    systick_delay_ms(50);
    while(InitMPU6050() != 0x00);
    OLED_P6x8Str(0,3,"MPU Init...");
    systick_delay_ms(200);
    OLED_P6x8Str(0,4,"PLZ KEEP STABLE");
    systick_delay_ms(250);
    MPU6050_Offset();//温漂校零，此时要求保持稳定，不要晃动！
    OLED_P6x8Str(0,5,"MPU Init Done");
    systick_delay_ms(200);
    OLED_P6x8Str(0,6,"Ready");
    systick_delay_ms(250);
    OLED_P6x8Str(0,7,"STABLE & UPRIGHT");
    //将小车直立稳定后 开始
    for(int i=0;i<10;i++){
      get_attitude();
      systick_delay_ms(20);
    }
    while(1){
      get_attitude();
      if(angle<MECHANICAL_BALANCE+8&&angle>MECHANICAL_BALANCE-8&&gyro<2&&gyro>-2){
        OLED_P6x8Str(0,8,"All Done");
        systick_delay_ms(250);
        RUN_OR_STOP_FLAG = 1;
        break;
      }
      systick_delay_ms(100);
    }
    while(1){
      exception_detection();
      if(RUN_OR_STOP_FLAG==1){
        OLED_Fill(0x00);
        OLED_P6x8Str(38,0,"Dashboard");
        OLED_P6x8Str(0,1,"ANGLE:");
        OLED_P6x8Str(0,2,"GYRO :");
        OLED_P6x8Str(0,3,"DUTY :");
        OLED_P6x8Str(0,4,"SPD-L:");
        OLED_P6x8Str(0,5,"SPD-R:");
        OLED_Print_Num1(42,1,angle);
        OLED_Print_Num1(42,2,gyro);
        OLED_Print_Num1(42,3,duty);
        OLED_Print_Num1(42,4,speed_left);
        OLED_Print_Num1(42,5,speed_right);
      }else{
        OLED_Fill(0x00);
        OLED_P6x8Str(43,0,"STOPPED");
        OLED_P6x8Str(0,1,"ANGLE:");
        OLED_P6x8Str(0,2,"GYRO :");
        OLED_Print_Num1(42,1,angle);
        OLED_Print_Num1(42,2,gyro);
      }
      systick_delay_ms(50);
    }
}


void PIT0_IRQHandler(void){
      PIT_FlAG_CLR(pit0);
      
      if(RUN_OR_STOP_FLAG==1){
        get_attitude();
        duty_balance = get_balance_duty(angle,gyro);
        //duty_velocity = get_velocity_duty(speed_left,speed_right);
        duty = duty_balance + duty_velocity;
        //go(duty);
      }else{
        get_attitude();
        go(0);
      }
}

void PIT1_IRQHandler(void){
  PIT_FlAG_CLR(pit1);
  if(RUN_OR_STOP_FLAG==1){
    speed_right = ftm_quad_get(ftm1);
    ftm_quad_clean(ftm1);
    speed_left = ftm_quad_get(ftm2);
    ftm_quad_clean(ftm2);
  }
}



