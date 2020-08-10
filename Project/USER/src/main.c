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

#define MECHANICAL_BALANCE 11
#define KP_balance 0.3
#define KD_balance 0
int16 DUTY_MAX = 300;
int16 get_balance_duty(int16 angle, int16 gyro){
    double err = angle - MECHANICAL_BALANCE;
    if(err>=0) err = pow(err,1/2.0);
    if(err<0) err = -pow(-err,1/2.0);
    int16 duty = err*KP_balance + gyro * KD_balance;
    if(duty > DUTY_MAX)duty = DUTY_MAX;
    if(duty < -DUTY_MAX)duty = -DUTY_MAX;
    return duty;
}

int16 angle = 0;
int16 gyro = 0;
int16 duty = 0;
int16 speed_left = 0;
int16 speed_right = 0;

void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);

int main(void)
{
    get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
    //此处编写用户代码(例如：外设初始化代码等)
    //uart_init(uart2,9600);
    OLED_Init();
    //OLED_P8x16Str(0,0,"Hello");
    
    ftm_pwm_init(ftm0,ftm_ch3,50,0); //D0
    ftm_pwm_init(ftm0,ftm_ch0,50,0); //D1
    ftm_pwm_init(ftm0,ftm_ch2,50,0); //D2
    ftm_pwm_init(ftm0,ftm_ch1,50,0); //D3
    
    ftm_quad_init(ftm1);
    ftm_quad_init(ftm2);
    
    pit_init_ms(pit0,100);
    set_irq_priority(PIT0_IRQn,1);//设置优先级,根据自己的需求设置
    enable_irq(PIT0_IRQn);//打开pit0的中断开关
    //EnableInterrupts;
    pit_init_ms(pit1,50);
    set_irq_priority(PIT1_IRQn,2);
    enable_irq(PIT1_IRQn);
    EnableInterrupts;
    
    IIC_init();
    systick_delay_ms(50);
    while(InitMPU6050() != 0x00);
    MPU6050_Offset();
    
    for(;;){
      Get_Gyro();
      Get_AccData();
      Data_Filter();
      Get_Attitude();
      KalmanFilter(pitch);
      angle = pitch*10; //.
      gyro = real_gyro_y*10; //均保留一位小数，转为int16
      duty = get_balance_duty(angle,gyro);
      go(duty);
      //systick_delay_ms(10);
    }
}

//int count = 0;
void PIT0_IRQHandler(void){
      PIT_FlAG_CLR(pit0);
//      int y = 31-angle;
//      if(y>63)y=63;
//      if(y<0)y=0;
//      OLED_PutPixel(count++,y);
//      if(count == 128){
//        OLED_Fill(0x00);
//        count = 0;
//      }
      OLED_Fill(0x00);
      OLED_P6x8Str(42,0,"WDNMD");
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
}

void PIT1_IRQHandler(void){
  PIT_FlAG_CLR(pit1);
  speed_left = ftm_quad_get(ftm1);
  ftm_quad_clean(ftm1);
  speed_right = ftm_quad_get(ftm2);
  ftm_quad_clean(ftm2);
}



