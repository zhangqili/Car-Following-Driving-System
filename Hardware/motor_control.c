/*
 * motor_control.c
 *
 *  Created on: Aug 15, 2022
 *      Author: HP
 */
#include "motor_control.h"
motor_param motor_l={0,0,0};
motor_param motor_r={0,0,0};
motor_param motor={0,0,0};

float Gyro_x;        
float Gyro_y;       
float Gyro_z;        

float Angle_gx;    
float Angle_gy;    
float Angle_gz;  

float GX;
float GY;
float GZ;//去零漂以后的角速度

float GX_ZERO=-0.12;
float GY_ZERO=0.4;
float GZ_ZERO=-0.12;

void angle_gyro_z()
{
	GZ=(int)((Gyro_z-GZ_ZERO)*1000)/1000;
	Angle_gz = Angle_gz + GZ*0.002f;
}
/**
 * @function: void Get_Encoder_Counter(int *Encoder_Left_Counter,int *Encoder_Right_Counter)
 * @description: 获取编码器计数值
 * @param {int*} *Encoder_Left_Counter 数据 :左电机编码器的值
 * @param {int*} *Encoder_Right_Counter 数据 :右电机编码器的值
 * @return {*}
 */
 /*
void Get_Encoder_Counter(int *Encoder_Left_Counter,int *Encoder_Right_Counter)
{
	*Encoder_Left_Counter =(short) __HAL_TIM_GET_COUNTER(&htim3);     //保存编码器计数器的值
	*Encoder_Right_Counter =(short) __HAL_TIM_GET_COUNTER(&htim4);
	__HAL_TIM_SET_COUNTER(&htim3,0);                   //保存之后要清零,以便下次继续读取.另外每次清零后采样值减0,直接用单位时间的话就可以得出速度信息了.不要麻烦还要减去初值了.
	__HAL_TIM_SET_COUNTER(&htim4,0);
}*/
float SecondOrderLagFilter_Motor(uint8_t value)
{
    static float last_value,last_2_value;
    value=0.2*value+0.4*last_value+0.4*last_2_value;     //�����ͺ��˲�
    last_2_value=last_value;
    last_value=value;
    return value;
}
float SecondOrderLagFilter_L(int8_t value)
{
    static float last_value,last_2_value;
    value=0.2*value+0.4*last_value+0.4*last_2_value;     //�����ͺ��˲�
    last_2_value=last_value;
    last_value=value;
    return value;
}

float SecondOrderLagFilter_R(int8_t value)
{
    static float last_value,last_2_value;
    value=0.2*value+0.4*last_value+0.4*last_2_value;     //�����ͺ��˲�
    last_2_value=last_value;
    last_value=value;
    return value;
}

void Get_Encoder()
{
	motor_l.Encoder=(short) __HAL_TIM_GET_COUNTER(&htim3);     //保存编码器计数器的值
	motor_r.Encoder=(short) __HAL_TIM_GET_COUNTER(&htim4);
	//motor.Encoder=0.5*(motor_l.Encoder+motor_r.Encoder);
}

void Get_Speed(void)
{
	Get_Encoder();
	motor_l.Encoder=SecondOrderLagFilter_L(motor_l.Encoder);
	motor_r.Encoder=SecondOrderLagFilter_R(motor_r.Encoder);
//	motor.Encoder=SecondOrderLagFilter_Motor(motor.Encoder);
	__HAL_TIM_SET_COUNTER(&htim3,0);                   //保存之后要清零,以便下次继续读取.另外每次清零后采样值减0,直接用单位时间的话就可以得出速度信息了.不要麻烦还要减去初值了.
	__HAL_TIM_SET_COUNTER(&htim4,0);
}

/**
 * @function: void Limit(int *PWMA,int *PWMB)
 * @description: PWM限幅函数:避免PWM过大超过马达的机械极限,增加电机寿命
 * @param {int*} *PWMA 数据 :左电机PWM值
 * @param {int*} *PWMB 数据 :右电机PWM值
 * @return {*}
 */
void Limit(float *PWMA,float *PWMB)
{
	*PWMA=*PWMA>3000?3000:(*PWMA<-3000?-3000:*PWMA);
	*PWMB=*PWMB>3000?3000:(*PWMB<-3000?-3000:*PWMB);
}


/**
 * @function: void Give_Motor_PWM(int MotorL_PWM,int MotorR_PWM)
 * @description: 赋值PWM，控制电机正反转
 * @param {int} MotorL_PWM 数据 :左电机PWM值
 * @param {int}  MotorR_PWM数据 :右电机PWM值
 * @return {*}
 */
void Give_Motor_PWM(int MotorL_PWM,int MotorR_PWM)
{
	if (MotorL_PWM>0) //左电机正转
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
	}
	else              //左电机反转
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
	}
	if (MotorR_PWM>0) //右电机正转
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
	}
	else              //右电机反转
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
	}

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, ABS(MotorL_PWM)+250);//180为死区电压对应的值
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, ABS(MotorR_PWM)+250);
}


/**
 * @function: int ABS(int a)
 * @description: 绝对值函数
 * @param {int} a 数据 :输入的值
 * @return {int} a :取绝对值输出
 */
int ABS(int a)
{
	a = a>0?a:(-a);
	return a;
}
