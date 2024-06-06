#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "POSE_CALC.h"

uint8_t ID,i;								//定义用于存放ID号的变量
int16_t AX, AY, AZ, GX, GY, GZ;			//定义用于存放各个数据的变量
extern imu660_data imu;    //陀螺仪数据存储


int main(void)
{
    /*模块初始化*/
	OLED_Init();		//OLED初始化 
	MPU6050_Init();		//MPU6050初始化
	IIR_imu();
	/*显示ID号*/
	OLED_ShowString(1, 1, "ID:");		//显示静态字符串
	ID = MPU6050_GetID();				//获取MPU6050的ID号
	OLED_ShowHexNum(1, 4, ID, 2);		//OLED显示ID号
	
    
	while (1)
	{
//		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);		//获取MPU6050的数据
        
        IMU();
//		OLED_ShowSignedNum(2, 1, AX, 5);					//OLED显示数据
//		OLED_ShowSignedNum(3, 1, AY, 5);
//		OLED_ShowSignedNum(4, 1, AZ, 5);
//		OLED_ShowSignedNum(2, 8, GX, 5);
//		OLED_ShowSignedNum(3, 8, GY, 5);
//		OLED_ShowSignedNum(4, 8, GZ, 5);
        
        
        
		OLED_ShowSignedNum(2, 1, imu.pitch, 5);					//OLED显示数据
		OLED_ShowSignedNum(3, 1, imu.roll, 5);
		OLED_ShowSignedNum(4, 1, imu.yaw, 5);
	}
}
