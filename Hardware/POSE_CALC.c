#include "stm32f10x.h"  
#include "math.h"
#include "MPU6050.h" 
#include "POSE_CALC.h" 

#define kp      0.5f                        //proportional gain governs rate of convergence to accelerometer/magnetometer
#define ki      0.0001f                     //integral gain governs rate of convergenceof gyroscope biases

int16_t mpu6050_gyro_x,mpu6050_gyro_y,mpu6050_gyro_z;
int16_t mpu6050_acc_x,mpu6050_acc_y,mpu6050_acc_z;

_Matrix Mat = {0};
_Attitude att = {0};
imu660_offset set = {0};
imu660_data imu = {0} ;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //quaternion elements representing theestimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;      //scaled integral error
int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;                           // 三轴陀螺仪滤波数据      gyro (陀螺仪)
int16_t acc_x = 0, acc_y = 0, acc_z = 0;                              // 三轴加速度计滤波数据     acc (accelerometer 加速度计)

/*
 * 函数名：get_iir_factor
 * 描述  ：求取IIR滤波器的滤波因子
 * 输入  ：out_factor滤波因子首地址，Time任务执行周期，Cut_Off滤波截止频率
 * 返回  ：
 */
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
    *out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}

/* 获取IIR低通滤波 */
void IIR_imu(void)
{
	int8_t i;
    for(i=0;i<=100;i++)
    {
        MPU6050_get_gyro(&mpu6050_gyro_x,&mpu6050_gyro_y,&mpu6050_gyro_z);
        set.gyro.x+=mpu6050_gyro_x;
        set.gyro.y+=mpu6050_gyro_y;
        set.gyro.z+=mpu6050_gyro_z;
    }
    set.gyro.x/= 100;
    set.gyro.y/= 100;
    set.gyro.z/= 100;
    set.offset_flag = 1;
    //printf("%d\n",set.offset_flag);
    get_iir_factor(&imu.att_acc_factor,imu_Read_Time,15);
    get_iir_factor(&imu.att_gyro_factor,imu_Read_Time,10);
}
/**
  * @brief   IIR低通滤波器
  * @param   *acc_in 输入三轴数据指针变量
  * @param   *acc_out 输出三轴数据指针变量
  * @param   lpf_factor 滤波因数
  * @retval  x
  */
float iir_lpf(float in,float out,float lpf_factor)
{
    out = out + lpf_factor * (in - out);
    return out;
}
// 快速开根号算法
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/*
 * 函数名：mahony_update
 * 描述  ：姿态解算
 * 输入  ：陀螺仪三轴数据（单位：弧度/秒），加速度三轴数据（单位：g）
 * 返回  ：
 */
//Gyroscope units are radians/second, accelerometer  units are irrelevant as the vector is normalised.
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    
    static float pitch_old=0, yaw_old=0;
    float temp = 0;
 
    if(ax*ay*az==0)
        return;
    gx=gx*(PI / 180.0f);
    gy=gy*(PI / 180.0f);
    gz=gz*(PI / 180.0f);
    //[ax,ay,az]是机体坐标系下加速度计测得的重力向量(竖直向下)
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
 
    vx = Mat.DCM_T[0][2];
    vy = Mat.DCM_T[1][2];
    vz = Mat.DCM_T[2][2];
 
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;
 
    //对误差向量进行积分
    exInt = exInt + ex*ki;
    eyInt = eyInt + ey*ki;
    ezInt = ezInt + ez*ki;
 
    //姿态误差补偿到角速度上，修正角速度积分漂移，通过调节Kp、Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度。
    gx = gx + kp*ex + exInt;
    gy = gy + kp*ey + eyInt;
    gz = gz + kp*ez + ezInt;
 
    //一阶龙格库塔法更新四元数
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)* MahonyPERIOD * 0.0005f;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)* MahonyPERIOD * 0.0005f;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)* MahonyPERIOD * 0.0005f;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)* MahonyPERIOD * 0.0005f;
 
    //把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
    //printf("%f,%f,%f,%f\n",q0,q1,q2,q3);
 
   
    //四元素转欧拉角
    imu.pitch =   atan2f(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * (180.0f / PI);
    imu.roll  =   -asinf(2.0f*(q0*q2 - q1*q3)) * (180.0f / PI);
    
    //z轴角速度积分的偏航角
    imu.yaw += imu.deg_s.z  * MahonyPERIOD * 0.001f;
 
    //防温漂
    temp = imu.pitch-pitch_old;
    if(temp<2 && temp > -2)
    {
        imu.pitch = pitch_old;
    }
    else
    {
        pitch_old = imu.pitch;
    }
 
    temp = imu.yaw-yaw_old;
    if(temp<0.1f && temp > -0.1f)
    {
        imu.yaw = yaw_old;
    }
    else
    {
        yaw_old = imu.yaw;
    }
} 
/*
 * 函数名：rotation_matrix
 * 描述  ：旋转矩阵：机体坐标系 -> 地理坐标系
 * 输入  ：
 * 返回  ：
 */
void rotation_matrix(void)
{
    Mat.DCM[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
    Mat.DCM[0][1] = 2.0f * (q1*q2 -q0*q3);
    Mat.DCM[0][2] = 2.0f * (q1*q3 +q0*q2);
 
    Mat.DCM[1][0] = 2.0f * (q1*q2 +q0*q3);
    Mat.DCM[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
    Mat.DCM[1][2] = 2.0f * (q2*q3 -q0*q1);
 
    Mat.DCM[2][0] = 2.0f * (q1*q3 -q0*q2);
    Mat.DCM[2][1] = 2.0f * (q2*q3 +q0*q1);
    Mat.DCM[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
}
/*
 * 函数名：rotation_matrix_T
 * 描述  ：旋转矩阵的转置矩阵：地理坐标系 -> 机体坐标系
 * 输入  ：
 * 返回  ：
 */
void rotation_matrix_T(void)
{
    Mat.DCM_T[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
    Mat.DCM_T[0][1] = 2.0f * (q1*q2 +q0*q3);
    Mat.DCM_T[0][2] = 2.0f * (q1*q3 -q0*q2);
 
    Mat.DCM_T[1][0] = 2.0f * (q1*q2 -q0*q3);
    Mat.DCM_T[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
    Mat.DCM_T[1][2] = 2.0f * (q2*q3 +q0*q1);
 
    Mat.DCM_T[2][0] = 2.0f * (q1*q3 +q0*q2);
    Mat.DCM_T[2][1] = 2.0f * (q2*q3 -q0*q1);
    Mat.DCM_T[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
}
/*
 * 函数名：Matrix_ready
 * 描述  ：矩阵更新准备，为姿态解算使用
 * 输入  ：
 * 返回  ：
 */
void Matrix_ready(void)
{
    rotation_matrix();                      //旋转矩阵更新
    rotation_matrix_T();                    //旋转矩阵的逆矩阵更新
}

/***********************************************************
函数名称：void IMU(void)
函数功能：获得姿态结算后的值
入口参数：无
出口参数：无
备 注：直接读取imu.pitch  imu.roll  imu.yaw
***********************************************************/
void IMU(void)
{
    if(set.offset_flag)
    {
        /*获取X、Y的角速度和加速度*/
        MPU6050_get_accdata(&mpu6050_acc_x,&mpu6050_acc_y,&mpu6050_acc_z);
        MPU6050_get_gyro(&mpu6050_gyro_x,&mpu6050_gyro_y,&mpu6050_gyro_z);
        
        /*滤波算法*/
        mpu6050_gyro_x-=set.gyro.x;
        mpu6050_gyro_y-=set.gyro.y;
        mpu6050_gyro_z-=set.gyro.z;
 
        acc_x = iir_lpf(mpu6050_acc_x,acc_x,imu.att_acc_factor);
        acc_y = iir_lpf(mpu6050_acc_y,acc_y,imu.att_acc_factor);
        acc_z = iir_lpf(mpu6050_acc_z,acc_z,imu.att_acc_factor);
        //printf("%.2f,%.2f,%.2f\n",acc_x,acc_y,acc_z);
        gyro_x =iir_lpf(mpu6050_gyro_x,gyro_x,imu.att_gyro_factor);
        gyro_y =iir_lpf(mpu6050_gyro_y,gyro_y,imu.att_gyro_factor);
        gyro_z =iir_lpf(mpu6050_gyro_z,gyro_z,imu.att_gyro_factor);
        //printf("%d,%d,%d\n",gyro_x,gyro_y,gyro_z);
 
        /*数据存储*/
        imu.acc_g.x = (float)acc_x/4096; //加速度计量程为:±8g/4096, ±16g/2048, ±4g/8192, ±2g/16384 
        imu.acc_g.y = (float)acc_y/4096;
        imu.acc_g.z = (float)acc_z/4096;
        imu.deg_s.x = (float)mpu6050_gyro_x/16.4f;//陀螺仪量程为:±2000dps/16.4, ±1000dps/32.8, ±500 dps /65.6
        imu.deg_s.y = (float)mpu6050_gyro_y/16.4f;//±250 dps/131.2, ±125 dps/262.4
        imu.deg_s.z = (float)mpu6050_gyro_z/16.4f;
 
        /*姿态解算*/
        mahony_update(imu.deg_s.x,imu.deg_s.y,imu.deg_s.z,imu.acc_g.x,imu.acc_g.y,imu.acc_g.z);
        Matrix_ready();
    }
}
