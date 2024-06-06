#ifndef  __POSE_CALC_H__
#define  __POSE_CALC_H__

#define PI              3.1415926f
#define imu_Read_Time   0.01f//原始数据采集时间间隔 秒为单位 0.01秒 10ms
#define MahonyPERIOD    10.0f//姿态解算周期（ms）


//extern _Matrix Mat;
//extern _Attitude att;

//extern imu660_offset set;
//extern imu660_data imu;    //陀螺仪数据存储

//=============================================姿态解算================
typedef struct
{
    signed short x;
    signed short y;
    signed short z;
}S16_XYZ;                     //有符号16位数(xyz)

typedef struct
{
    float x;
    float y;
    float z;
}SI_F_XYZ;//无符号浮点数


typedef struct
{
    SI_F_XYZ deg_s;      //度每秒
    SI_F_XYZ acc_g;      //加速度

    float att_acc_factor;
    float att_gyro_factor;
    float roll;
    float pitch;
    float yaw;
}imu660_data;


typedef struct
{
    SI_F_XYZ gyro;      //角速度
    int8_t offset_flag;
}imu660_offset;


typedef struct
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float pit;
    float rol;
    float yaw;
}_Attitude;


typedef struct
{
    float DCM[3][3];        //机体坐标系 -> 地理坐标系
    float DCM_T[3][3];      //地理坐标系 -> 机体坐标系
}_Matrix;

void  IIR_imu(void);                                         //获得IIR低通滤波参数
void  IMU(void);                                         //解算


#endif
