/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 16:02:33
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-05 15:45:32
 */
#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#include "MY_define.h"
#include "can_bsp.h"
#include "controller.h"

typedef struct
{
    int8_t ONLINE_JUDGE_TIME;
    int16_t Angle_last; // 上一个角度值
    int16_t Angle_now;  // 现在的角度值
    int16_t Speed_last; // 上一个速度值
    int16_t Speed_now;  // 现在的速度值
    int16_t current;// 电流值
    int8_t temperature;// 温度值
    int32_t Angle_Infinite;// 无限旋转角度值
    int64_t Stuck_Time;// 卡死时间
    uint16_t Stuck_Flag[2];// 卡死标志位，0x0001为正转卡死，0x0002为反转卡死
    int16_t Laps;// 圈数
    float Error;// 误差值
    float Aim;// 目标值
    float Aim_last;// 上一个目标值
    float dt;// 时间间隔
}DJI_MOTOR_DATA_Typedef;

typedef struct
{
    uint8_t PID_INIT;// PID初始化标志位
    DJI_MOTOR_DATA_Typedef DATA;// 电机数据
    PID_t PID_P;//位置环
    PID_t PID_S;//速度环
}DJI_MOTOR_Typedef;

typedef struct 
{
    DJI_MOTOR_Typedef M3508[4];
    DJI_MOTOR_Typedef M6020[2];
}MOTOR_Typedef;

typedef struct
{
    struct 
    {
        float Pitch;// 目标角度
        float Pitch_MAX;// 目标角度上限
        float Pitch_MIN;// 目标角度下限
        float Yaw;// 目标角度
        float Yaw_Init;//目标角度初始值
    }HEAD;//云台头部数据
    struct 
    {
        float Chassis_dt;// 底盘控制周期
        float Gimbal_dt;// 云台控制周期
        float Monitor_dt;// 监视控制周期
        float Shoot_dt;// 射击控制周期
        float Gimbal_time;// 云台控制时间戳
        uint32_t Chassis_Count;// 底盘控制计数
        uint32_t Gimbal_Count;// 云台控制计数
        uint32_t Monitor_Count;// 监视控制计数
        uint32_t Shoot_Count;// 射击控制计数
    }DWT_TIME;
}CONTAL_Typedef;//控制相关数据结构体

void DJI_Current_Ctrl(hcan_t* hcan, uint16_t stdid, int16_t num1, int16_t num2, int16_t num3, int16_t num4);
void MotorResolve(DJI_MOTOR_Typedef *motor, uint8_t *RxMessage);
void MotorRoundResolve(DJI_MOTOR_Typedef *motor);

#endif
