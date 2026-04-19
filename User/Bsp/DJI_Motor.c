#include "DJI_Motor.h"


void DJI_Current_Ctrl(hcan_t* hcan, uint16_t stdid, int16_t num1, int16_t num2, int16_t num3, int16_t num4)//发送CAN帧控制云台电机，stdid为ID号，后三个参数分别为四个电机的电流值，单位为mA
{
    uint8_t Data[8];

    Data[0] = ((num1) >> 8);
    Data[1] = (num1);
    Data[2] = ((num2) >> 8);
    Data[3] = (num2);
    Data[4] = ((num3) >> 8);
    Data[5] = (num3);
    Data[6] = ((num4) >> 8);
    Data[7] = (num4);

    canx_send_data(hcan, stdid, Data);
}


void MotorResolve(DJI_MOTOR_Typedef *motor, uint8_t *RxMessage)// 解析CAN帧数据，更新电机数据结构体
{
    motor->DATA.Angle_last = motor->DATA.Angle_now;
    motor->DATA.Angle_now  = (uint16_t)RxMessage[0] << 8 | (uint16_t)RxMessage[1];
    motor->DATA.Speed_last = motor->DATA.Speed_now;
    motor->DATA.Speed_now  = (uint16_t)RxMessage[2] << 8 | (uint16_t)RxMessage[3];
    motor->DATA.current = ((uint16_t)RxMessage[4] << 8 | (uint16_t)RxMessage[5]);// 电流值为16位有符号整数，单位为mA
    motor->DATA.temperature = RxMessage[6];// 温度值为8位有符号整数，单位为摄氏度
}

/**
 * @brief 根据电机编码器值计算运转圈数
 *
 * @param motor 电机结构体指针
 */
void MotorRoundResolve(DJI_MOTOR_Typedef *motor)
{
    if (motor->DATA.Angle_now - motor->DATA.Angle_last < -4096)
    {
        motor->DATA.Laps++;
    }
    if (motor->DATA.Angle_now - motor->DATA.Angle_last > 4096)
    {
        motor->DATA.Laps--;
    }
    motor->DATA.Angle_Infinite = motor->DATA.Laps * 8192 + motor->DATA.Angle_now;
}
/* 
GM6020 的编码器输出范围是 0 ~ 8191（对应 0° ~ 360°）。
当电机旋转超过一圈时，角度值会从 8191 跳变到 0（或反向从 0 跳变到 8191），
若直接使用原始角度做位置闭环，会导致云台在跨越 0° 时剧烈抖动

正常情况：两次采样间隔内，电机转动不会超过半圈（4096 编码值），因此角度差应在 -4096 ~ +4096 之间。
正向过零：若上一周期角度为 8000，当前角度为 100，差值 100 - 8000 = -7900，小于 -4096，判定为正向过零，圈数 Laps 加 1。
反向过零：若上一周期为 100，当前为 8000，差值 8000 - 100 = 7900，大于 4096，判定为反向过零，圈数 Laps 减 1。
连续角度计算：Angle_Infinite = Laps × 8192 + Angle_now，得到一个单调连续变化的角度值，适用于位置闭环。

注意：该算法要求控制周期足够快，确保两次采样间的实际转角不超过半圈。
本项目控制频率为 1 kHz，云台最大转速通常低于 300 rpm（即每秒 5 转，每毫秒 0.005 转 ≈ 41 编码值），完全满足条件
*/