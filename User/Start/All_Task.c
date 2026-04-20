#include "All_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include "VOFA.h"
#include "All_init.h"
#include "can_bsp.h"
#include "Robot.h"
#include "bsp_dwt.h"

void StartDefaultTask(void)
{
    All_Init();
    for(;;)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
        osDelay(500);
    }
}

void StartGimbalTask(void)
{
    for(;;)
    {
        ALL_CONTAL.DWT_TIME.Gimbal_dt = DWT_GetDeltaT(&ALL_CONTAL.DWT_TIME.Gimbal_Count);
        ALL_CONTAL.DWT_TIME.Gimbal_time = DWT_GetTimeline_ms();
        // VOFA_justfloat((float)DBUS_V_DATA.Remote.CH0_int16, 
        //                 (float)ALL_MOTOR.M6020[YAW].DATA.Angle_Infinite,
        //                 (float)ALL_MOTOR.M6020[PITCH].DATA.Angle_Infinite,
        //                 (float)ALL_MOTOR.M6020[PITCH].DATA.Speed_now/60.0f*360.0f,
        //                 (float)ALL_MOTOR.M6020[YAW].DATA.Speed_now/60.0f*360.0f,
        //                 ALL_CONTAL.DWT_TIME.Gimbal_dt,
        //                 ALL_CONTAL.DWT_TIME.Monitor_dt,
        //                 (float) VisionRxData.Data.isOnline,
        //                 0,
        //                 ALL_CONTAL.DWT_TIME.Gimbal_time);
        VOFA_justfloat((float)DBUS_V_DATA.Remote.CH0_int16, // 通道0的值
                        (float)ALL_MOTOR.M6020[YAW].DATA.Angle_Infinite / 8192.0f * 360.0f,// yaw电机的角度值，单位为度
                        (float)ALL_MOTOR.M6020[PITCH].DATA.Angle_Infinite / 8192.0f * 360.0f,// pitch电机的角度值，单位为度
                        (float)ALL_MOTOR.M6020[PITCH].DATA.Speed_now/60.0f*360.0f,// pitch电机的速度值，单位为度每秒
                        (float)ALL_MOTOR.M6020[YAW].DATA.Speed_now/60.0f*360.0f,// yaw电机的速度值，单位为度每秒
                        (float)VisionRxData.Data.PitchAngle,// 视觉系统测得的云台俯仰角，单位为度，目标
                        (float)VisionRxData.Data.YawAngle,// 视觉系统测得的云台偏航角，单位为度，目标
                        ALL_CONTAL.DWT_TIME.Monitor_dt,// 监视控制周期，单位为秒
                        (float) VisionRxData.Data.isOnline,// 视觉系统在线状态，1为在线，0为离线
                        0);
        Vision_Tx_Data(&ALL_MOTOR);// 发送视觉数据到上位机
        gimbal_task(&ALL_CONTAL, &ALL_MOTOR);// 云台控制任务
        RobotTask(3, &DBUS_V_DATA, &ALL_CONTAL);  // 3 自瞄 2 遥控
        osDelay(1);// 延时1ms，控制任务频率为1000Hz
    }
}

void StartMonitorTask(void)// 监视任务，主要用于在线检测和数据监视
{
    //
    for(;;)
    {
        ALL_CONTAL.DWT_TIME.Monitor_dt = DWT_GetDeltaT(&ALL_CONTAL.DWT_TIME.Monitor_Count);// 监视控制周期，单位为秒
        RobotTask(3, &DBUS_V_DATA, &ALL_CONTAL);  // 3 自瞄 2 遥控
        DBUS_OFFLINE_Check(&DBUS_V_DATA);           // can发送在这里，应该写到task里的，懒了
        Vision_Monitor(&VisionRxData);// 监视视觉系统状态
        osDelay(1);
    }
}
