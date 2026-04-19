#include "All_Init.h"

//遥控相关变量
DBUS_UNION_Typdef DBUS_V_UNION = {0};
DBUS_Typedef DBUS_V_DATA = { 0 };

//电机
MOTOR_Typedef ALL_MOTOR;//电机数据

//视觉
VisionRxDataUnion VisionRxData = {0};//视觉接收数据

CONTAL_Typedef ALL_CONTAL = {0};//控制相关数据

void All_Init(void)
{   
    DWT_Init(72);
    CAN_Filter_Init();

    HAL_TIM_Base_Start_IT(&htim2);//定时器2中断，时间基准

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//遥控串口
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)DBUS_V_UNION.GetData, 19);//开启DMA接收，接收数据存放在DBUS_V_UNION.GetData中，长度为19字节

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//视觉串口
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)VisionRxData.OriginData, sizeof(VisionRxData.OriginData));//开启DMA接收，接收数据存放在VisionRxData.OriginData中，长度为21字节

    ALL_CONTAL.HEAD.Pitch_MAX =  800.0f;//云台俯仰角上限，单位为度，正数表示云台向下
    ALL_CONTAL.HEAD.Pitch_MIN = -800.0f;//云台俯仰角下限，单位为度，负数表示云台向上
    ALL_CONTAL.HEAD.Yaw_Init  = (float)ALL_MOTOR.M6020[YAW].DATA.Angle_Infinite;//云台偏航角初始值，单位为度，正数表示云台向右
    MOTOR_PID_Gimbal_INIT(&ALL_MOTOR);//电机PID参数初始化
}