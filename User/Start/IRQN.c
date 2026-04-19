#include "IRQN.h"
#include "MY_Define.h"
#include "All_init.h"
#include "Vision.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RXMessage;//接收消息结构体
    uint8_t Data[8];//接收数据数组
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RXMessage, Data);//获取接收消息

    if (hcan->Instance == CAN1) {
        switch (RXMessage.StdId)
        {
        case (0X205+YAW):// 云台电机ID，0x206为yaw轴
        {
            MotorResolve(&ALL_MOTOR.M6020[YAW], Data);// 解析数据
            MotorRoundResolve(&ALL_MOTOR.M6020[YAW]);// 处理数据
        }
        break;
        case (0x205+PITCH):// 云台电机ID，0x206为yaw轴
        {
            MotorResolve(&ALL_MOTOR.M6020[PITCH], Data);
            MotorRoundResolve(&ALL_MOTOR.M6020[PITCH]);
        }
        break;
        }
    } 
}

// 串口
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        
    } 
    if (huart->Instance == USART2) {
        
    }
    if (huart->Instance == USART3)
    {
        
    }
}

// 空闲中断处理函数  TODO 优化成直接调用
void User_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
        // {
        //     __HAL_UART_CLEAR_IDLEFLAG(&huart1); 
        //     HAL_UART_DMAStop(&huart1);          
        //     Vision_Rx_Data(VisionRxData.OriginData, &VisionRxData);
        //     HAL_UART_Receive_DMA(&huart1, (uint8_t *)VisionRxData.OriginData, sizeof(VisionRxData.OriginData));
        // }
    }
    if (huart->Instance == USART2)//视觉串口
    {
       if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart2); 
            HAL_UART_DMAStop(&huart2);          
            Vision_Rx_Data(VisionRxData.OriginData, &VisionRxData);
            HAL_UART_Receive_DMA(&huart2, (uint8_t *)VisionRxData.OriginData, sizeof(VisionRxData.OriginData));
        }
    }
    if (huart->Instance == USART3)
    {
        if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart3); 
            HAL_UART_DMAStop(&huart3);          

            // 解析数据
            DBUS_V_DATA.ONLINE_JUDGE_TIME = 0;
            RUI_F_DUBS_Resovled(&DBUS_V_UNION, &DBUS_V_DATA);

            HAL_UART_Receive_DMA(&huart3, (uint8_t *)DBUS_V_UNION.GetData, 19);
        }
    }
 
}