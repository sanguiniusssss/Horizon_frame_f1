/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-18 16:44:18
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-18 16:48:46
 */
#include "can_bsp.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //CAN ID过滤
 *	@parameter:		    //
 *	@time:				//22-11-23 20:39
 *	@ReadMe:			//放在主函数里初始化一次
 ************************************************************万能分隔符**************************************************************/
void CAN_Filter_Init(void)
{
	CAN_FilterTypeDef CAN_FilterInitStrt;//CAN过滤器配置结构体
	
    CAN_FilterInitStrt.SlaveStartFilterBank = 14;//过滤器分配给哪个CAN，F1只有一个CAN，所以分配给CAN1
    CAN_FilterInitStrt.FilterBank           = 14;//过滤器编号，0-13为CAN1，14-27为CAN2，F1只有一个CAN，所以只能用0-13
    CAN_FilterInitStrt.FilterActivation     = ENABLE;//使能过滤器
    CAN_FilterInitStrt.FilterMode           = CAN_FILTERMODE_IDMASK;//过滤器模式，IDMASK模式可以设置过滤器ID和掩码，只有当CAN报文ID与过滤器ID按位与掩码的结果相等时，报文才会被接收
    CAN_FilterInitStrt.FilterScale          = CAN_FILTERSCALE_32BIT;//过滤器位宽，32位过滤器可以过滤32位ID，16位过滤器可以过滤16位ID，F1只有一个CAN，所以只能用32位过滤器
    CAN_FilterInitStrt.FilterIdHigh         = 0x0000;//过滤器ID高16位，F1只有一个CAN，所以只能用32位过滤器，所以过滤器ID高16位为0
    CAN_FilterInitStrt.FilterIdLow          = 0x0000;//过滤器ID低16位，F1只有一个CAN，所以只能用32位过滤器，所以过滤器ID低16位为0
    CAN_FilterInitStrt.FilterMaskIdHigh     = 0x0000;//过滤器掩码高16位，F1只有一个CAN，所以只能用32位过滤器，所以过滤器掩码高16位为0
    CAN_FilterInitStrt.FilterMaskIdLow      = 0x0000;//过滤器掩码低16位，F1只有一个CAN，所以只能用32位过滤器，所以过滤器掩码低16位为0，表示接收所有报文
    CAN_FilterInitStrt.FilterBank           = 0;//过滤器编号，0-13为CAN1，14-27为CAN2，F1只有一个CAN，所以只能用0-13，这里用0
    CAN_FilterInitStrt.FilterFIFOAssignment = CAN_RX_FIFO0;//过滤器分配给哪个FIFO，F1只有一个CAN，所以只能用FIFO0

    HAL_CAN_ConfigFilter(&hcan , &CAN_FilterInitStrt);//配置过滤器
    HAL_CAN_Start(&hcan);//启动CAN
    HAL_CAN_ActivateNotification(&hcan , CAN_IT_RX_FIFO0_MSG_PENDING);//使能CAN接收中断，F1只有一个CAN，所以只能用FIFO0的接收中断

    return;
}


void canx_send_data(CAN_HandleTypeDef* _hcan , uint16_t stdid , uint8_t* Data)
{
    CAN_TxHeaderTypeDef TXMessage;//CAN发送消息结构体

    uint32_t send_mail_box;//CAN发送邮箱，F1只有三个发送邮箱，所以用uint32_t类型，值为0-2

    TXMessage.DLC   = 0x08;//数据长度，单位为字节，这里发送8字节数据
    TXMessage.IDE   = CAN_ID_STD;//标准帧，F1只有一个CAN，所以只能用标准帧，扩展帧需要使用32位过滤器，F1只有一个CAN，所以只能用32位过滤器，所以只能用标准帧
    TXMessage.RTR   = CAN_RTR_DATA;//数据帧，F1只有一个CAN，所以只能用数据帧，远程帧需要使用32位过滤器，F1只有一个CAN，所以只能用32位过滤器，所以只能用数据帧
    TXMessage.StdId = stdid;

    HAL_CAN_AddTxMessage(_hcan , &TXMessage , Data , &send_mail_box);//发送CAN消息，F1只有一个CAN，所以只能用一个CAN句柄，发送消息结构体，数据数组，发送邮箱
}


void CAN_send(CAN_HandleTypeDef *_hcan, int16_t stdid, int16_t num1, int16_t num2, int16_t num3, int16_t num4)
{
    CAN_TxHeaderTypeDef tx;
    uint8_t Data[8];
    uint32_t mailbox = 0;
    tx.DLC = 0x08;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.StdId = stdid;
    tx.ExtId = 0x000;
    Data[0] = ((num1) >> 8);
    Data[1] = (num1);
    Data[2] = ((num2) >> 8);
    Data[3] = (num2);
    Data[4] = ((num3) >> 8);
    Data[5] = (num3);
    Data[6] = ((num4) >> 8);
    Data[7] = (num4);

    HAL_CAN_AddTxMessage(&hcan, &tx, Data, &mailbox);
}