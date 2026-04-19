#ifndef _VISION_H_
#define _VISION_H_

#include "main.h"
#include "usart.h"
#include "DJI_Motor.h"

typedef union
{
  uint8_t Data[4];
  float Data_f;
  uint32_t Data_u32;
}VisionTemp;


typedef struct
{
    uint8_t OriginData[21];
    struct Data
    {
      uint8_t Head_frame;// 帧头
      uint8_t End_frame;// 帧尾
      float PitchAngle;// 视觉系统测得的云台俯仰角，目标
      float YawAngle;// 视觉系统测得的云台偏航角，目标
      float PitchOmega;// 视觉系统测得的云台俯仰角速度
      float YawOmega;// 视觉系统测得的云台偏航角速度
      float VisionTime;// 视觉系统测得的时间戳
      uint16_t OffCounter; // 在线检测
      uint8_t  isOnline;// 在线状态
    } Data;
}VisionRxDataUnion;

typedef struct
{
  uint8_t data[22];

  uint8_t Head_frame;
  float PitchAngle;
  float YawAngle;
  float PitchOmega;
  float YawOmega;
  float VisionTime;
  uint8_t End_frame;
}VisionTxDataUnion;// 视觉数据发送结构体

int8_t Vision_Rx_Data(uint8_t* buffer, VisionRxDataUnion *VisionRx);
void Vision_Tx_Data(MOTOR_Typedef *motor);
void Vision_Monitor(VisionRxDataUnion *VisionRx);

#endif
