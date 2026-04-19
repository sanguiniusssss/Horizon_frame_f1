#include "Vision.h"
#include "DJI_Motor.h"
#include "All_init.h"

int8_t Vision_Rx_Data(uint8_t* buffer, VisionRxDataUnion *VisionRx)
{
    VisionTemp Union_temp;
    VisionRx->Data.OffCounter = 1;
    uint8_t i = 0;
    //获取头帧
    VisionRx->Data.Head_frame = buffer[i++];
    if (VisionRx->Data.Head_frame != 0xCD)//帧头为0xCD
    {
        return -1;
    }
    //获取Pitch角度
    Union_temp.Data[0] = buffer[i++];// 这里的i++会在使用后自增，所以先使用再加1
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[2] = buffer[i++];
    Union_temp.Data[3] = buffer[i++];
    VisionRx->Data.PitchAngle = Union_temp.Data_f;// 将4个字节的数据转换为float类型的PitchAngle
    //获取Yaw角度
    Union_temp.Data[0] = buffer[i++];
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[2] = buffer[i++];
    Union_temp.Data[3] = buffer[i++];
    VisionRx->Data.YawAngle = Union_temp.Data_f;
    //获取Pitch 前馈值
    Union_temp.Data[0] = buffer[i++];
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[2] = buffer[i++];
    Union_temp.Data[3] = buffer[i++];
    VisionRx->Data.PitchOmega = Union_temp.Data_f;
    //获取Yaw 前馈值
    Union_temp.Data[0] = buffer[i++];
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[2] = buffer[i++];
    Union_temp.Data[3] = buffer[i++];
    VisionRx->Data.YawOmega = Union_temp.Data_f;

    //获取VisionTime
    Union_temp.Data[0] = buffer[i++];
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[2] = buffer[i++];
    Union_temp.Data[3] = buffer[i++];
    VisionRx->Data.VisionTime = Union_temp.Data_f;

    VisionRx->Data.End_frame = buffer[i];

    //  VisionRxData.PitchAngle_kal =0;//kalmanFilter(&kfp_visionPitch,VisionRxData.PitchAngle);
    //  VisionRxData.YawAngle_kal =0;//kalmanFilter(&kfp_visionYaw,VisionRxData.YawAngle);

    if (VisionRx->Data.End_frame != 0xDC)
    {
        return -2;
    }
    // HAL_UART_Receive_DMA(&huart2, (uint8_t *)VisionRx->OriginData, sizeof(VisionRx->OriginData));
    return 0;
}

void Vision_Tx_Data(MOTOR_Typedef *motor)
{
    VisionTemp Union_temp;// 定义一个VisionTemp类型的变量，用于将float类型的数据转换为4个字节的uint8_t数组
    static VisionTxDataUnion VisionTxData;// 定义一个VisionTxDataUnion类型的变量，用于存储要发送的视觉数据
    uint8_t i = 0;

    VisionTxData.Head_frame = 0xCD;
    VisionTxData.PitchAngle = motor->M6020[PITCH].DATA.Angle_Infinite/8192.0f*360.0f;
    VisionTxData.YawAngle = motor->M6020[YAW].DATA.Angle_Infinite/8192.0f*360.0f;
    VisionTxData.PitchOmega = motor->M6020[PITCH].DATA.Speed_now/60.0f*360.0f;
    VisionTxData.YawOmega = motor->M6020[YAW].DATA.Speed_now/60.0f*360.0f;
    VisionTxData.VisionTime = ALL_CONTAL.DWT_TIME.Gimbal_time; // ms
    VisionTxData.End_frame = 0xDC;


    VisionTxData.data[i++] = VisionTxData.Head_frame;
    Union_temp.Data_f = VisionTxData.PitchAngle;
    VisionTxData.data[i++] = Union_temp.Data[0];
    VisionTxData.data[i++] = Union_temp.Data[1];
    VisionTxData.data[i++] = Union_temp.Data[2];
    VisionTxData.data[i++] = Union_temp.Data[3];

    Union_temp.Data_f = VisionTxData.YawAngle;
    VisionTxData.data[i++] = Union_temp.Data[0];
    VisionTxData.data[i++] = Union_temp.Data[1];
    VisionTxData.data[i++] = Union_temp.Data[2];
    VisionTxData.data[i++] = Union_temp.Data[3];

    Union_temp.Data_f = VisionTxData.PitchOmega;
    VisionTxData.data[i++] = Union_temp.Data[0];
    VisionTxData.data[i++] = Union_temp.Data[1];
    VisionTxData.data[i++] = Union_temp.Data[2];
    VisionTxData.data[i++] = Union_temp.Data[3];

    Union_temp.Data_f = VisionTxData.YawOmega;
    VisionTxData.data[i++] = Union_temp.Data[0];
    VisionTxData.data[i++] = Union_temp.Data[1];
    VisionTxData.data[i++] = Union_temp.Data[2];
    VisionTxData.data[i++] = Union_temp.Data[3];

    Union_temp.Data_f = VisionTxData.VisionTime;
    VisionTxData.data[i++] = Union_temp.Data[0];
    VisionTxData.data[i++] = Union_temp.Data[1];
    VisionTxData.data[i++] = Union_temp.Data[2];
    VisionTxData.data[i++] = Union_temp.Data[3];

    VisionTxData.data[i++] = VisionTxData.End_frame;
    HAL_UART_Transmit_DMA(&huart2, VisionTxData.data, sizeof(VisionTxData.data));
}

void Vision_Monitor(VisionRxDataUnion *VisionRx)// 监视视觉系统状态，主要用于在线检测,如果连续 1000 ms（1秒） 未收到新视觉数据，就认为视觉模块离线
{
    if (VisionRx->Data.OffCounter < 1000)
    {
        VisionRx->Data.OffCounter++;
    }
    VisionRx->Data.isOnline = (VisionRx->Data.OffCounter < 1000) ? 1 : 0;
}


// 下面是原视觉方案，可用于视觉自瞄通信测试
#ifdef USE_VISION
int8_t Vision_Rx_Data(uint8_t* buffer, VisionRxDataUnion *VisionRx)
{
    VisionTemp Union_temp;
    VisionRx->Data.OffCounter = 1;
    uint8_t i = 0;
    //获取头帧
    VisionRx->Data.Head_frame = buffer[i++];
    if (VisionRx->Data.Head_frame != 0xCD)
    {
        return -1;
    }
    //获取Pitch角度
    Union_temp.Data[0] = buffer[i++];
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[2] = buffer[i++];
    Union_temp.Data[3] = buffer[i++];
    VisionRx->Data.PitchAngle = Union_temp.Data_f;
    //获取Yaw角度
    Union_temp.Data[0] = buffer[i++];
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[2] = buffer[i++];
    Union_temp.Data[3] = buffer[i++];
    VisionRx->Data.YawAngle = Union_temp.Data_f;

    VisionRx->Data.VisionState = buffer[i] & 0x07;
    VisionRx->Data.ShootBool = (buffer[i] & 0x08)>>3;
    VisionRx->Data.Target = (buffer[i] & 0x10)>>4;
    i++;
    //if(VisionRxData.PitchAngle>=0)VisionRxData.Target=0;
    //获取VisionTime
    Union_temp.Data[0] = buffer[i++];
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[2] = buffer[i++];
    Union_temp.Data[3] = buffer[i++];
    VisionRx->Data.VisionTime = Union_temp.Data_u32;

    VisionRx->Data.End_frame = buffer[i];


    //  VisionRxData.PitchAngle_kal =0;//kalmanFilter(&kfp_visionPitch,VisionRxData.PitchAngle);
    //  VisionRxData.YawAngle_kal =0;//kalmanFilter(&kfp_visionYaw,VisionRxData.YawAngle);


    if (VisionRx->Data.End_frame != 0xDC)
    {
        return -2;
    }
    // HAL_UART_Receive_DMA(&huart2, (uint8_t *)VisionRx->OriginData, sizeof(VisionRx->OriginData));
    return 0;
}

void Vision_Tx_Data(float PitchAngle, float YawAngle, uint32_t Time, uint8_t State, uint8_t Rate_of_fire)
{
    VisionTemp Union_temp;
    static VisionTxDataUnion VisionTxData;
    uint8_t i = 0;

    VisionTxData.Head_frame = 0xCD;
    VisionTxData.PitchAngle = PitchAngle;
    VisionTxData.YawAngle = YawAngle;
    VisionTxData.VisionTime = Time;
    VisionTxData.VisionState = State;
    VisionTxData.Rate_of_fire = Rate_of_fire;
    VisionTxData.End_frame = 0xDC;


    VisionTxData.data[i++] = VisionTxData.Head_frame;
    Union_temp.Data_f = VisionTxData.PitchAngle;
    VisionTxData.data[i++] = Union_temp.Data[0];
    VisionTxData.data[i++] = Union_temp.Data[1];
    VisionTxData.data[i++] = Union_temp.Data[2];
    VisionTxData.data[i++] = Union_temp.Data[3];

    Union_temp.Data_f = VisionTxData.YawAngle;
    VisionTxData.data[i++] = Union_temp.Data[0];
    VisionTxData.data[i++] = Union_temp.Data[1];
    VisionTxData.data[i++] = Union_temp.Data[2];
    VisionTxData.data[i++] = Union_temp.Data[3];

    VisionTxData.data[i++] = VisionTxData.VisionState;

    Union_temp.Data_u32 = VisionTxData.VisionTime;
    VisionTxData.data[i++] = Union_temp.Data[0];
    VisionTxData.data[i++] = Union_temp.Data[1];
    VisionTxData.data[i++] = Union_temp.Data[2];
    VisionTxData.data[i++] = Union_temp.Data[3];

    VisionTxData.data[i++] = 0;

    VisionTxData.data[i++] = VisionTxData.End_frame;
    HAL_UART_Transmit_DMA(&huart2, VisionTxData.data, 16);
}
#endif