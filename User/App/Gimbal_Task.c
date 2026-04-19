#include "Gimbal_Task.h"
#include "controller.h"
#include "All_init.h"
#include "MY_Define.h"

uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typedef *motor)
{
    float PID_S_Pitch[3] = {75.0f, 0.0f, 0.0f};             //在此处修改pich轴pid参数，三个参数分别为kp,ki,kd
    float PID_P_Pitch[3] = {0.85f, 0.001f, 0.0f};           //在此处修改pich轴pid参数，三个参数分别为kp,ki,kd

    PID_Init(&motor->M6020[PITCH].PID_S, 
              20000.0f,                                     //输出限幅
              0.0f,                                         //死区
              PID_S_Pitch,                                  //pid参数
              1000.0f,                                      //积分限幅
              1000.0f,                                      //误差限幅
              0.7f, 0.7f, 2,                                //滤波器参数
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    PID_Init(&motor->M6020[PITCH].PID_P, 10000.0f, 50.0f, 
              PID_P_Pitch, 2000.0f, 1000.0f, 
              0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    
    float PID_S_Yaw[3] = {50.0f, 0.0f, 0.0f};               //在此处修改yaw轴pid参数，三个参数分别为kp,ki,kd
    float PID_P_Yaw[3] = {2.0f, 0.0008f, 0.001f};           //在此处修改yaw轴pid参数，三个参数分别为kp,ki,kd

    PID_Init(&motor->M6020[YAW].PID_S, 15000.0f, 0.0f, 
              PID_S_Yaw, 1000.0f, 1000.0f, 
              0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    PID_Init(&motor->M6020[YAW].PID_P, 10000.0f, 50.0f, 
              PID_P_Yaw, 1000.0f, 1000.0f, 
              0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);


#if USE_FUZZY_PID
    /* 定义四个模糊规则实例（静态局部变量，生命周期为整个程序） */
    static FuzzyRule_t FuzzyRule_Pitch_Pos;   // Pitch位置环
    static FuzzyRule_t FuzzyRule_Pitch_Spd;   // Pitch速度环
    static FuzzyRule_t FuzzyRule_Yaw_Pos;     // Yaw位置环
    static FuzzyRule_t FuzzyRule_Yaw_Spd;     // Yaw速度环

    /* 初始化模糊规则表（参数需根据实际调试调整） */
    // Pitch 位置环：角度误差较大时论域步长可设大一些
    Fuzzy_Rule_Init(&FuzzyRule_Pitch_Pos,
                    NULL, NULL, NULL,       // 使用默认规则表
                    0.8f, 0.3f, 0.1f,       // KpRatio, KiRatio, KdRatio
                    400.0f, 200.0f);        // eStep (编码值), ecStep

    // Pitch 速度环：速度误差变化快，步长适当调整
    Fuzzy_Rule_Init(&FuzzyRule_Pitch_Spd,
                    NULL, NULL, NULL,
                    1.0f, 0.2f, 0.05f,
                    50.0f, 30.0f);          // eStep (rpm), ecStep

    // Yaw 位置环
    Fuzzy_Rule_Init(&FuzzyRule_Yaw_Pos,
                    NULL, NULL, NULL,
                    1.0f, 0.4f, 0.15f,
                    500.0f, 250.0f);

    // Yaw 速度环
    Fuzzy_Rule_Init(&FuzzyRule_Yaw_Spd,
                    NULL, NULL, NULL,
                    1.2f, 0.3f, 0.1f,
                    60.0f, 35.0f);



                     /* 将模糊规则绑定到对应的 PID 结构体 */
    motor->M6020[PITCH].PID_S.FuzzyRule = &FuzzyRule_Pitch_Spd;
    motor->M6020[PITCH].PID_P.FuzzyRule = &FuzzyRule_Pitch_Pos;
    motor->M6020[YAW].PID_S.FuzzyRule   = &FuzzyRule_Yaw_Spd;
    motor->M6020[YAW].PID_P.FuzzyRule   = &FuzzyRule_Yaw_Pos;

    /* 可选：让模糊规则使用与 PID 相同的 DWT 计数器（保持时间同步） */
    // 这里不做额外处理，Fuzzy_Rule_Implementation 内部会使用自己的 DWT_CNT
#endif


    return 0;
}

uint8_t Gimbal_AIM_INIT()
{

    return 0;
}

uint8_t gimbal_task(CONTAL_Typedef *CONTAL, MOTOR_Typedef *MOTOR)
{
    MOTOR->M6020[PITCH].DATA.Aim = CONTAL->HEAD.Pitch;
    MOTOR->M6020[YAW].DATA.Aim   = CONTAL->HEAD.Yaw;

    PID_Calculate(&MOTOR->M6020[PITCH].PID_P, //pich轴位置环计算
                   MOTOR->M6020[PITCH].DATA.Angle_Infinite,//测量值
                   MOTOR->M6020[PITCH].DATA.Aim);//期望值
    PID_Calculate(&MOTOR->M6020[PITCH].PID_S, 
                   MOTOR->M6020[PITCH].DATA.Speed_now,
                   MOTOR->M6020[PITCH].PID_P.Output);//pich轴位置环输出作为速度环的期望值

    PID_Calculate(&MOTOR->M6020[YAW].PID_P,//yaw轴位置环计算
                   MOTOR->M6020[YAW].DATA.Angle_Infinite,
                   MOTOR->M6020[YAW].DATA.Aim);
    PID_Calculate(&MOTOR->M6020[YAW].PID_S,
                   MOTOR->M6020[YAW].DATA.Speed_now,
                   MOTOR->M6020[YAW].PID_P.Output);//yaw轴位置环输出作为速度环的期望值
    
    DJI_Current_Ctrl(&hcan, 0x1ff,//发送CAN帧控制云台电机，0x1ff为ID号，后三个参数分别为四个电机的电流值，单位为mA
                     (int16_t)MOTOR->M6020[YAW].PID_S.Output,
                     (int16_t)MOTOR->M6020[PITCH].PID_S.Output,
                     0,
                     0);
    //  DJI_Current_Ctrl(&hcan, 0x1ff, 
    //                  (int16_t)MOTOR->M6020[YAW].PID_S.Output,
    //                  0,
    //                  0,
    //                  0);
    return 0;
}



