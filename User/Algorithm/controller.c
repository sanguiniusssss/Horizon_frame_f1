/*
 * ......................................&&.........................
 * ....................................&&&..........................
 * .................................&&&&............................
 * ...............................&&&&..............................
 * .............................&&&&&&..............................
 * ...........................&&&&&&....&&&..&&&&&&&&&&&&&&&........
 * ..................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&..............
 * ................&...&&&&&&&&&&&&&&&&&&&&&&&&&&&&.................
 * .......................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&.........
 * ...................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...............
 * ..................&&&   &&&&&&&&&&&&&&&&&&&&&&&&&&&&&............
 * ...............&&&&&@  &&&&&&&&&&..&&&&&&&&&&&&&&&&&&&...........
 * ..............&&&&&&&&&&&&&&&.&&....&&&&&&&&&&&&&..&&&&&.........
 * ..........&&&&&&&&&&&&&&&&&&...&.....&&&&&&&&&&&&&...&&&&........
 * ........&&&&&&&&&&&&&&&&&&&.........&&&&&&&&&&&&&&&....&&&.......
 * .......&&&&&&&&.....................&&&&&&&&&&&&&&&&.....&&......
 * ........&&&&&.....................&&&&&&&&&&&&&&&&&&.............
 * ..........&...................&&&&&&&&&&&&&&&&&&&&&&&............
 * ................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&............
 * ..................&&&&&&&&&&&&&&&&&&&&&&&&&&&&..&&&&&............
 * ..............&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&....&&&&&............
 * ...........&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&......&&&&............
 * .........&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&.........&&&&............
 * .......&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...........&&&&............
 * ......&&&&&&&&&&&&&&&&&&&...&&&&&&...............&&&.............
 * .....&&&&&&&&&&&&&&&&............................&&..............
 * ....&&&&&&&&&&&&&&&.................&&...........................
 * ...&&&&&&&&&&&&&&&.....................&&&&......................
 * ...&&&&&&&&&&.&&&........................&&&&&...................
 * ..&&&&&&&&&&&..&&..........................&&&&&&&...............
 * ..&&&&&&&&&&&&...&............&&&.....&&&&...&&&&&&&.............
 * ..&&&&&&&&&&&&&.................&&&.....&&&&&&&&&&&&&&...........
 * ..&&&&&&&&&&&&&&&&..............&&&&&&&&&&&&&&&&&&&&&&&&.........
 * ..&&.&&&&&&&&&&&&&&&&&.........&&&&&&&&&&&&&&&&&&&&&&&&&&&.......
 * ...&&..&&&&&&&&&&&&.........&&&&&&&&&&&&&&&&...&&&&&&&&&&&&......
 * ....&..&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...........&&&&&&&&.....
 * .......&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&..............&&&&&&&....
 * .......&&&&&.&&&&&&&&&&&&&&&&&&..&&&&&&&&...&..........&&&&&&....
 * ........&&&.....&&&&&&&&&&&&&.....&&&&&&&&&&...........&..&&&&...
 * .......&&&........&&&.&&&&&&&&&.....&&&&&.................&&&&...
 * .......&&&...............&&&&&&&.......&&&&&&&&............&&&...
 * ........&&...................&&&&&&.........................&&&..
 * .........&.....................&&&&........................&&....
 * ...............................&&&.......................&&......
 * ................................&&......................&&.......
 * .................................&&..............................
 * ..................................&..............................
 */

/**
 ******************************************************************************
 * @file    controller.c
 * @author  Wang Hongxi
 * @author  Zhang Hongyu (fuzzy pid)
 * @version V1.2.0 (Modified for USE_FUZZY_PID switch)
 * @date    2021/7/3
 * @brief   DWT定时器用于计算控制周期 OLS用于提取信号微分
 ******************************************************************************
 */

#include "controller.h"
#include "MY_Define.h"      // 包含 USE_FUZZY_PID 宏定义
#include "bsp_dwt.h"         // 包含 DWT_GetDeltaT 等

/* 如果未定义 USE_FUZZY_PID，则默认禁用 */
#ifndef USE_FUZZY_PID
#define USE_FUZZY_PID   0
#endif

/******************************** FUZZY PID **********************************/
#if USE_FUZZY_PID

static float FuzzyRuleKpRAW[7][7] = {
    {PB, PB, PM, PM, PS, ZE, ZE},
    {PB, PB, PM, PS, PS, ZE, PS},
    {PM, PM, PM, PS, ZE, PS, PS},
    {PM, PM, PS, ZE, PS, PM, PM},
    {PS, PS, ZE, PS, PS, PM, PM},
    {PS, ZE, PS, PM, PM, PM, PB},
    {ZE, ZE, PM, PM, PM, PB, PB}
};

static float FuzzyRuleKiRAW[7][7] = {
    {PB, PB, PM, PM, PS, ZE, ZE},
    {PB, PB, PM, PS, PS, ZE, ZE},
    {PB, PM, PM, PS, ZE, PS, PS},
    {PM, PM, PS, ZE, PS, PM, PM},
    {PS, PS, ZE, PS, PS, PM, PB},
    {ZE, ZE, PS, PS, PM, PB, PB},
    {ZE, ZE, PS, PM, PM, PB, PB}
};

static float FuzzyRuleKdRAW[7][7] = {
    {PS, PS, PB, PB, PB, PM, PS},
    {PS, PS, PB, PM, PM, PS, ZE},
    {ZE, PS, PM, PM, PS, PS, ZE},
    {ZE, PS, PS, PS, PS, PS, ZE},
    {ZE, ZE, ZE, ZE, ZE, ZE, ZE},
    {PB, PS, PS, PS, PS, PS, PB},
    {PB, PM, PM, PM, PS, PS, PB}
};

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep)
{
    if (fuzzyRuleKp == NULL)
        fuzzyRule->FuzzyRuleKp = FuzzyRuleKpRAW;
    else
        fuzzyRule->FuzzyRuleKp = fuzzyRuleKp;
    if (fuzzyRuleKi == NULL)
        fuzzyRule->FuzzyRuleKi = FuzzyRuleKiRAW;
    else
        fuzzyRule->FuzzyRuleKi = fuzzyRuleKi;
    if (fuzzyRuleKd == NULL)
        fuzzyRule->FuzzyRuleKd = FuzzyRuleKdRAW;
    else
        fuzzyRule->FuzzyRuleKd = fuzzyRuleKd;

    fuzzyRule->KpRatio = kpRatio;
    fuzzyRule->KiRatio = kiRatio;
    fuzzyRule->KdRatio = kdRatio;

    if (eStep < 0.00001f) eStep = 1.0f;
    if (ecStep < 0.00001f) ecStep = 1.0f;
    fuzzyRule->eStep = eStep;
    fuzzyRule->ecStep = ecStep;

    fuzzyRule->DWT_CNT = 0;
    fuzzyRule->eLast = 0.0f;
}

void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref)
{
    float eLeftTemp, ecLeftTemp;
    float eRightTemp, ecRightTemp;
    int eLeftIndex, ecLeftIndex;
    int eRightIndex, ecRightIndex;

    fuzzyRule->dt = DWT_GetDeltaT(&fuzzyRule->DWT_CNT);

    fuzzyRule->e = ref - measure;
    fuzzyRule->ec = (fuzzyRule->e - fuzzyRule->eLast) / fuzzyRule->dt;
    fuzzyRule->eLast = fuzzyRule->e;

    // 隶属区间
    eLeftIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 2)));
    eRightIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 4) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3)));
    ecLeftIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 2)));
    ecRightIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 4) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3)));

    // 隶属度
    eLeftTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 1 : (eRightIndex - fuzzyRule->e / fuzzyRule->eStep - 3));
    eRightTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 1 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e / fuzzyRule->eStep - eLeftIndex + 3));
    ecLeftTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 1 : (ecRightIndex - fuzzyRule->ec / fuzzyRule->ecStep - 3));
    ecRightTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 1 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec / fuzzyRule->ecStep - ecLeftIndex + 3));

    fuzzyRule->KpFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecLeftIndex] +
                         eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecLeftIndex] +
                         eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecRightIndex] +
                         eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecRightIndex];

    fuzzyRule->KiFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecLeftIndex] +
                         eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecLeftIndex] +
                         eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecRightIndex] +
                         eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecRightIndex];

    fuzzyRule->KdFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecLeftIndex] +
                         eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecLeftIndex] +
                         eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecRightIndex] +
                         eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecRightIndex];
}

#endif /* USE_FUZZY_PID */

/******************************* PID CONTROL *********************************/
// PID优化环节函数声明
static void f_Trapezoid_Intergral(PID_t *pid);
static void f_Integral_Limit(PID_t *pid);
static void f_Derivative_On_Measurement(PID_t *pid);
static void f_Changing_Integration_Rate(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Derivative_Filter(PID_t *pid);
static void f_Output_Limit(PID_t *pid);
static void f_Proportion_Limit(PID_t *pid);
static void f_PID_ErrorHandle(PID_t *pid);

/**
 * @brief          PID初始化   PID initialize
 * @param[in]      pid               PID结构体指针
 * @param[in]      max_out           输出限幅
 * @param[in]      intergral_limit   积分限幅
 * @param[in]      kpid              初始Kp,Ki,Kd数组
 * @param[in]      A                 变速积分参数A
 * @param[in]      B                 变速积分参数B
 * @param[in]      output_lpf_rc     输出滤波时间常数
 * @param[in]      derivative_lpf_rc 微分滤波时间常数
 * @param[in]      ols_order         OLS阶数(预留)
 * @param[in]      improve           优化功能位掩码
 * @retval         无
 */
void PID_Init(
    PID_t *pid,
    float max_out,
    float intergral_limit,
    float kpid[3],
    float A,
    float B,
    float output_lpf_rc,
    float derivative_lpf_rc,
    uint16_t ols_order,
    uint8_t improve)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->Ref = 0.0f;

    pid->Kp = kpid[0];
    pid->Ki = kpid[1];
    pid->Kd = kpid[2];
    pid->ITerm = 0.0f;

    pid->CoefA = A;
    pid->CoefB = B;

    pid->Output_LPF_RC = output_lpf_rc;
    pid->Derivative_LPF_RC = derivative_lpf_rc;

    pid->DWT_CNT = 0;

    pid->Improve = improve;

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0.0f;

    /* 若未启用模糊PID，确保FuzzyRule指针为NULL */
#if !USE_FUZZY_PID
    pid->FuzzyRule = NULL;
#endif
}

void PID_set(PID_t *pid, float kpid[3])
{
    pid->Kp = kpid[0];
    pid->Ki = kpid[1];
    pid->Kd = kpid[2];
}

/**
 * @brief          PID计算
 * @param[*pid]    PID结构体
 * @param[measure] 测量值
 * @param[ref]     期望值
 * @retval         控制输出
 */
float PID_Calculate(PID_t *pid, float measure, float ref)
{
    if (pid->Improve & ErrorHandle)
        f_PID_ErrorHandle(pid);

    /* 获取真实控制周期时间（秒） */
    uint32_t tmp = pid->DWT_CNT;
    pid->dt = DWT_GetDeltaT(&tmp);
    pid->DWT_CNT = tmp;

    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (pid->User_Func1_f != NULL)
        pid->User_Func1_f(pid);

#if USE_FUZZY_PID
    if (pid->FuzzyRule != NULL)
    {
        Fuzzy_Rule_Implementation(pid->FuzzyRule, measure, ref);
        pid->Pout = (pid->Kp + pid->FuzzyRule->KpFuzzy) * pid->Err;
        pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * pid->Err * pid->dt;
        pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Err - pid->Last_Err) / pid->dt;
    }
    else
#endif
    {
        /* 固定PID计算 */
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
    }

    if (pid->User_Func2_f != NULL)
        pid->User_Func2_f(pid);

    // 梯形积分
    if (pid->Improve & Trapezoid_Intergral)
        f_Trapezoid_Intergral(pid);
    // 变速积分
    if (pid->Improve & ChangingIntegrationRate)
        f_Changing_Integration_Rate(pid);
    // 微分先行
    if (pid->Improve & Derivative_On_Measurement)
        f_Derivative_On_Measurement(pid);
    // 微分滤波器
    if (pid->Improve & DerivativeFilter)
        f_Derivative_Filter(pid);
    // 积分限幅
    if (pid->Improve & Integral_Limit)
        f_Integral_Limit(pid);

    pid->Iout += pid->ITerm;
    pid->Output = pid->Pout + pid->Iout + pid->Dout;

    // 输出滤波
    if (pid->Improve & OutputFilter)
        f_Output_Filter(pid);

    // 输出限幅
    f_Output_Limit(pid);
    f_Proportion_Limit(pid);

    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;
    return pid->Output;
}

static void f_Trapezoid_Intergral(PID_t *pid)
{
#if USE_FUZZY_PID
    if (pid->FuzzyRule == NULL)
        pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
    else
        pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
#else
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
#endif
}

static void f_Changing_Integration_Rate(PID_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        if (fabsf(pid->Err) <= pid->CoefB)
            return;
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_t *pid)
{
    float temp_Iout = pid->Iout + pid->ITerm;
    float temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (fabsf(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    else if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_t *pid)
{
#if USE_FUZZY_PID
    if (pid->FuzzyRule == NULL)
        pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    else
        pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Last_Measure - pid->Measure) / pid->dt;
#else
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
#endif
}

static void f_Derivative_Filter(PID_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

static void f_Output_Filter(PID_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->MaxOut)
        pid->Output = pid->MaxOut;
    else if (pid->Output < -pid->MaxOut)
        pid->Output = -pid->MaxOut;
}

static void f_Proportion_Limit(PID_t *pid)
{
    if (pid->Pout > pid->MaxOut)
        pid->Pout = pid->MaxOut;
    else if (pid->Pout < -pid->MaxOut)
        pid->Pout = -pid->MaxOut;
}

static void f_PID_ErrorHandle(PID_t *pid)
{
    if (pid->Output < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
        pid->ERRORHandler.ERRORCount++;
    else
        pid->ERRORHandler.ERRORCount = 0;

    if (pid->ERRORHandler.ERRORCount > 500)
        pid->ERRORHandler.ERRORType = Motor_Blocked;
}