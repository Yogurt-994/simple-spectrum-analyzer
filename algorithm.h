/*
 * algorithm.h
 *
 *  Created on: Jun 24, 2022
 *      Author: 王志涵
 */

#ifndef INC_ALGORITHM_H_
#define INC_ALGORITHM_H_

#include "base.h"
#include "stdbool.h"
#include "arm_math.h"

#define AD_Size 256

#define BaseTest   1		//基础部分测试
#define UpTest     2		//发挥部分测试
#define ExAmpFreq  3		//扩展幅频特性曲线测试
#define ExElecTest 4		//扩展故障部分测试

/* 电路故障原因 */
#define NoError         0
#define R1ErrorOpen     1
#define R1ErrorShort    2
#define R2ErrorOpen     3
#define R2ErrorShort    4
#define R3ErrorOpen     5
#define R3ErrorShort    6
#define R4ErrorOpen     7
#define R4ErrorShort    8
#define C1ErrorOpen     9
#define C1ErrorTwice    10
#define C2ErrorOpen     11
#define C2ErrorTwice    12
#define C3ErrorOpen     13
#define C3ErrorTwice    14



typedef struct _Sys
{
    short mode;         //测量模式

    u16 AmpFreqFlag;    //是否幅频特性测试
    float inputRes;     //输入电阻
    float outputRes;    //输出电阻
    float gain;         //电路增益
    float beta;         //贝塔值

    float upFreq;       //上限频率
    u16 dB;           //对应分贝点

    short bugElec;      //故障元件
    u8*   bugResult;    //故障原因

    float Rs;           //输入电阻分压电阻
    float Ro;           //输出电阻分压电阻
    float DC;           //直流电平
    float AC;			//交流峰峰值
    float dis;          //失真度
    u16   Auto;         //切换自动挡与手动挡
    float InputRes15;
    float R1R2Short;
    float R3R4Open;
    float skewing;		//相位差
    float RmsForC1;
    float amp1;			//1kHz
    float amp2;			//10Hz
    float amp3;			//100kHz
    u16 flag;
    u16 err;
}Sys;
extern Sys sys;

typedef struct _ArrayParam   //数组参数结构体
{
    float max;      //最大值
    float min;      //最小值
    float tft_len;  //波形一个周期占多少像素
    float tft_cycle;
    float Vpp;   //峰峰值 mV
    float Period;//周期   us
    float Aver;  //平均值 mv
    float Rms;   //有效值 mv
}ArrayParam;
extern ArrayParam AD_Params;

void AD_arrInit(void);
void getADResults(void);
void Params_Init(void);
void calc_FFT(float*Input,float*Output);
void PowerPhaseRadians_f32(float32_t *_ptr, float32_t *_phase, uint16_t _usFFTPoints, float32_t _uiCmpValue);

bool isGoodWave(void);
void PrepareForTest(void);

void CalCircuitParam(Sys *param);

void SweepTest(void);
void ExSweepTest(void);

bool isRound(float newNum,float min,float max);
void CircuitFaultShow(u16 error);
void ExCircuitFaultShow(u16 error);
u16 CalCircuitError(void);
void getDC(Sys* param);
void CalResIn_AC(Sys *param);

u16 CheckAmp(u16 part);

#endif /* INC_ALGORITHM_H_ */
