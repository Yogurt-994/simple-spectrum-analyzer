/*
 * algorithm.c
 *
 *  Created on: Jun 24, 2022
 *      Author: 王志涵
 */

#include "algorithm.h"
#include "base.h"
#include "math.h"
#include "stdbool.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include "ADS8688.h"
#include "AD9959.h"
#include "cmd_process.h"
#include "hmi_user_uart.h"
#include "main.h"
#include "tim.h"


Sys sys;		//系统结构体
Sys sysError;   //故障测试结构体

const u8 *CiucuitErrors[15]={
"电路正常","R1开路","R1短路","R2开路","R2短路",
"R3开路","R3短路","R4开路","R4短路","C1开路",
"C1加倍","C2开路","C2加倍","C3开路","C3加倍"
};

/**
 * ADS8688六通道AD采样
 */
ArrayParam AD_Params;
float AD_array[6][AD_Size]={0,},	//AD采样二维数组
		*pAD_array,*pAD_array_end;

u8
TFT_array[AD_Size]={0,},
*pTFT_array,*pTFT_array_end
;

#define adResIn1 AD_array[0]	//输入采样点1
#define adResIn2 AD_array[1]	//输入采样点2
#define adResLoad AD_array[2]	//输出有载
#define adResNoLoad AD_array[3]	//输出无载交流
#define adResRMS AD_array[4]	//输出有效值
#define adResDC AD_array[5]		//输出直流

/**
 * @brief AD采样二维数组初始化
 */
void AD_arrInit(void){
	pAD_array=AD_array[0];
	pAD_array_end=AD_array[0]+AD_Size;
	arm_fill_f32(0,AD_array[0],AD_Size);
	arm_fill_f32(0,AD_array[1],AD_Size);
	arm_fill_f32(0,AD_array[2],AD_Size);
	arm_fill_f32(0,AD_array[3],AD_Size);
	arm_fill_f32(0,AD_array[4],AD_Size);
	arm_fill_f32(0,AD_array[5],AD_Size);
}


/**
 * @brief ADS采值，采用定时器中断
 */
void getADResults(void)
{
	AD_arrInit();
    get_ADS_allch(pAD_array);
    HAL_TIM_Base_Start_IT(&htim3);
    while(pAD_array!=pAD_array_end){};
    HAL_TIM_Base_Stop_IT(&htim3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim3)
    {
        if(pAD_array < pAD_array_end)
        {
            get_ADS_allch(pAD_array++);
        }
    }
}

/**
 * @brief 参数初始化
 */
void Params_Init(void)
{
	AD_arrInit();
    sys.mode = BaseTest;
    sys.AmpFreqFlag =0;	//是否进行幅频特性特性
    sys.inputRes = 0;   //输入电阻
    sys.outputRes= 0;   //输出电阻
    sys.gain     = 0;   //电路增益

    sys.upFreq   = 0;   //上限频率
    sys.dB       = 3;   //对应分贝点

    sys.bugElec  = 0;   //故障元件

    sys.Rs       = 6550.340f;//输入电阻分压电阻
    sys.Ro       = 997.870f;//输出电阻分压电阻
    sys.DC       = 0;
    sys.AC       = 0;

    sys.dis      = 0;
    sys.Auto     = 0;
    sys.err      = 0;
    sys.R1R2Short = 0;
    sys.R3R4Open = 0;
    sys.flag = 0 ;
    relayNoLoad;         //输出低电平空载
}

/**
 * @brief 傅里叶变换
 */
#define FFT_SIZE AD_Size
static float FFT_Buffer[FFT_SIZE];
float fftResult[6][AD_Size]={0,},
inputVol[2]={0,},		//输入两处电位
outputVol[2]={0,},		//输出两处电位
sweepFreArr[21],		//扫频
ampValue[3],			//在当前情况下，输出三种频率下的幅值,1kHz,100Hz,100kHz
goodAmp[3]={360,95,1080};			//电路正常情况下，三种频率下输出的波形RMS
u32 outFreq[3]={1000,100,100000};	//输出频率
void calc_FFT(float*Input,float*Output)
{
    arm_rfft_fast_instance_f32 S;//结构体
    arm_rfft_fast_init_f32(&S,FFT_SIZE);//初始化该结构体
    arm_rfft_fast_f32(&S, Input, FFT_Buffer, 0);//ifft_flag=0是正变换， 为1则是逆变换
    arm_cmplx_mag_f32(FFT_Buffer, Output, FFT_SIZE);
    arm_scale_f32(Output,2.0f/FFT_SIZE,Output,FFT_SIZE);    //换算成真实V
    Output[0] *= 0.5f;
}

float32_t Phase_f32[FFT_SIZE]; /* 相位*/
float fftPhase[6][FFT_SIZE]={0,},
phaseTwo[2],		  //输入输出相位
skewing,			  //相位差
judgeC1,
RmsForC1,
judgeC1Rms;
/*
*	函 数 名: PowerPhaseRadians_f32
*	功能说明: 求相位
*	形    参：_ptr  复位地址，含实部和虚部
*             _phase 求出相位，单位角度制，范围(-180, 180]
*             _usFFTPoints  复数个数，每个复数是两个float32_t数值
*             _uiCmpValue  比较值，需要求出相位的数值
*	返 回 值: 无
*/
void PowerPhaseRadians_f32(float32_t *_ptr, float32_t *_phase, uint16_t _usFFTPoints, float32_t _uiCmpValue)
{
	float32_t lX, lY;
	uint16_t i;
	float32_t phase;
	float32_t mag;

	for (i=0; i <_usFFTPoints; i++)
	{
		lX= _ptr[2*i];  	  /* 实部 */
		lY= _ptr[2*i + 1];    /* 虚部 */

 		phase = atan2f(lY, lX);    		  				 /* atan2求解的结果范围是(-pi, pi], 弧度制 */
		arm_sqrt_f32((float32_t)(lX*lX+ lY*lY), &mag);   /* 求模 */

//		if(_uiCmpValue < mag)
//		{
//			_phase[i] = 0;
//		}
//		else
//		{
			_phase[i] = phase* 180.0f/3.1415926f;   /* 将求解的结果由弧度转换为角度 */
//		}
	}
}

void cal_fftPhase(float*Input,float*Output){
	arm_rfft_fast_instance_f32 S;
	arm_rfft_fast_init_f32(&S,FFT_SIZE);	//初始化该结构体
	arm_rfft_fast_f32(&S,Input,FFT_Buffer,0);//ifft_flag=0是正变换， 为1则是逆变换
	PowerPhaseRadians_f32(FFT_Buffer,Output,FFT_SIZE,500);
}


/**
 * @brief 判断波形是否失真
 */
#define WAVE_THERSHOLD 0.10f
bool isGoodWave(void){
    float sum=0,noise=0,harmonic;u16 i=0;
    getADResults();				//采集电压
    calc_FFT(adResNoLoad,fftResult[3]);	//对输出无载情况下，采集的电压做傅里叶变换
    if(fftResult[3][50]<1)		//一次谐波幅值
	{
    	sinfrq.amp=500;
    	ad9959_write_amplitude(AD9959_CHANNEL_0, 500);
		  return true;
	}
    for(i=1;i<128;i++)
        sum += fftResult[3][i];

    harmonic = fftResult[3][50*2]+fftResult[3][50*3]+fftResult[3][50*4];//谐波
    noise = (sum - fftResult[3][50*1]- harmonic)/128;	//噪声

    if(fftResult[3][50]==noise) noise--;
    	sys.dis = (harmonic-noise) / (fftResult[3][50]-noise);
    SetTFTText(0,10,"%.3f", sys.dis * 100.0f);
    if( sys.dis <= WAVE_THERSHOLD)
        return true;
    else return false;
}

//准备阶段:频率初始化,并为电路提供最合适的幅值
void PrepareForTest(void)
{
    u16 PreCnt=0;		//准备计数值
    sinfrq.freq=1000;
    sinfrq.amp=500;
    ad9959_write_frequency(AD9959_CHANNEL_0, sinfrq.freq);
    ad9959_write_amplitude(AD9959_CHANNEL_0, sinfrq.amp);
    delay_ms(100);
    while(!isGoodWave())		//判断幅值是否合适
    {
    	sinfrq.amp -= 50;
    	ad9959_write_amplitude(AD9959_CHANNEL_0, sinfrq.amp);
        delay_ms(100);
        if(++PreCnt >= 8)
        {
            return;
        }
    }
}


//基础部分测试：测试电路各部分参数
void CalCircuitParam(Sys *param){

	/* */
	getADResults();			//ADS8688六路通道采样

	calc_FFT(adResIn1,fftResult[0]);	//输入采样点1FFT
	calc_FFT(adResIn2,fftResult[1]);	//输入采样点2FFT
	inputVol[0]=fftResult[0][50];		//输入测试点1电位
	inputVol[1] = fftResult[1][50];		//输入测试点2电位

	calc_FFT(adResNoLoad,fftResult[3]);		//输出空载FFT
	outputVol[1] = fftResult[3][50];			//输出空载处电位

	/* 有载测试 */
	relayLoad;
	delay_ms(200);

	getADResults();
	calc_FFT(adResLoad,fftResult[2]);
	outputVol[0]=fftResult[2][50];				//输出有载处电位

	/* 根据测试值计算电路参数 */
	param->inputRes = sys.Rs*inputVol[1]/(inputVol[0]-inputVol[1]);		//输入阻抗
	param->outputRes = sys.Ro*(outputVol[1]/outputVol[0]-1);			//输出阻抗
	param->gain = outputVol[1]/inputVol[1];		//电路增益

    /* 数据异常处理 */
    if(param->inputRes < -1000)
    	ad9959_init();
    if(param->outputRes <= 0)
        param->outputRes = 1927.0f;

	/* 下次空载测试 */
	relayNoLoad;		//再次空载
	delay_ms(200);

}

//找到上限截止频率
float dB2times[21]={1,0.891250938,0.794328235,0.707945784,0.630957344,0.562341325,0.501187234,0.446683592,0.398107171,0.354813389,0.316227766,0.281838293,0.251188643,0.223872114,0.199526231,0.177827941,0.158489319,0.141253754,0.125892541,0.112201845,0.1};
float calcUpFreq(float*array)
{
    u16 i;
    float value = dB2times[sys.dB],freq=0;
    for(i=sweepfreq.time-1;i>0;i--)
    {
        if(array[i-1]>value && array[i]<value)
        {
            freq = i*sweepfreq.step;
            break;
        }
    }
    if(i)
    freq = freq - sweepfreq.step*(value-array[i])/(array[i-1]-array[i]);
    return freq;
}

//扫频测试
void SweepTest(void)
{
    u16 i;float result;u32 useless;
    AD_arrInit();
    sweepfreq.start=0;Out_freq(0,sweepfreq.start);delay_ms(200);
    sweepfreq.step=10000;
    sweepfreq.end=200000;
    sweepfreq.time=(sweepfreq.end/sweepfreq.step)+1;
    for(i=0;i<sweepfreq.time;i++)  // 100Hz -> 200K,21点,步进10 KHz
    {
    	ad9959_write_frequency(AD9959_CHANNEL_0, sweepfreq.start + i*sweepfreq.step);
        delay_ms(100);
        get_ADS_allch(pAD_array++);
    }
    sinfrq.freq = 1000;
    ad9959_write_frequency(AD9959_CHANNEL_0,sinfrq.freq);
    /* 归一化处理 */
    arm_max_f32(adResRMS,i,&result,&useless);
    if(result==0)result=1;    //防止采样出错
    arm_scale_f32(adResRMS, 1.0f/result, adResRMS,i);
    SetTFTText(0,36,"%.1fkHz",calcUpFreq(adResRMS)*0.001f);
    pTFT_array = TFT_array;pAD_array = adResRMS;
    arm_scale_f32(adResRMS, 255.0f, adResRMS,i);
    for(i=0;i<sweepfreq.time;i++)
        *pTFT_array++ = *pAD_array++;
    GraphChannelDataAdd(0,38,0,TFT_array,i);
}

//扩展扫频测试
void ExSweepTest(void)
{
    u16 i;float result;u32 useless;
    AD_arrInit();
    sweepfreq.start=0;Out_freq(0,sweepfreq.start);delay_ms(200);
    sweepfreq.step=2000;
    sweepfreq.end=200000;
    sweepfreq.time=(sweepfreq.end/sweepfreq.step)+1;
    for(i=0;i<sweepfreq.time;i++)  // 100Hz -> 200K,101点,步进1 KHz
    {
    	ad9959_write_frequency(AD9959_CHANNEL_0, sweepfreq.start + i*sweepfreq.step);
        delay_ms(20);
        get_ADS_allch(pAD_array++);
    }
    sinfrq.freq = 1000;
    ad9959_write_frequency(AD9959_CHANNEL_0,sinfrq.freq);
    /* 归一化处理 */
    arm_max_f32(adResRMS,i,&result,&useless);
    if(result==0)result=1;    //防止采样出错
    arm_scale_f32(adResRMS, 1.0f/result, adResRMS,i);
    pTFT_array = TFT_array;pAD_array = adResRMS;
    arm_scale_f32(adResRMS, 255.0f, adResRMS,i);
    for(i=0;i<sweepfreq.time;i++)
        *pTFT_array++ = *pAD_array++;
    GraphChannelDataAdd(1,3,0,TFT_array,i);
}


//判断一个是否在此区间
bool isRound(float newNum,float min,float max)
{
    if(  (newNum>min)
       &&(newNum<max)
      )
    return true;
    else return false;
}

//发挥部分电路故障显示
void CircuitFaultShow(u16 error)
{
    if(error)
        SetTFTText(0,22,(char*)"故障");
    else
        SetTFTText(0,22,(char*)"正常");
    SetTFTText(0,23,(char*)(u8*)CiucuitErrors[error]);
//    if(error == 10) delay_ms(1000);
    delay_ms(20);
}

//故障测试扩展UI界面更新
void ExCircuitFaultShow(u16 error)
{
//    SetControlVisiable(2,(error+7)/2,1);

    SetTFTText(2,3,(char*)(u8*)CiucuitErrors[error]);

//    SetControlVisiable(2,(sys.err+7)/2,0);
    delay_ms(20);

//    sys.err = error;

}


//故障部分测试
u16 CalCircuitError(void){

	CalResIn_AC(&sysError);	//计算输入电阻

	if(isRound(sysError.inputRes,13500,15000)){
		delay_ms(300);
		return R1ErrorOpen;				//R1开路
	}

	else if(sysError.inputRes>20000){
		delay_ms(300);
		return C1ErrorOpen;		//C1开路
	}


	getDC(&sysError);		//获取直流电压
	Out_mV(0,500);
	delay_ms(200);

	if(isRound(sysError.inputRes,9000,12000)){
		if(isRound(sysError.DC,3800,4000)){
			return R4ErrorOpen;			//R4开路
		}
		else if(isRound(sysError.AC,10,20)){
			return C2ErrorOpen;			//C2开路
		}
	}

	else if(sysError.inputRes<200){
		if(isRound(sysError.DC,3500,3800)){
			return R1ErrorShort;		//R1短路
		}
		else if(isRound(sysError.DC,1200,1600)){
			return R2ErrorOpen;			//R2开路
		}
		else if(isRound(sysError.DC,3800,4000)){
			return R2ErrorShort;		//R2短路
		}
		else if(isRound(sysError.DC,190,300)){
			return R3ErrorOpen;			//R3开路
		}
		else if(isRound(sysError.DC,100,180)){
			return R4ErrorShort;		//R4短路
		}
	}


	else if(isRound(sysError.inputRes,2000,4000)){
		if(isRound(sysError.DC,3800,4000)){
			return R3ErrorShort;		//R3短路
		}

		CheckAmp(2);
		if(ampValue[1] / sys.amp2 > 1.6f){
			return C2ErrorTwice;		//C2加倍
		}
		else if(ampValue[2] / sys.amp3 > 1.12f){
			return C3ErrorOpen;			//C3开路
		}
		else if(ampValue[2] / sys.amp3 < 0.85f){
			return C3ErrorTwice;		//C3加倍
		}

	}

//	CheckAmp(0);
//	if(judgeC1 > 2.0f){
//		return C1ErrorTwice;		//C1加倍
//	}

	return NoError;
}

//获取直流电压
void getDC(Sys* param)
{
    Out_mV(0,0);delay_ms(200);
    pAD_array=AD_array[0];
    get_ADS_allch(pAD_array);
    param->DC = adResDC[0];
}

//计算电路输入电阻
void CalResIn_AC(Sys *param){

	getADResults();

	calc_FFT(adResIn1,fftResult[0]);	//输入采样点1FFT
	calc_FFT(adResIn2,fftResult[1]);	//输入采样点2FFT
	calc_FFT(adResNoLoad,fftResult[3]);	//输出交流FFT
	inputVol[0]=fftResult[0][50];		//输入测试点1电位
	inputVol[1]=fftResult[1][50];		//输入测试点2电位

	//计算电路参数
	param->inputRes = sys.Rs*inputVol[1]/(inputVol[0]-inputVol[1]);	//输入阻抗
	param->AC = fftResult[3][50];		//交流峰值

    if(param->inputRes < -1000)
    {
    	ad9959_init();
    }
}


//检查点频幅度,返回1则一切正常
u16 CheckAmp(u16 part)
{
    /* 测量三个点频的值 */
    pAD_array=AD_array[0];

    get_ADS_allch(pAD_array);
    ampValue[0] = adResRMS[0];
    if(ampValue[0]<0)ampValue[0]=1;

    if(sys.mode==BaseTest)sys.amp1=ampValue[0];

    if(part==0)
    {
    	relay2Load;
    	ad9959_write_frequency(AD9959_CHANNEL_0, 20);
    	ad9959_write_amplitude(AD9959_CHANNEL_0, 500);
        delay_ms(200);
        getADResults();
        cal_fftPhase(adResIn1,fftPhase[0]);
        cal_fftPhase(adResNoLoad,fftPhase[3]);
        phaseTwo[0] = fftPhase[0][1];
        phaseTwo[1] = fftPhase[3][1];
        if(sys.mode==BaseTest){
        	sys.skewing=phaseTwo[0]-phaseTwo[1];
        	if(sys.skewing < 0) sys.skewing += 360;
        	sys.RmsForC1 = adResRMS[0];
        }
        skewing =phaseTwo[0]-phaseTwo[1];
        if(skewing < 0) skewing += 360;
        RmsForC1 = adResRMS[0];

        judgeC1 = skewing - sys.skewing;
        judgeC1Rms = RmsForC1 - sys.RmsForC1;

        relay2NoLoad;
    }
    else if(part==1)
    {
        Out_freq(0,outFreq[2]);		//输出100kHz
        delay_ms(300);
        get_ADS_allch(pAD_array);
        ampValue[2] = adResRMS[0];
    }
    else
    {
    	/* 输出100kHz */
    	relay2Load;
    	ad9959_write_frequency(AD9959_CHANNEL_0, 100000);
    	ad9959_write_amplitude(AD9959_CHANNEL_0, 500);
        pAD_array=AD_array[0];
        delay_ms(300);
        get_ADS_allch(pAD_array);
        ampValue[2] = adResRMS[0];
        if(sys.mode==BaseTest)sys.amp3=ampValue[2];
        relay2NoLoad;

        /* 输出50Hz */
        ad9959_write_frequency(AD9959_CHANNEL_0, 50);
        ad9959_write_amplitude(AD9959_CHANNEL_0, 500);
        pAD_array=AD_array[0];
        delay_ms(800);
        get_ADS_allch(pAD_array);
        ampValue[1] = adResRMS[0];
        if(sys.mode==BaseTest)sys.amp2=ampValue[1];

    }

    sinfrq.freq = 1000;
    ad9959_write_frequency(AD9959_CHANNEL_0, 1000);

    if(isRound(ampValue[1],goodAmp[1]*0.96f,goodAmp[1]*1.04f)
     &&isRound(ampValue[2],goodAmp[2]*0.96f,goodAmp[2]*1.04f))
        return 1;
    else return 0;
}

