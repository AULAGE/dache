#include "hcsr04.h"
#include "SysTick.h"

#define test_num 10

float CAT(float *date,u8 len,u8 tip);

void HC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(Trig_RCC|Echo_RCC,ENABLE);
	
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//发出信号要用输出模式
	GPIO_InitStructure.GPIO_Pin=Trig_PIN;
	GPIO_Init(Trig_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;//接收信号要用输入模式且有时钟计时较方便
	GPIO_InitStructure.GPIO_Pin=Echo_PIN;
	GPIO_Init(Echo_PORT,&GPIO_InitStructure);
}

void HC_Start(void)
{
	Trig=1;
	delay_us(15);//大于10us
	Trig=0;
}

float HC_GetDistance(void)
{
    uint16_t time=0;        //us  
    float Distance[test_num]={0};//cm
    uint8_t i;
 
    for(i=0;i<test_num;i++)
    {
        HC_Start();
     
        while(Echo==0);
        if(Echo)
        {
            TIM_Cmd(TIM3,ENABLE);
            while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==SET);
            if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==RESET)
            {
                TIM_Cmd(TIM3,DISABLE);
                time=TIM3->CNT;
                TIM_SetCounter(TIM3,0);
                Distance[i]=(34000*time/1000000)/2;//cm
            }
        }
        delay_ms(80);//测量周期60ms以上
    }
    //数据滤波，test_num次数据排序，舍弃最大最小值求平均值
     return  2.0*CAT(Distance,test_num,2);
}

// 想办法减少机械误差的函数
//第一个办法，去高去低取平均
//返回值为一组数字处理后的值
float CAT(float *date,u8 len,u8 tip)
{
	u8 i,j,flag=0;
	//      加速
	float temp,sum=0;
	for(i=0;i<len-1;i++)
	{
		flag=0;
		for(j=0;j<len-1-i;j++)
		{
			if(date[j]>date[j+i])
			{
				temp=date[j];
				date[j]=date[j+1];
				date[j+1]=temp;
				flag=1;
			}
		}
		if(flag==0)break;
	}//排序
	for(i=tip;i<len-tip;i++)
	{
		sum+=date[i];
	}
	return sum/(len-tip);
}

//第二个办法，卡尔曼滤波
//较慢，3cm在2s左右
//个人理解就是每次用上一次误差量求均量以减小这次误差
float kalman_gain;         // 卡尔曼增益
float estimate;            // 状态估计值
float estimate_error;      // 估计误差协方差
float measure_error;       // 测量误差协方差

//推荐初始值 0，1，0.1（视情况调（大为快但误差大）（小为慢但误差小））
void kalman_Init(float Init_est,float Init_est_err,float Init_mea_err)
{
	estimate=Init_est;
	estimate_error=Init_est_err;
	measure_error=Init_mea_err;
}

//此函数为给值返回更新值
//建议每次使用时重置Init加每次使用循环几次得到较稳定准确值
float kalman_update(float measurement)
{
    // 预测步骤
    estimate_error += measure_error;

    // 更新步骤
    kalman_gain = estimate_error / (estimate_error + measure_error);
    estimate += kalman_gain * (measurement - estimate);
    estimate_error *= (1 - kalman_gain);

    return estimate;
}

//结果调用
float kalman_get(u8 turn)
{
	u8 i;
	float dists=0.0;
	kalman_Init(0,1,0.1);
	for(i=0;i<turn;i++)
	{
		dists=HC_GetDistance();
		dists=kalman_update(dists);
	}
	return dists;
}

void TIM3_Int_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 60000-1;
	TIM_TimeBaseStructure.TIM_Prescaler =72-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM3, DISABLE);
							 
}
