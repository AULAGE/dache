#include "include.h"

/*
 * Interior angles (deg): T1=12-3-4, T2=3-4-5, T3=4-5 ref; T1+T2+T3=360.
 * T1=0.09*(p5-p4)+90, T2=0.09*p4+45, T3=225-0.09*p5  (p3 not in T1,T3 fit)
 * Corners (p3,p4,p5):
 *   Back / init: (90,90,180)     @ (500,500,500)     p3+p4-p5=500
 *   Most forward: (180,180,0)   @ (1500,1500,2500)  p3+p4-p5=500
 *   Most up: (90,180,90)        @ (500,1500,1500)   p3+p4-p5=500
 *   Forward-down: (270,90,0)    @ (2500,500,2500)  p3+p4-p5=500
 * '6' down: pwm5>=1500 -> (270,90,0); pwm5<1500 -> init (90,90,180)
 */
#define AH_PWM_MIN	500
#define AH_PWM_MAX	2500
#define AH_STEP_DEG	2.0f
#define AH_DOWN_P5_GT	1500

static float AH_fabsf(float x)
{
	return (x >= 0.f) ? x : -x;
}

static uint8 ah_angles_inited = 0;
static float ah_t1, ah_t2, ah_t3;

static void AH_pwm_to_T(float p3, float p4, float p5, float *t1, float *t2, float *t3)
{
	(void)p3;
	*t1 = 0.09f * (p5 - p4) + 90.0f;
	*t2 = 0.09f * p4 + 45.0f;
	*t3 = 225.0f - 0.09f * p5;
}

static void AH_sync_angles_from_pwm(void)
{
	AH_pwm_to_T((float)ServoPwmDutySet[3], (float)ServoPwmDutySet[4], (float)ServoPwmDutySet[5],
		&ah_t1, &ah_t2, &ah_t3);
	ah_angles_inited = 1;
}

static void AH_clamp_T_box(float *t1, float *t2, float *t3)
{
	if (*t1 < 90.f) *t1 = 90.f;
	if (*t1 > 270.f) *t1 = 270.f;
	if (*t2 < 90.f) *t2 = 90.f;
	if (*t2 > 180.f) *t2 = 180.f;
	if (*t3 < 0.f) *t3 = 0.f;
	if (*t3 > 180.f) *t3 = 180.f;
}

static void AH_fix_sum360(float *t1, float *t2, float *t3)
{
	float e = (360.f - (*t1 + *t2 + *t3)) / 3.f;
	*t1 += e;
	*t2 += e;
	*t3 += e;
	AH_clamp_T_box(t1, t2, t3);
	e = (360.f - (*t1 + *t2 + *t3)) / 3.f;
	*t1 += e;
	*t2 += e;
	*t3 += e;
	AH_clamp_T_box(t1, t2, t3);
}

static void AH_T_to_pwm(float t1, float t2, float t3, float *p3, float *p4, float *p5)
{
	float p4f, p5f, p3a, p3b, sca, scb;
	int32 pi;
	float p3c, best, sc;

	(void)t1;
	p4f = (t2 - 45.0f) / 0.09f;
	p5f = (225.0f - t3) / 0.09f;
	if (p4f < AH_PWM_MIN) p4f = (float)AH_PWM_MIN;
	if (p4f > AH_PWM_MAX) p4f = (float)AH_PWM_MAX;
	if (p5f < AH_PWM_MIN) p5f = (float)AH_PWM_MIN;
	if (p5f > AH_PWM_MAX) p5f = (float)AH_PWM_MAX;

	p3a = 500.0f - p4f + p5f;
	p3b = 1500.0f - p4f + p5f;
	sca = 1.0e12f;
	scb = 1.0e12f;
	if (p3a >= AH_PWM_MIN && p3a <= AH_PWM_MAX)
		sca = AH_fabsf(p3a - (float)ServoPwmDutySet[3]);
	if (p3b >= AH_PWM_MIN && p3b <= AH_PWM_MAX)
		scb = AH_fabsf(p3b - (float)ServoPwmDutySet[3]);

	if (sca < 1.0e11f || scb < 1.0e11f)
	{
		if (sca <= scb && sca < 1.0e11f)
			*p3 = p3a;
		else if (scb < 1.0e11f)
			*p3 = p3b;
		else
			*p3 = p3a;
		*p4 = p4f;
		*p5 = p5f;
		return;
	}

	best = 1.0e12f;
	*p3 = (float)AH_PWM_MIN;
	for (pi = AH_PWM_MIN; pi <= AH_PWM_MAX; pi += 25)
	{
		p3c = (float)pi;
		sc = AH_fabsf(p3c - (float)ServoPwmDutySet[3]);
		if (sc < best)
		{
			best = sc;
			*p3 = p3c;
		}
	}
	*p4 = p4f;
	*p5 = p5f;
}

static uint8 AH_pwm_in_range(float p3, float p4, float p5)
{
	if (p3 < AH_PWM_MIN || p3 > AH_PWM_MAX)
		return 0;
	if (p4 < AH_PWM_MIN || p4 > AH_PWM_MAX)
		return 0;
	if (p5 < AH_PWM_MIN || p5 > AH_PWM_MAX)
		return 0;
	return 1;
}

static void AH_step_toward(float g1, float g2, float g3)
{
	float d1, d2, d3, ds, m, s, k;
	float n1, n2, n3;
	float p3, p4, p5;
	int iter;

	if (!ah_angles_inited)
		AH_sync_angles_from_pwm();

	d1 = g1 - ah_t1;
	d2 = g2 - ah_t2;
	d3 = g3 - ah_t3;
	{
		float ge = AH_fabsf(d1);
		if (AH_fabsf(d2) > ge) ge = AH_fabsf(d2);
		if (AH_fabsf(d3) > ge) ge = AH_fabsf(d3);
		if (ge < 0.02f)
			return;
		if (ge < 1.25f)
		{
			n1 = g1;
			n2 = g2;
			n3 = g3;
			AH_clamp_T_box(&n1, &n2, &n3);
			if (AH_fabsf(n1 + n2 + n3 - 360.f) > 0.05f)
				AH_fix_sum360(&n1, &n2, &n3);
			AH_T_to_pwm(n1, n2, n3, &p3, &p4, &p5);
			if (AH_pwm_in_range(p3, p4, p5))
			{
				ServoPwmDutySet[3] = (uint16)(p3 + 0.5f);
				ServoPwmDutySet[4] = (uint16)(p4 + 0.5f);
				ServoPwmDutySet[5] = (uint16)(p5 + 0.5f);
				AH_pwm_to_T((float)ServoPwmDutySet[3], (float)ServoPwmDutySet[4], (float)ServoPwmDutySet[5],
					&ah_t1, &ah_t2, &ah_t3);
				ServoSetPluseAndTime(3, ServoPwmDutySet[3], 50);
				ServoSetPluseAndTime(4, ServoPwmDutySet[4], 50);
				ServoSetPluseAndTime(5, ServoPwmDutySet[5], 50);
				return;
			}
			/* snap not feasible: fall through to projected stepping */
		}
	}

	ds = d1 + d2 + d3;
	d1 -= ds / 3.f;
	d2 -= ds / 3.f;
	d3 -= ds / 3.f;

	m = AH_fabsf(d1);
	if (AH_fabsf(d2) > m) m = AH_fabsf(d2);
	if (AH_fabsf(d3) > m) m = AH_fabsf(d3);
	if (m < 0.001f)
		return;

	s = AH_STEP_DEG / m;
	if (s > 1.f)
		s = 1.f;

	for (k = s, iter = 0; iter < 14 && k > 0.001f; iter++, k *= 0.5f)
	{
		n1 = ah_t1 + d1 * k;
		n2 = ah_t2 + d2 * k;
		n3 = ah_t3 + d3 * k;
		AH_clamp_T_box(&n1, &n2, &n3);
		AH_fix_sum360(&n1, &n2, &n3);
		AH_T_to_pwm(n1, n2, n3, &p3, &p4, &p5);
		if (AH_pwm_in_range(p3, p4, p5))
		{
			ServoPwmDutySet[3] = (uint16)(p3 + 0.5f);
			ServoPwmDutySet[4] = (uint16)(p4 + 0.5f);
			ServoPwmDutySet[5] = (uint16)(p5 + 0.5f);
			AH_pwm_to_T((float)ServoPwmDutySet[3], (float)ServoPwmDutySet[4], (float)ServoPwmDutySet[5],
				&ah_t1, &ah_t2, &ah_t3);
			ServoSetPluseAndTime(3, ServoPwmDutySet[3], 50);
			ServoSetPluseAndTime(4, ServoPwmDutySet[4], 50);
			ServoSetPluseAndTime(5, ServoPwmDutySet[5], 50);
			return;
		}
	}
}

static void ArmHoriz_StepForward(void)
{
	AH_step_toward(180.f, 180.f, 0.f);
}

static void ArmHoriz_StepBackward(void)
{
	AH_step_toward(90.f, 90.f, 180.f);
}

static void ArmHoriz_StepUp(void)
{
	AH_step_toward(90.f, 180.f, 90.f);
}

static void ArmHoriz_StepDown(void)
{
	if (!ah_angles_inited)
		AH_sync_angles_from_pwm();
	if (ServoPwmDutySet[5] >= AH_DOWN_P5_GT)
		AH_step_toward(270.f, 90.f, 0.f);
	else
		AH_step_toward(90.f, 90.f, 180.f);
}


static bool fUartRxComplete = FALSE;
static uint8 UartRxBuffer[260];
uint8 Uart1RxBuffer[260];

// static bool UartBusy = FALSE;

uint8  frameIndexSumSum[256];


void InitUart1(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART ?????????

	USART_InitStructure.USART_BaudRate = 9600;//????????9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//????????

	USART_Cmd(USART1, ENABLE);                    //??????
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ??????
	NVIC_Init(&NVIC_InitStructure);	//????NVIC_InitStruct???????????????????NVIC?????USART1
}

void Uart1SendData(BYTE dat)
{
	while((USART1->SR&0X40)==0);//???????,??????????
	USART1->DR = (u8) dat;
	while((USART1->SR&0X40)==0);//???????,??????????
}

void UART1SendDataPacket(uint8 dat[],uint8 count)
{
	uint32 i;
	for(i = 0; i < count; i++)
	{
//		USART1_TransmitData(tx[i]);
		while((USART1->SR&0X40)==0);//???????,??????????
		USART1->DR = dat[i];
		while((USART1->SR&0X40)==0);//???????,??????????
	}
}


void USART1_IRQHandler(void)
{
	uint8 i;
	uint8 rxBuf;

	static uint8 startCodeSum = 0;
	static bool fFrameStart = FALSE;
	static uint8 messageLength = 0;
	static uint8 messageLengthSum = 2;
	

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {

        rxBuf = USART_ReceiveData(USART1);//(USART1->DR);	//??????????????
				if(rxBuf == '1')  /* ?????5????+??4????-??????3?????? */
				{
					ArmHoriz_StepForward();
				}
				else if(rxBuf == '2')  /* ???5-??4+ */
				{
					ArmHoriz_StepBackward();
				}
				else if(rxBuf == '3')  //??????6??
				{
					if(ServoPwmDutySet[6] > 500) ServoPwmDutySet[6] -= 20;
					ServoSetPluseAndTime(6, ServoPwmDutySet[6], 50);
				}
				else if(rxBuf == '4')  //????????6??
				{
					if(ServoPwmDutySet[6] < 2500) ServoPwmDutySet[6] += 20;
					ServoSetPluseAndTime(6, ServoPwmDutySet[6], 50);
				}
				else if(rxBuf == '5')  /* ?????????3????+20??4-/5+??????? */
				{
					ArmHoriz_StepUp();
				}
				else if(rxBuf == '6')  /* ?????????3????-20 */
				{
					ArmHoriz_StepDown();
				}
		if(!fFrameStart)
		{
			if(rxBuf == 0x55)
			{

				startCodeSum++;
				if(startCodeSum == 2)
				{
					startCodeSum = 0;
					fFrameStart = TRUE;
					messageLength = 1;
				}
			}
			else
			{

				fFrameStart = FALSE;
				messageLength = 0;
	
				startCodeSum = 0;
			}
			
		}
		if(fFrameStart)
		{
			Uart1RxBuffer[messageLength] = rxBuf;
			if(messageLength == 2)
			{
				messageLengthSum = Uart1RxBuffer[messageLength];
				if(messageLengthSum < 2)// || messageLengthSum > 30
				{
					messageLengthSum = 2;
					fFrameStart = FALSE;
					
				}
					
			}
			messageLength++;
	
			if(messageLength == messageLengthSum + 2) 
			{
				if(fUartRxComplete == FALSE)
				{
					fUartRxComplete = TRUE;
					for(i = 0;i < messageLength;i++)
					{
						UartRxBuffer[i] = Uart1RxBuffer[i];
					}
				}
				

				fFrameStart = FALSE;
			}
		}
    }

}

void McuToPCSendData(uint8 cmd,uint8 prm1,uint8 prm2)
{
	uint8 dat[8];
	uint8 datlLen = 2;
	switch(cmd)
	{

//		case CMD_ACTION_DOWNLOAD:
//			datlLen = 2;
//			break;

		default:
			datlLen = 2;
			break;
	}

	dat[0] = 0x55;
	dat[1] = 0x55;
	dat[2] = datlLen;
	dat[3] = cmd;
	dat[4] = prm1;
	dat[5] = prm2;
	UART1SendDataPacket(dat,datlLen + 2);
}

static bool UartRxOK(void)
{
	if(fUartRxComplete)
	{
		fUartRxComplete = FALSE;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
void FlashEraseAll(void);
void SaveAct(uint8 fullActNum,uint8 frameIndexSum,uint8 frameIndex,uint8* pBuffer);
void TaskPCMsgHandle(void)
{

	uint16 i;
	uint8 cmd;
	uint8 id;
	uint8 servoCount;
	uint16 time;
	uint16 pos;
	uint16 times;
	uint8 fullActNum;
	if(UartRxOK())
	{
		LED = !LED;
		cmd = UartRxBuffer[3];
 		switch(cmd)
 		{
 			case CMD_MULT_SERVO_MOVE:
				servoCount = UartRxBuffer[4];
				time = UartRxBuffer[5] + (UartRxBuffer[6]<<8);
				for(i = 0; i < servoCount; i++)
				{
					id =  UartRxBuffer[7 + i * 3];
					pos = UartRxBuffer[8 + i * 3] + (UartRxBuffer[9 + i * 3]<<8);
	
					ServoSetPluseAndTime(id,pos,time);
					BusServoCtrl(id,SERVO_MOVE_TIME_WRITE,pos,time);
					if (id == 3 || id == 4 || id == 5)
						ah_angles_inited = 0;
				}				
 				break;
			
			case CMD_FULL_ACTION_RUN:
				fullActNum = UartRxBuffer[4];//????????
				times = UartRxBuffer[5] + (UartRxBuffer[6]<<8);//????????
				McuToPCSendData(CMD_FULL_ACTION_RUN, 0, 0);
				FullActRun(fullActNum,times);
				break;
				
			case CMD_FULL_ACTION_STOP:
				FullActStop();
				break;
				
			case CMD_FULL_ACTION_ERASE:
				FlashEraseAll();
				McuToPCSendData(CMD_FULL_ACTION_ERASE,0,0);
				break;

			case CMD_ACTION_DOWNLOAD:
				SaveAct(UartRxBuffer[4],UartRxBuffer[5],UartRxBuffer[6],UartRxBuffer + 7);
				McuToPCSendData(CMD_ACTION_DOWNLOAD,0,0);
				break;
 		}
	}
}
void SaveAct(uint8 fullActNum,uint8 frameIndexSum,uint8 frameIndex,uint8* pBuffer)
{
	uint8 i;
	
	if(frameIndex == 0)//?????????????????????
	{//??????????16k???????????????????4k?????????4??
		for(i = 0;i < 4;i++)//ACT_SUB_FRAME_SIZE/4096 = 4
		{
			FlashEraseSector((MEM_ACT_FULL_BASE) + (fullActNum * ACT_FULL_SIZE) + (i * 4096));
		}
	}

	FlashWrite((MEM_ACT_FULL_BASE) + (fullActNum * ACT_FULL_SIZE) + (frameIndex * ACT_SUB_FRAME_SIZE)
		,ACT_SUB_FRAME_SIZE,pBuffer);
	
	if((frameIndex + 1) ==  frameIndexSum)
	{
		FlashRead(MEM_FRAME_INDEX_SUM_BASE,256,frameIndexSumSum);
		frameIndexSumSum[fullActNum] = frameIndexSum;
		FlashEraseSector(MEM_FRAME_INDEX_SUM_BASE);
		FlashWrite(MEM_FRAME_INDEX_SUM_BASE,256,frameIndexSumSum);
	}
}


void FlashEraseAll(void)
{//??????255???????????????????0???????????????????????
	uint16 i;
	
	for(i = 0;i <= 255;i++)
	{
		frameIndexSumSum[i] = 0;
	}
	FlashEraseSector(MEM_FRAME_INDEX_SUM_BASE);
	FlashWrite(MEM_FRAME_INDEX_SUM_BASE,256,frameIndexSumSum);
}

void InitMemory(void)
{
	uint8 i;
	uint8 logo[] = "LOBOT";
	uint8 datatemp[8];

	FlashRead(MEM_LOBOT_LOGO_BASE,5,datatemp);
	for(i = 0; i < 5; i++)
	{
		if(logo[i] != datatemp[i])
		{
		LED = LED_ON;
			//??????????????????????FLASH??????????
			FlashEraseSector(MEM_LOBOT_LOGO_BASE);
			FlashWrite(MEM_LOBOT_LOGO_BASE,5,logo);
			FlashEraseAll();
			break;
		}
	}
	
}



