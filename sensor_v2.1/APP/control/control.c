#include "control.h"
#include "bsp_tcrt5000.h"
#include "servo.h"
#include "usart.h"
#include "math.h"
#include "pwm.h"
#include <stdlib.h>
#include "sg90.h"

int l = 400;
double k = 0.2;		//转向灵敏度

extern int x1,y1,x2,y2;
extern int L,R;

double Angle_x=0,Angle_y=90;

void Control_bluetooth(void)
{	
			if(abs(x1)<DEAD_ZONE)
			{
				x1=0;
			}
			if(abs(x2)<DEAD_ZONE)
			{
				x2=0;
			}
			if(abs(y1)<DEAD_ZONE)
			{
				y1=0;
			}
			if(abs(y2)<DEAD_ZONE)
			{
				y2=0;
			}//摇杆死区
			if(y1>0)
			{
				L=y1+k*x1;
				R=y1-k*x1;
			}
			else if(y1<0)
			{
				L=y1-k*x1;
				R=y1+k*x1;
			}
			else
			{
				L=y1+x1;
				R=y1-x1;
			}
			
			
			L=L / SMOOTH * SMOOTH;
			R=R / SMOOTH * SMOOTH;//简易滤波消抖 
			x2=x2 / SMOOTH * SMOOTH;
			y2=y2 / SMOOTH * SMOOTH;
			Angle_x= -x2/(1000.0/180)+90;
			Angle_y= -y2/(1000.0/180)+93;
			if(Angle_x>180)Angle_x=180;
			if(Angle_x<0)Angle_x=0;
			if(Angle_y>150)Angle_y=150;
			if(Angle_y<30)Angle_y=30;
			
			Left_Speed(L);
			Right_Speed(R);
		

//			if(x1==0&&y1==0)
//			{
//				stop();
//			}

}

