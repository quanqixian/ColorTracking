#include "pbdata.h"
unsigned char Raw_Data[18];//pixy原始数据
u16 counter,i,j;//计数用
int Pixy_PosX =0, Pixy_PosX_NEXT=0;//x坐标
int Pixy_PosY ,Pixy_PosY_NEXT=0;//y坐标
float zengx,pwmoutx,numx;//pwm控制参数x
float zengy,pwmouty,numy;//pwm控制参数y
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(void);
void TIM4_Configuration(void);

int fputc(int ch,FILE *f)//输出重定义
{
	USART_SendData(USART1,(u8)ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}
/*****************************************/
struct _pid
{
float aim;//定义设定值
float err;//定义偏差值
float err_next;//定义上一个偏差值
float err_last;//定义最上前的偏差值
float Kp,Ki,Kd;//定义比例、积分、微分系数
}pidx,pidy;

void PIDx_init()//xpid参数初始化
{
pidx.aim=0.0;
pidx.err=0.0;
pidx.err_last=0.0;
pidx.err_next=0.0;
pidx.Kp=0.6;
pidx.Ki=0.015;//0.015;
pidx.Kd=0.20;
}
void PIDy_init()//y pid参数初始化
{
pidy.aim=0.0;
pidy.err=0.0;
pidy.err_last=0.0;
pidy.err_next=0.0;
pidy.Kp=1.9;
pidy.Ki=0.016;//0.014;
pidy.Kd=0.21;
}
void PID_x(float aim_x)//x轴的pid计算
{
pidx.aim=aim_x;
pidx.err=pidx.aim- Pixy_PosX ;
zengx=pidx.Kp*(pidx.err-pidx.err_next)+pidx.Ki*pidx.err+pidx.Kd*(pidx.err-2*pidx.err_next+pidx.err_last);
pwmoutx+=zengx;//pwm输出加上增量
pidx.err_last=pidx.err_next;
pidx.err_next=pidx.err;
}
void PID_y(float aim_y)//y轴的pid计算
{
pidy.aim=aim_y;
pidy.err=pidy.aim- Pixy_PosY ;
zengy=pidy.Kp*(pidy.err-pidy.err_next)+pidy.Ki*pidy.err+pidy.Kd*(pidy.err-2*pidy.err_next+pidy.err_last);
pwmouty+=zengy;//pwm输出加上增量
pidy.err_last=pidy.err_next;
pidy.err_next=pidy.err;
}
void x_contral()//x轴总控制
{
	PID_x(160);				
	if(pwmoutx>0)numx=pwmoutx;
	if(pwmoutx<0)numx=-pwmoutx;
	if(pwmoutx>578)numx=578;
	if(pwmoutx<(-578))numx=578;
	if(pwmoutx>-1)
	{																
		TIM_SetCompare1(TIM4,0);
		TIM_SetCompare2(TIM4,numx+320);	 
	}
 if(pwmoutx<1)
	{			
		TIM_SetCompare1(TIM4,numx+320);
		TIM_SetCompare2(TIM4,0);
	}
}
void y_contral()//y轴总控制
{
 Pixy_PosY   = Raw_Data[10] + Raw_Data[11]*256;	
 PID_y(100);
		 //	printf("%6.2f,\r\n",pwmouty);	 
	if(pwmouty>500)pwmouty=500;
	if(pwmouty<(-500))numy=500;
	if(pwmouty>0)numy=pwmouty;
	if(pwmouty<0)numy=-pwmouty;
	
	if(pwmouty>0)
	{																
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,numy+200);				 
	}
 if(pwmouty<0)
	{			
	TIM_SetCompare3(TIM4,numy+200);
	TIM_SetCompare4(TIM4,0);		 
	}
}
/*****************************************/
int main(void)
{


   RCC_Configuration();	//系统时钟初始化
   GPIO_Configuration();//端口初始化
   USART_Configuration();
   NVIC_Configuration();
   TIM4_Configuration();
	 PIDx_init();
	 PIDy_init();
   for(;;)
   {					
		
						Pixy_PosX   = Raw_Data[8]  +Raw_Data[9]*256;
					
	          if(  Pixy_PosX !=Pixy_PosX_NEXT)  
						{	
                i=0,j=0;							
							x_contral();
							y_contral();		 	
						 }
						 else
						 {
									 i++;
							 if(i>50000)
							 {
								 j++;i=0;
									 if(j>20)
									 {
										 i=0,j=0;
									TIM_SetCompare1(TIM4,0);
									TIM_SetCompare2(TIM4,0);
									TIM_SetCompare3(TIM4,0);
									TIM_SetCompare4(TIM4,0);
//                 PIDx_init();										 
//							   PIDy_init();			
									 }
								}
							}
			          Pixy_PosX_NEXT =Pixy_PosX;
		

							
							 
   }	
}

void RCC_Configuration(void)
{
    SystemInit();//72m
	//串口2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	//串口1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	//pwm
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	//串口2
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//TX
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//RX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//串口1
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//TX
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//RX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//pwm部分
	 GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 |GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9  ;  
	 GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;         
   GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

void NVIC_Configuration(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	//串口1
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}

void USART_Configuration(void)
{
    USART_InitTypeDef  USART_InitStructure;
//串口2
	USART_InitStructure.USART_BaudRate=19200;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;

	USART_Init(USART2,&USART_InitStructure);//串口初始化
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//中断使能
	USART_Cmd(USART2,ENABLE);//对串口整个外设使能
	USART_ClearFlag(USART2,USART_FLAG_TC);//清空发送完成标准
	//串口1
	USART_InitStructure.USART_BaudRate=19200;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;

	USART_Init(USART1,&USART_InitStructure);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART1,ENABLE);
	USART_ClearFlag(USART1,USART_FLAG_TC);
}
void TIM4_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
//	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	TIM_TimeBaseStruct.TIM_Period=899;//初值
	TIM_TimeBaseStruct.TIM_Prescaler=0;//预分频
	TIM_TimeBaseStruct.TIM_ClockDivision=0;
	TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStruct);

//	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//中断使能
	
	
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
//  TIM_OCInitStructure.TIM_Pulse =880;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High; // TIM_OCPolarity_Low ;////当定时器计数值小于CCR1_Val时为高电平
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//使能通道1

 
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse =  880;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);//使能通道2

 
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse =  880;	//设置通道3的电平跳变值，输出另外一个占空比的PWM
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);//使能通道3

 
 // TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = 880;	//设置通道4的电平跳变值，输出另外一个占空比的PWM
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);	//使能通道4
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);			 // 使能TIM4重载寄存器ARR

	TIM_Cmd(TIM4,ENABLE);	 
}


