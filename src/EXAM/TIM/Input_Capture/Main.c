/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/08/08
* Description        : Main program body.
*******************************************************************************/

/*
 *@Note
 ���벶�����̣�
 TIM1_CH1(PA8)
 ��������ʾ TIM_CH1(PA8) ���Ÿ������룬�����ż�⵽�������䴥�� TIM1 �����жϣ�
 �����ش��� TIM_IT_CC1 �жϣ��½��ش��� TIM_IT_CC2 �жϡ�
 
*/

#include "debug.h"

/*******************************************************************************
* Function Name  : Input_Capture_Init
* Description    : Initializes TIM1 input capture.
* Input          : arr: the period value.
*                  psc: the prescaler value.
*									 ccp: the pulse value.
* Return         : None
*******************************************************************************/
void Input_Capture_Init( u16 arr, u16 psc )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init( GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits( GPIOA, GPIO_Pin_8 );

	TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter =  0x00;
	TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

	TIM_PWMIConfig( TIM1, &TIM_ICInitStructure );

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig( TIM1, TIM_IT_CC1 | TIM_IT_CC2, ENABLE );

	TIM_SelectInputTrigger( TIM1, TIM_TS_TI1FP1 );
	TIM_SelectSlaveMode( TIM1, TIM_SlaveMode_Reset );
	TIM_SelectMasterSlaveMode( TIM1, TIM_MasterSlaveMode_Enable );
	TIM_Cmd( TIM1, ENABLE );
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Return         : None
*******************************************************************************/
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);

  Input_Capture_Init( 0xFFFF, 48000-1 );

	while(1);
}

/*******************************************************************************
* Function Name  : TIM1_CC_IRQHandler
* Description    : This function handles TIM1  Capture Compare Interrupt exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_CC_IRQHandler(void)
{
	if( TIM_GetITStatus( TIM1, TIM_IT_CC1 ) != RESET )
	{
		printf( "CH1_Val:%d\r\n", TIM_GetCapture1( TIM1 ) );
		TIM_SetCounter( TIM1, 0 );
	}

	if( TIM_GetITStatus( TIM1, TIM_IT_CC2 ) != RESET )
	{
		printf( "CH2_Val:%d\r\n", TIM_GetCapture2( TIM1 ) );
	}

	TIM_ClearITPendingBit( TIM1, TIM_IT_CC1 | TIM_IT_CC2 );
}
