/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/08/08
* Description        : Main program body.
*******************************************************************************/

/*
 *@Note 
  ˫ADC���ٽ���������̣�
 ADC1ͨ��2(PA2),ADC2ͨ��2(PA2)),������ͨ��ͨ��ADC�жϻ�ȡ˫ ADCת�����ݡ�
*/

#include "debug.h"

/* Global Variable */
u16 Adc_Val[2];
u32 temp;
s16 Calibrattion_Val1 = 0;
s16 Calibrattion_Val2 = 0;


/*******************************************************************************
* Function Name  : ADC_Function_Init
* Description    : Initializes ADC collection.
* Input          : None
* Return         : None
*******************************************************************************/
void  ADC_Function_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE );
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1  , ENABLE );
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2  , ENABLE );
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_InitStructure.ADC_Mode = ADC_Mode_FastInterl;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Disable;
    ADC_InitStructure.ADC_Pga = ADC_Pga_1;

    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_7Cycles5 );

    ADC_ITConfig( ADC1, ADC_IT_EOC, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_BufferCmd(ADC1, DISABLE);   //disable buffer
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
	Calibrattion_Val1 = Get_CalibrationValue(ADC1);
	
    ADC_BufferCmd(ADC1, ENABLE);   //enable buffer

    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_7Cycles5 );

    ADC_ITConfig( ADC2, ADC_IT_EOC, ENABLE);
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    ADC_BufferCmd(ADC2, DISABLE);   //disable buffer
    ADC_ResetCalibration(ADC2);
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));
	Calibrattion_Val2 = Get_CalibrationValue(ADC2);
	
    ADC_BufferCmd(ADC2, ENABLE);   //enable buffer
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
	Delay_Init();
	printf("SystemClk:%d\r\n",SystemCoreClock);
	ADC_Function_Init();
	printf("CalibrattionValue1:%d\n", Calibrattion_Val1);
	printf("CalibrattionValue2:%d\n", Calibrattion_Val2);

	while(1)
    {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);

        Delay_Ms(500);
    }
}

/*******************************************************************************
* Function Name  : ADC1_2_IRQHandler
* Description    : ADC1_2 Interrupt Service Function.
* Input          : None
* Return         : None
*******************************************************************************/
void ADC1_2_IRQHandler()
{
    if(ADC_GetITStatus( ADC1, ADC_IT_EOC)){
        temp=ADC1->RDATAR;
        Adc_Val[0]=temp&0xffff;
        Adc_Val[1]=(temp>>16)&0xffff;

        printf("\r\nADC1 ch2=%d\r\n",Adc_Val[0]+Calibrattion_Val1);
        printf("\r\nADC2 ch2=%d\r\n",Adc_Val[1]+Calibrattion_Val2);
    }
    ADC_ClearITPendingBit( ADC1, ADC_IT_EOC);
}

