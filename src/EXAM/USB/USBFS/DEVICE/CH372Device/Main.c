/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/08/08
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/ 

/*
 *@Note
 模拟自定义USB设备（CH372设备）例程(仅适用于，CH32F20x_D6-CH32F20x_D8W-CH32F20x_D8C)：
 OTG_FS_DM(PA11)、OTG_FS_DP(PA12)
 本例程演示使用 USBHS 模拟自定义设备 CH372，和上位机通信。
 注：本例程需与上位机软件配合演示。
 
*/

#include "debug.h"
#include "ch32f20x_usbotg_device.h"

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	Delay_Init();
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);

    /* USBOTG_FS device init */
	printf( "CH372Device Running On USBOTG_FS Controller\n" );
	Delay_Ms(10);
	USBOTG_Init( );

	while(1)
	{ }
}






