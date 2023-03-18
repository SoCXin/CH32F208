/********************************** (C) COPYRIGHT *******************************
* File Name          : eth_driver.c
* Author             : WCH
* Version            : V1.3.0
* Date               : 2022/06/02
* Description        : eth program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "eth_driver.h"

 __attribute__((__aligned__(4))) ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB];      /* MAC receive descriptor, 4-byte aligned*/
 __attribute__((__aligned__(4))) ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB];      /* MAC send descriptor, 4-byte aligned */

 __attribute__((__aligned__(4))) uint8_t  MACRxBuf[ETH_RXBUFNB*ETH_RX_BUF_SZE];     /* MAC receive buffer, 4-byte aligned */
 __attribute__((__aligned__(4))) uint8_t  MACTxBuf[ETH_TXBUFNB*ETH_TX_BUF_SZE];     /* MAC send buffer, 4-byte aligned */

__attribute__((__aligned__(4))) SOCK_INF SocketInf[WCHNET_MAX_SOCKET_NUM];          /* Socket information table, 4-byte alignment */
const uint16_t MemNum[8] = {WCHNET_NUM_IPRAW,
                         WCHNET_NUM_UDP,
                         WCHNET_NUM_TCP,
                         WCHNET_NUM_TCP_LISTEN,
                         WCHNET_NUM_TCP_SEG,
                         WCHNET_NUM_IP_REASSDATA,
                         WCHNET_NUM_PBUF,
                         WCHNET_NUM_POOL_BUF
                         };
const uint16_t MemSize[8] = {WCHNET_MEM_ALIGN_SIZE(WCHNET_SIZE_IPRAW_PCB),
                          WCHNET_MEM_ALIGN_SIZE(WCHNET_SIZE_UDP_PCB),
                          WCHNET_MEM_ALIGN_SIZE(WCHNET_SIZE_TCP_PCB),
                          WCHNET_MEM_ALIGN_SIZE(WCHNET_SIZE_TCP_PCB_LISTEN),
                          WCHNET_MEM_ALIGN_SIZE(WCHNET_SIZE_TCP_SEG),
                          WCHNET_MEM_ALIGN_SIZE(WCHNET_SIZE_IP_REASSDATA),
                          WCHNET_MEM_ALIGN_SIZE(WCHNET_SIZE_PBUF),
                          WCHNET_MEM_ALIGN_SIZE(WCHNET_SIZE_PBUF) + WCHNET_MEM_ALIGN_SIZE(WCHNET_SIZE_POOL_BUF)
                         };
 __attribute__((__aligned__(4)))uint8_t Memp_Memory[WCHNET_MEMP_SIZE];
 __attribute__((__aligned__(4)))uint8_t Mem_Heap_Memory[WCHNET_RAM_HEAP_SIZE];
 __attribute__((__aligned__(4)))uint8_t Mem_ArpTable[WCHNET_RAM_ARP_TABLE_SIZE];

uint16_t gPHYAddress;
uint32_t volatile LocalTime;
ETH_DMADESCTypeDef *pDMARxSet;
ETH_DMADESCTypeDef *pDMATxSet;
																		 
#if defined(CH32F20x_D8W)									 
ETH_DMADESCTypeDef *DMATxDescToSet;
ETH_DMADESCTypeDef *DMARxDescToGet;
uint32_t phyLinkTime;
uint8_t phyLinkStatus=0;
uint8_t phyStatus=0;
uint8_t phyPN=0x01;
uint8_t phyPNChangeCnt = 0;
uint8_t phyLinkCnt = 0;
uint8_t phyRetryCnt = 0;
uint8_t phySucCnt = 0;						 
#endif										 

#if( defined(CH32F20x_D8C) && (PHY_MODE ==  USE_10M_BASE) )	
uint32_t phyLinkTime;
uint8_t  phyLinkStatus = 0;
uint8_t  phyStatus = 0;
uint8_t  phyRetryCnt = 0;
uint8_t  phyLinkCnt = 0;
uint8_t  phySucCnt = 0;
uint8_t  phyPN = PHY_PN_SWITCH_AUTO;
#endif

#if( defined(CH32F20x_D8C) && (PHY_MODE ==  USE_MAC_MII) )								 
u16 LastPhyStat = 0;
u32 LastQueryPhyTime = 0;
#endif
/*********************************************************************
 * @fn      WCHNET_GetMacAddr
 *
 * @brief   Get MAC address
 *
 * @return  none.
 */
void WCHNET_GetMacAddr( uint8_t *p )
{
    uint8_t i;
    uint8_t *macaddr=(uint8_t *)(ROM_CFG_USERADR_ID+5);

    for(i=0;i<6;i++)
    {
        *p = *macaddr;
        p++;
        macaddr--;
    }
}

/*********************************************************************
 * @fn      WCHNET_TimeIsr
 *
 * @brief
 *
 * @return  none.
 */
void WCHNET_TimeIsr( uint16_t timperiod )
{
    LocalTime += timperiod;
}

#if defined(CH32F20x_D8C)	
/*********************************************************************
 * @fn      WCHNET_QueryPhySta
 *
 * @brief   Query external PHY status
 *
 * @return  none.
 */
#if( PHY_MODE ==  USE_MAC_MII )
void WCHNET_QueryPhySta(void)
{
    u16 phy_stat;
    if(QUERY_STAT_FLAG){                                         /* Query the PHY link status every 1s */
        LastQueryPhyTime = LocalTime / 1000;
        phy_stat = ETH_ReadPHYRegister( PHY_ADDRESS, PHY_BSR );
        if(phy_stat != LastPhyStat){
            ETH_PHYLink();
        }
    }
}
#endif

#if( PHY_MODE ==  USE_10M_BASE )
/*********************************************************************
 * @fn      WCHNET_LinkProcess
 *
 * @brief   link process.
 *
 * @param   none.
 *
 * @return  none.
 */
void WCHNET_LinkProcess( void )
{
    uint16_t phy_anlpar, phy_bmsr, RegVal;
    phy_anlpar = ETH_ReadPHYRegister(gPHYAddress, PHY_ANLPAR);
    phy_bmsr = ETH_ReadPHYRegister( gPHYAddress, PHY_BMSR);

    if( (phy_anlpar&PHY_ANLPAR_SELECTOR_FIELD) )
    {
        if( !(phyLinkStatus&PHY_LINK_WAIT_SUC) )
        {
            if( phyPN == PHY_PN_SWITCH_AUTO )
            {
                PHY_PN_SWITCH(PHY_PN_SWITCH_P);
            }
            else if( phyPN == PHY_PN_SWITCH_P )
            {
                phyLinkStatus = PHY_LINK_WAIT_SUC;
            }
            else
            {
                phyLinkStatus = PHY_LINK_WAIT_SUC;
            }
        }
        else{
            if((phySucCnt++ == 5) && ((phy_bmsr&(1<<5)) == 0))
            {
                phySucCnt = 0;
                RegVal = ETH_ReadPHYRegister(gPHYAddress, PHY_BCR);
                RegVal |= 1<<9;
                ETH_WritePHYRegister( gPHYAddress, PHY_BCR, RegVal);
                phyPN ^= PHY_PN_SWITCH_N;
                ETH_WritePHYRegister(gPHYAddress, PHY_MDIX, phyPN);
            }
        }
        phyLinkCnt = 0;
        phyRetryCnt = 0;
    }
    else
    {
        if( phyLinkStatus == PHY_LINK_WAIT_SUC )
        {
            phyRetryCnt = 0;
            if(phyLinkCnt++ == 15 )
            {
                phyLinkCnt = 0;
                phySucCnt = 0;
                phyLinkStatus = PHY_LINK_INIT;
                PHY_PN_SWITCH(PHY_PN_SWITCH_AUTO);
            }
        }
        else
        {
            if( phyPN == PHY_PN_SWITCH_P )
            {
                PHY_PN_SWITCH(PHY_PN_SWITCH_N);
            }
            else if( phyPN == PHY_PN_SWITCH_N )
            {
                phyRetryCnt = 0;
                if(phyLinkCnt++ == 15 )
                {
                    phyLinkCnt = 0;
                    phySucCnt = 0;
                    phyLinkStatus = PHY_LINK_INIT;
                    PHY_PN_SWITCH(PHY_PN_SWITCH_AUTO);
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      WCHNET_HandlePhyNegotiation
 *
 * @brief   Handle PHY Negotiation.
 *
 * @param   none.
 *
 * @return  none.
 */
void WCHNET_HandlePhyNegotiation(void)
{
    if( !phyStatus )                        /* Handling PHY Negotiation Exceptions */
    {
        if(phyLinkTime > LocalTime)
            phyLinkTime = LocalTime;
        if( LocalTime - phyLinkTime >= PHY_LINK_TASK_PERIOD )  /* 50ms cycle timing call */
        {
            phyLinkTime = LocalTime;
            PHY_RESTART_NEGOTIATION( );
            WCHNET_LinkProcess( );
        }
    }
}
#endif
/*********************************************************************
 * @fn      WCHNET_MainTask
 *
 * @brief   library main task function
 *
 * @return  none.
 */
void WCHNET_MainTask(void)
{
    WCHNET_NetInput( );                     /* Ethernet data input */
    WCHNET_PeriodicHandle( );               /* Protocol stack time-related task processing */

#if( PHY_MODE ==  USE_10M_BASE )
    WCHNET_HandlePhyNegotiation();
#endif

#if( PHY_MODE ==  USE_MAC_MII )
    WCHNET_QueryPhySta();                   /* Query external PHY status*/
#endif
}

#if( PHY_MODE ==  USE_10M_BASE )
/*********************************************************************
 * @fn      ETH_LedLinkSet
 *
 * @brief   set eth link led,setbit 0 or 1,the link led turn on or turn off
 *
 * @return  none
 */
void ETH_LedLinkSet( uint8_t mode )
{
    if( mode == LED_OFF )
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_0);
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);
    }
}

/*********************************************************************
 * @fn      ETH_LedDataSet
 *
 * @brief   set eth data led,setbit 0 or 1,the data led turn on or turn off
 *
 * @return  none
 */
void ETH_LedDataSet( uint8_t mode )
{
    if( mode == LED_OFF )
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_1);
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);
    }
}

/*********************************************************************
 * @fn      ETH_LedConfiguration
 *
 * @brief   set eth data and link led pin
 *
 * @return  none
 */
void ETH_LedConfiguration(void)
{
    GPIO_InitTypeDef  GPIO={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    GPIO.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO);
    ETH_LedDataSet(LED_OFF);
    ETH_LedLinkSet(LED_OFF);
}

/*********************************************************************
 * @fn      ETH_SetClock
 *
 * @brief   Set ETH Clock(60MHZ).
 *
 * @return  none
 */
void ETH_SetClock(void)
{
    RCC_PLL3Cmd(DISABLE);
    RCC_PREDIV2Config(RCC_PREDIV2_Div2);                // HSE = 8M
    RCC_PLL3Config(RCC_PLL3Mul_15);                     // 4M*15 = 60MHz
    RCC_PLL3Cmd(ENABLE);
    while(RESET == RCC_GetFlagStatus(RCC_FLAG_PLL3RDY));
}
#elif( PHY_MODE ==  USE_MAC_MII )
/*********************************************************************
 * @fn      ETH_MIIPinInit
 *
 * @brief   PHY MII interface GPIO initialization.
 *
 * @return  none
 */
void ETH_MIIPinInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);

    define_O(GPIOA,GPIO_Pin_2);                                                         /* MDIO */
    define_O(GPIOC,GPIO_Pin_1);                                                         /* MDC */

    define_I(GPIOC,GPIO_Pin_3);                                                         /* txclk */
    define_O(GPIOB,GPIO_Pin_11);                                                        /* txen */
    define_O(GPIOB,GPIO_Pin_12);                                                        /* txd0 */
    define_O(GPIOB,GPIO_Pin_13);                                                        /* txd1 */
    define_O(GPIOC,GPIO_Pin_2);                                                         /* txd2 */
    define_O(GPIOB,GPIO_Pin_8);                                                         /* txd3 */
    /* RX */
    define_I(GPIOA,GPIO_Pin_1);                                                         /* PA1 RXC */
    define_I(GPIOA,GPIO_Pin_7);                                                         /* PA7 RXDV */
    define_I(GPIOC,GPIO_Pin_4);                                                         /* RXD0 */
    define_I(GPIOC,GPIO_Pin_5);                                                         /* RXD1 */
    define_I(GPIOB,GPIO_Pin_0);                                                         /* RXD2 */
    define_I(GPIOB,GPIO_Pin_1);                                                         /* RXD3 */
    define_I(GPIOB,GPIO_Pin_10);                                                        /* RXER */

    define_O(GPIOA,GPIO_Pin_0);                                                         /* PA0 */
    define_O(GPIOA,GPIO_Pin_3);                                                         /* PA3 */
}
#endif

void ETH_PHYLink( void )
{
    u32 phy_stat;

#if( PHY_MODE ==  USE_10M_BASE )
    u16 phy_anlpar;
    phy_anlpar = ETH_ReadPHYRegister( gPHYAddress, PHY_ANLPAR);
    phy_stat = ETH_ReadPHYRegister( gPHYAddress, PHY_BSR);

    if((phy_stat&(PHY_Linked_Status))&&(phy_anlpar == 0)){                         //restart negotiation
        ETH_WritePHYRegister(gPHYAddress, PHY_BCR, PHY_Reset);
        EXTEN->EXTEN_CTR &= ~EXTEN_ETH_10M_EN;
        Delay_Ms(500);
        EXTEN->EXTEN_CTR |= EXTEN_ETH_10M_EN;
		PHY_NEGOTIATION_PARAM_INIT( );
        return;
    }
    WCHNET_PhyStatus( phy_stat );

    if( (phy_stat&(PHY_Linked_Status)) && (phy_stat&PHY_AutoNego_Complete) )
    {
        phy_stat = ETH_ReadPHYRegister( gPHYAddress, PHY_STATUS );
        if( phy_stat & (1<<2) )
        {
            ETH->MACCR |= ETH_Mode_FullDuplex;
        }
        else
        {
            if( (phy_anlpar&PHY_ANLPAR_SELECTOR_FIELD) != PHY_ANLPAR_SELECTOR_VALUE )
            {
                ETH->MACCR |= ETH_Mode_FullDuplex;
            }
            else
            {
                ETH->MACCR &= ~ETH_Mode_FullDuplex;
            }
        }
        ETH->MACCR &= ~(ETH_Speed_100M|ETH_Speed_1000M);
        phyStatus = PHY_Linked_Status;
        ETH_Start( );
    }
    else
    {
        PHY_NEGOTIATION_PARAM_INIT( );
    }
#else
    phy_stat = ETH_ReadPHYRegister( PHY_ADDRESS, PHY_BSR );
    LastPhyStat = phy_stat;
    WCHNET_PhyStatus( phy_stat );
    if( (phy_stat&PHY_Linked_Status) && (phy_stat&PHY_AutoNego_Complete) )
    {
        phy_stat = ETH_ReadPHYRegister( PHY_ADDRESS, PHY_BCR );
        /* PHY negotiation result */
        if(phy_stat&(1<<13))                                   //100M
        {
            ETH->MACCR &= ~(ETH_Speed_100M|ETH_Speed_1000M);
            ETH->MACCR |= ETH_Speed_100M;
        }
        else                                                  //10M
        {
            ETH->MACCR &= ~(ETH_Speed_100M|ETH_Speed_1000M);
        }
        if(phy_stat&(1<<8))                                   //full duplex
        {
            ETH->MACCR |= ETH_Mode_FullDuplex;
        }
        else                                                  //half duplex
        {
            ETH->MACCR &= ~ETH_Mode_FullDuplex;
        }
        ETH_Start( );
    }
#endif
}

/*********************************************************************
 * @fn      ETH_RegInit
 *
 * @brief   ETH register initialization.
 *
 * @param   ETH_InitStruct:initialization struct.
 *          PHYAddress:PHY address.
 *
 * @return  Initialization status.
 */
uint32_t ETH_RegInit( ETH_InitTypeDef* ETH_InitStruct, uint16_t PHYAddress )
{
    uint32_t tmpreg = 0;

    /*---------------------- Physical layer configuration -------------------*/
    /* Set the SMI interface clock, set as the main frequency divided by 42  */
    tmpreg = ETH->MACMIIAR;
    tmpreg &= MACMIIAR_CR_MASK;
    tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div42;
    ETH->MACMIIAR = (uint32_t)tmpreg;

    /*------------------------ MAC register configuration  ----------------------- --------------------*/
    tmpreg = ETH->MACCR;
    tmpreg &= MACCR_CLEAR_MASK;
    tmpreg |= (uint32_t)(ETH_InitStruct->ETH_AutoNegotiation |
                  ETH_InitStruct->ETH_Watchdog |
                  ETH_InitStruct->ETH_Jabber |
                  ETH_InitStruct->ETH_InterFrameGap |
                  ETH_InitStruct->ETH_CarrierSense |
                  ETH_InitStruct->ETH_Speed |
                  ETH_InitStruct->ETH_ReceiveOwn |
                  ETH_InitStruct->ETH_LoopbackMode |
                  ETH_InitStruct->ETH_Mode |
                  ETH_InitStruct->ETH_ChecksumOffload |
                  ETH_InitStruct->ETH_RetryTransmission |
                  ETH_InitStruct->ETH_AutomaticPadCRCStrip |
                  ETH_InitStruct->ETH_BackOffLimit |
                  ETH_InitStruct->ETH_DeferralCheck);
    /* Write MAC Control Register */
    ETH->MACCR = (uint32_t)tmpreg;
#if( PHY_MODE ==  USE_10M_BASE )
    ETH->MACCR |= ETH_Internal_Pull_Up_Res_Enable;/*  */
#endif
    ETH->MACFFR = (uint32_t)(ETH_InitStruct->ETH_ReceiveAll |
                          ETH_InitStruct->ETH_SourceAddrFilter |
                          ETH_InitStruct->ETH_PassControlFrames |
                          ETH_InitStruct->ETH_BroadcastFramesReception |
                          ETH_InitStruct->ETH_DestinationAddrFilter |
                          ETH_InitStruct->ETH_PromiscuousMode |
                          ETH_InitStruct->ETH_MulticastFramesFilter |
                          ETH_InitStruct->ETH_UnicastFramesFilter);
    /*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
    /* Write to ETHERNET MACHTHR */
    ETH->MACHTHR = (uint32_t)ETH_InitStruct->ETH_HashTableHigh;
    /* Write to ETHERNET MACHTLR */
    ETH->MACHTLR = (uint32_t)ETH_InitStruct->ETH_HashTableLow;
    /*----------------------- ETHERNET MACFCR Configuration --------------------*/
    /* Get the ETHERNET MACFCR value */
    tmpreg = ETH->MACFCR;
    /* Clear xx bits */
    tmpreg &= MACFCR_CLEAR_MASK;
    tmpreg |= (uint32_t)((ETH_InitStruct->ETH_PauseTime << 16) |
                     ETH_InitStruct->ETH_ZeroQuantaPause |
                     ETH_InitStruct->ETH_PauseLowThreshold |
                     ETH_InitStruct->ETH_UnicastPauseFrameDetect |
                     ETH_InitStruct->ETH_ReceiveFlowControl |
                     ETH_InitStruct->ETH_TransmitFlowControl);
    ETH->MACFCR = (uint32_t)tmpreg;

    ETH->MACVLANTR = (uint32_t)(ETH_InitStruct->ETH_VLANTagComparison |
                               ETH_InitStruct->ETH_VLANTagIdentifier);

    tmpreg = ETH->DMAOMR;
    tmpreg &= DMAOMR_CLEAR_MASK;
    tmpreg |= (uint32_t)(ETH_InitStruct->ETH_DropTCPIPChecksumErrorFrame |
                    ETH_InitStruct->ETH_ReceiveStoreForward |
                    ETH_InitStruct->ETH_FlushReceivedFrame |
                    ETH_InitStruct->ETH_TransmitStoreForward |
                    ETH_InitStruct->ETH_TransmitThresholdControl |
                    ETH_InitStruct->ETH_ForwardErrorFrames |
                    ETH_InitStruct->ETH_ForwardUndersizedGoodFrames |
                    ETH_InitStruct->ETH_ReceiveThresholdControl |
                    ETH_InitStruct->ETH_SecondFrameOperate);
    ETH->DMAOMR = (uint32_t)tmpreg;

    ETH->DMABMR = (uint32_t)(ETH_InitStruct->ETH_AddressAlignedBeats |
                            ETH_InitStruct->ETH_FixedBurst |
                            ETH_InitStruct->ETH_RxDMABurstLength | /* !! if 4xPBL is selected for Tx or Rx it is applied for the other */
                            ETH_InitStruct->ETH_TxDMABurstLength |
                           (ETH_InitStruct->ETH_DescriptorSkipLength << 2) |
                            ETH_InitStruct->ETH_DMAArbitration |
                            ETH_DMABMR_USP);
    /* Reset the physical layer */
    ETH_WritePHYRegister(PHYAddress, PHY_BCR, PHY_Reset);
    return ETH_SUCCESS;
}

/*********************************************************************
 * @fn      ETH_Configuration
 *
 * @brief   Ethernet configure.
 *
 * @return  none
 */
void ETH_Configuration( uint8_t *macAddr )
{
    ETH_InitTypeDef ETH_InitStructure;
    uint16_t timeout = 10000;

    /* Enable Ethernet MAC clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC|RCC_AHBPeriph_ETH_MAC_Tx|RCC_AHBPeriph_ETH_MAC_Rx,ENABLE);

    gPHYAddress = PHY_ADDRESS;
#if( PHY_MODE ==  USE_10M_BASE )
    ETH_SetClock( );
    /* Enable internal 10BASE-T PHY*/
    EXTEN->EXTEN_CTR |= EXTEN_ETH_10M_EN;    /* Enable 10M Ethernet physical layer   */
#elif( PHY_MODE ==  USE_MAC_MII)
    /*  Enable MII GPIO */
    ETH_MIIPinInit( );
#endif
    /* Reset ETHERNET on AHB Bus */
    ETH_DeInit();

    /* Software reset */
    ETH_SoftwareReset();

    /* Wait for software reset */
    do{
        Delay_Us(10);
        if( !--timeout )  break;
    }while(ETH->DMABMR & ETH_DMABMR_SR);

    /* ETHERNET Configuration */
    /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
    ETH_StructInit(&ETH_InitStructure);
    /* Fill ETH_InitStructure parameters */
    /*------------------------   MAC   -----------------------------------*/
    ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
#if( PHY_MODE ==  USE_10M_BASE )
    ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
#else
    ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
#endif
    ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable  ;
    ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
    ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
    ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
    ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Enable;
    ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
    ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Enable;
    ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
    ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
    /*------------------------   DMA   -----------------------------------*/
    /* When we use the Checksum offload feature, we need to enable the Store and Forward mode:
    the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum,
    if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
    ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
    ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
    ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;
    ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Enable;
    ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Enable;
    ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Disable;
    ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
    ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
    ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
    ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
    ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;
    /* Configure Ethernet */
    ETH_RegInit( &ETH_InitStructure, gPHYAddress );
#if( PHY_MODE ==  USE_10M_BASE )
    /* Enable the Ethernet Rx Interrupt */
    ETH_DMAITConfig( ETH_DMA_IT_NIS |\
      ETH_DMA_IT_R |\
      ETH_DMA_IT_T |\
      ETH_DMA_IT_PHYLINK,\
      ENABLE );
#else
    /* Enable the Ethernet Rx Interrupt */
    ETH_DMAITConfig( ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T, ENABLE );
#endif
}

/*********************************************************************
 * @fn      ETH_TxPktChainMode
 *
 * @brief   process net send a Ethernet frame in chain mode.
 *
 * @param   Send length
 *
 * @return  Send status.
 */
uint32_t ETH_TxPktChainMode(uint16_t len, uint32_t *pBuff )
{
    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
    {
        /* Return ERROR: OWN bit set */
        return ETH_ERROR;
    }
    /* Setting the Frame Length: bits[12:0] */
    DMATxDescToSet->ControlBufferSize = (len & ETH_DMATxDesc_TBS1);
    DMATxDescToSet->Buffer1Addr = (uint32_t)pBuff;
    pDMATxSet = DMATxDescToSet;
    /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */
    DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;

    /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
    DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

    /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
    if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET)
    {
        /* Clear TBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_TBUS;
        /* Resume DMA transmission*/
        ETH->DMATPDR = 0;
    }
    /* Update the ETHERNET DMA global Tx descriptor with next Tx descriptor */
    /* Chained Mode */
    /* Selects the next DMA Tx descriptor list for next buffer to send */
    DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);
    /* Return SUCCESS */
    return ETH_SUCCESS;
}

/*********************************************************************
 * @fn      WCHNET_ETHIsr
 *
 * @brief   Ethernet Interrupt Service program
 *
 * @return  none
 */
void WCHNET_ETHIsr(void)
{
    uint32_t int_sta;

    int_sta = ETH->DMASR;
    if( int_sta & ETH_DMA_IT_NIS )
    {
        if( int_sta & ETH_DMA_IT_R )
        {
            if ((int_sta & ETH_DMA_IT_RBU) != (u32)RESET)
            {
                /* Clear RBUS ETHERNET DMA flag */
                ETH->DMASR = ETH_DMA_IT_RBU;

                ((ETH_DMADESCTypeDef *)(((ETH_DMADESCTypeDef *)(ETH->DMACHRDR))->Buffer2NextDescAddr))->Status = ETH_DMARxDesc_OWN;

                /* Resume DMA reception */
                ETH->DMARPDR = 0;
            }
            ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
            /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
            if((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (u32)RESET)
            {
                /***/
            }
            else
            {
                /* Update the ETHERNET DMA global Rx descriptor with next Rx descriptor */
                /* Chained Mode */
                /* Selects the next DMA Rx descriptor list for next buffer to read */
                DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);
            }
        }
        if( int_sta & ETH_DMA_IT_T )
        {
            ETH_DMAClearITPendingBit(ETH_DMA_IT_T);
            if( (pDMATxSet->Status&ETH_DMATxDesc_ES) )
            {
                /***/
            }
        }
        if( int_sta & ETH_DMA_IT_PHYLINK)
        {
            ETH_PHYLink( );
            ETH_DMAClearITPendingBit(ETH_DMA_IT_PHYLINK);
        }
        ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
    }
}

/*********************************************************************
 * @fn      ETH_Init
 *
 * @brief   Ethernet initialization.
 *
 * @return  none
 */
void ETH_Init( uint8_t *macAddr )
{
#if( PHY_MODE ==  USE_10M_BASE )
    ETH_LedConfiguration( );
#endif
    Delay_Ms(100);
    ETH_Configuration( macAddr );
    ETH_DMATxDescChainInit(DMATxDscrTab, MACTxBuf, ETH_TXBUFNB);
    ETH_DMARxDescChainInit(DMARxDscrTab, MACRxBuf, ETH_RXBUFNB);
    pDMARxSet = DMARxDscrTab;
    pDMATxSet = DMATxDscrTab;
    NVIC_EnableIRQ(ETH_IRQn);
}

/*********************************************************************
 * @fn      ETH_LibInit
 *
 * @brief   Ethernet library initialization program
 *
 * @return  command status
 */
uint8_t ETH_LibInit( uint8_t *ip, uint8_t *gwip, uint8_t *mask, uint8_t *macaddr )
{
    uint8_t s;
    struct _WCH_CFG  cfg;

    memset(&cfg,0,sizeof(cfg));
    cfg.TxBufSize = ETH_TX_BUF_SZE;
    cfg.TCPMss   = WCHNET_TCP_MSS;
    cfg.HeapSize = WCHNET_MEM_HEAP_SIZE;
    cfg.ARPTableNum = WCHNET_NUM_ARP_TABLE;
    cfg.MiscConfig0 = WCHNET_MISC_CONFIG0;
    cfg.MiscConfig1 = WCHNET_MISC_CONFIG1;
#if( PHY_MODE ==  USE_10M_BASE )
    cfg.led_link = ETH_LedLinkSet;
    cfg.led_data = ETH_LedDataSet;
#endif
    cfg.net_send = ETH_TxPktChainMode;
    cfg.CheckValid = WCHNET_CFG_VALID;
    s = WCHNET_ConfigLIB(&cfg);
    if( s ){
       return (s);
    }
    s = WCHNET_Init(ip,gwip,mask,macaddr);
    ETH_Init( macaddr );
    return (s);
}

#elif defined(CH32F20x_D8W)	

/*********************************************************************
 * @fn      WritePHYReg
 *
 * @brief   MCU write PHY register.
 *
 * @param   reg_add - PHY address,
 *          reg_val - value you want to write.
 *
 * @return  none
 */
void WritePHYReg(uint8_t reg_add,uint16_t reg_val)
{
    R32_ETH_MIWR = (reg_add & RB_ETH_MIREGADR_MIRDL) | (1<<8) | (reg_val<<16);
}

/*********************************************************************
 * @fn      ReadPHYReg
 *
 * @brief   MCU read PHY register.
 *
 * @param   reg_add - PHY address.
 *
 * @return  value you want to get.
 */
uint16_t ReadPHYReg(uint8_t reg_add)
{
    R8_ETH_MIREGADR = reg_add;                          // write address
    return R16_ETH_MIRD;                                // get data
}

/*********************************************************************
 * @fn      WCHNET_LinkProcess
 *
 * @brief   link process.
 *
 * @param   none.
 *
 * @return  none.
 */
void WCHNET_LinkProcess( void )
{
    uint16_t phy_anlpar, phy_bmcr, phy_bmsr;

    phy_anlpar = ReadPHYReg(PHY_ANLPAR);
    phy_bmsr = ReadPHYReg(PHY_BMSR);

    if( (phy_anlpar&PHY_ANLPAR_SELECTOR_FIELD) )
    {
        if( !(phyLinkStatus&PHY_LINK_WAIT_SUC) )
        {
            if( (phyPN&0x0C) == PHY_PN_SWITCH_P )
            {
                phySucCnt = 0;
                phyLinkCnt = 0;
                phyLinkStatus = PHY_LINK_WAIT_SUC;
            }
            else
            {
                if( !(phyLinkStatus&PHY_LINK_SUC_N) )
                {
                    phyRetryCnt = 0;
                    phyLinkStatus |= PHY_LINK_SUC_N;
                    phyPN &= ~PHY_PN_SWITCH_N;
                    phy_bmcr = ReadPHYReg(PHY_BMCR);
                    phy_bmcr |= 1<<9;
                    WritePHYReg(PHY_BMCR, phy_bmcr);
                    WritePHYReg(PHY_MDIX, phyPN);
                }
                else
                {
                    phySucCnt = 0;
                    phyLinkCnt = 0;
                    phyLinkStatus = PHY_LINK_WAIT_SUC;
                }
            }
        }
        else{
            if((phySucCnt++ == 5) && ((phy_bmsr&(1<<5)) == 0))
            {
                phySucCnt = 0;
                phyRetryCnt = 0;
                phyPNChangeCnt = 0;
                phyLinkStatus = PHY_LINK_INIT;
                phy_bmcr = ReadPHYReg(PHY_BMCR);
                phy_bmcr |= 1<<9;
                WritePHYReg(PHY_BMCR, phy_bmcr);
                if((phyPN&0x0C) == PHY_PN_SWITCH_P)
                {
                    phyPN |= PHY_PN_SWITCH_N;
                }
                else {
                    phyPN &= ~PHY_PN_SWITCH_N;
                }
                WritePHYReg(PHY_MDIX, phyPN);
            }
        }
    }
    else
    {
        if( phyLinkStatus == PHY_LINK_WAIT_SUC )
        {
            if(phyLinkCnt++ == 10)
            {
                phyLinkCnt = 0;
                phyRetryCnt = 0;
                phyPNChangeCnt = 0;
                phyLinkStatus = PHY_LINK_INIT;
            }
        }
        else if(phyLinkStatus == PHY_LINK_INIT)
        {
            if(phyPNChangeCnt++ == 10)
            {
                phyPNChangeCnt = 0;
                phyPN = ReadPHYReg(PHY_MDIX);
                phyPN &= ~0x0c;
                phyPN ^= 0x03;
                WritePHYReg(PHY_MDIX, phyPN);
            }
            else{
                if((phyPN&0x0C) == PHY_PN_SWITCH_P)
                {
                    phyPN |= PHY_PN_SWITCH_N;
                }
                else {
                    phyPN &= ~PHY_PN_SWITCH_N;
                }
                WritePHYReg(PHY_MDIX, phyPN);
            }
        }
        else if(phyLinkStatus == PHY_LINK_SUC_N)
        {
            if((phyPN&0x0C) == PHY_PN_SWITCH_P)
            {
                phyPN |= PHY_PN_SWITCH_N;
                phy_bmcr = ReadPHYReg(PHY_BMCR);
                phy_bmcr |= 1<<9;
                WritePHYReg(PHY_BMCR, phy_bmcr);
                Delay_Us(10);
                WritePHYReg(PHY_MDIX, phyPN);
            }
            else{
                if(phyRetryCnt++ == 15)
                {
                    phyRetryCnt = 0;
                    phyPNChangeCnt = 0;
                    phyLinkStatus = PHY_LINK_INIT;
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      WCHNET_HandlePhyNegotiation
 *
 * @brief   Handle PHY Negotiation.
 *
 * @param   none.
 *
 * @return  none.
 */
void WCHNET_HandlePhyNegotiation(void)
{
    if( !phyStatus )                        /* Handling PHY Negotiation Exceptions */
    {
		if(phyLinkTime > LocalTime)
			phyLinkTime = LocalTime;
        if( LocalTime - phyLinkTime >= PHY_LINK_TASK_PERIOD )  /* 50ms cycle timing call */
        {
            phyLinkTime = LocalTime;
            WCHNET_LinkProcess( );
        }
    }
}

/*********************************************************************
 * @fn      WCHNET_MainTask
 *
 * @brief   library main task function
 *
 * @return  none.
 */
void WCHNET_MainTask(void)
{
    WCHNET_NetInput( );         /* Ethernet data input */
    WCHNET_PeriodicHandle( );   /* Protocol stack time-related task processing */
    WCHNET_HandlePhyNegotiation( );
}

/*********************************************************************
 * @fn      ETH_LedLinkSet
 *
 * @brief   set eth link led,setbit 0 or 1,the link led turn on or turn off
 *
 * @return  none
 */
void ETH_LedLinkSet( uint8_t mode )
{
    if( mode == LED_OFF )
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_0);
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);
    }
}

/*********************************************************************
 * @fn      ETH_LedDataSet
 *
 * @brief   set eth data led,setbit 0 or 1,the data led turn on or turn off
 *
 * @return  none
 */
void ETH_LedDataSet( uint8_t mode )
{
    if( mode == LED_OFF )
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_1);
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);
    }
}

/*********************************************************************
 * @fn      ETH_LedConfiguration
 *
 * @brief   set eth data and link led pin
 *
 * @return  none
 */
void ETH_LedConfiguration(void)
{
    GPIO_InitTypeDef  GPIO={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    GPIO.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO);
    ETH_LedDataSet(LED_OFF);
    ETH_LedLinkSet(LED_OFF);
}

/*********************************************************************
 * @fn      ETH_DMATxDescChainInit
 *
 * @brief   Initializes the DMA Tx descriptors in chain mode.
 *
 * @param   DMATxDescTab - Pointer on the first Tx desc list
 *          TxBuff - Pointer on the first TxBuffer list
 *          TxBuffCount - Number of the used Tx desc in the list
 *
 * @return  none
 */
void ETH_DMATxDescChainInit(ETH_DMADESCTypeDef *DMATxDescTab, uint8_t *TxBuff, uint32_t TxBuffCount)
{
    ETH_DMADESCTypeDef *DMATxDesc;

    DMATxDescToSet = DMATxDescTab;
    DMATxDesc = DMATxDescTab;
    DMATxDesc->Status = 0;
    DMATxDesc->Buffer1Addr = (uint32_t)TxBuff;
    DMATxDesc->Buffer2NextDescAddr = (uint32_t)DMATxDescTab;
}

/*********************************************************************
 * @fn      ETH_DMARxDescChainInit
 *
 * @brief   Initializes the DMA Rx descriptors in chain mode.
 *
 * @param   DMARxDescTab - Pointer on the first Rx desc list.
 *          RxBuff - Pointer on the first RxBuffer list.
 *          RxBuffCount - Number of the used Rx desc in the list.
 *
 * @return  none
 */
void ETH_DMARxDescChainInit(ETH_DMADESCTypeDef *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount)
{
    uint8_t i = 0;
    ETH_DMADESCTypeDef *DMARxDesc;

    DMARxDescToGet = DMARxDescTab;
    for(i = 0; i < RxBuffCount; i++)
    {
        DMARxDesc = DMARxDescTab + i;
        DMARxDesc->Status = ETH_DMARxDesc_OWN;
        DMARxDesc->Buffer1Addr = (uint32_t)(&RxBuff[i * ETH_MAX_PACKET_SIZE]);

        if(i < (RxBuffCount - 1))
        {
            DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab + i + 1);
        }
        else
        {
            DMARxDesc->Buffer2NextDescAddr = (uint32_t)(DMARxDescTab);
        }
    }
}

/*********************************************************************
 * @fn      ETH_Start
 *
 * @brief   Enables ENET MAC and DMA reception/transmission.
 *
 * @return  none
 */
void ETH_Start(void)
{
    R16_ETH_ERXST = DMARxDescToGet->Buffer1Addr;
    R8_ETH_ECON1 |= RB_ETH_ECON1_RXEN;                                //receive enable
    EXTEN->EXTEN_CTR |= EXTEN_ETH_10M_EN;
}

/*********************************************************************
 * @fn      ETH_SetClock
 *
 * @brief   Set ETH Clock(60MHz).
 *
 * @return  none
 */
void ETH_SetClock(void)
{
    /* ETH initialization */
    RCC_ETHDIVConfig(RCC_ETHCLK_Div2);  // 120M/2 = 60MHz
}

/*********************************************************************
 * @fn      ETH_Configuration
 *
 * @brief   Ethernet configure.
 *
 * @return  none
 */
void ETH_Configuration( uint8_t *macAddr )
{
    ETH_SetClock( );
    R8_ETH_EIE = 0;
    R8_ETH_EIE |= RB_ETH_EIE_INTIE |
                  RB_ETH_EIE_RXIE|
                  RB_ETH_EIE_LINKIE|
                  RB_ETH_EIE_TXIE  |
                  RB_ETH_EIE_TXERIE|
                  RB_ETH_EIE_RXERIE;                                    //Turn on all interrupts

    R8_ETH_EIE |= RB_ETH_EIE_R_EN50;                                    //Turn on 50 ohm pull-up

    R8_ETH_EIR = 0xff;                                                  //clear interrupt flag
    R8_ETH_ESTAT |= RB_ETH_ESTAT_INT | RB_ETH_ESTAT_BUFER;              //clear state

    R8_ETH_ECON1 |= (RB_ETH_ECON1_TXRST|RB_ETH_ECON1_RXRST);            //Transceiver module reset
    R8_ETH_ECON1 &= ~(RB_ETH_ECON1_TXRST|RB_ETH_ECON1_RXRST);

    //Filter mode, received packet type
    R8_ETH_ERXFCON = 0;
    R8_ETH_MAADRL1 = macAddr[5];                                        // MAC assignment
    R8_ETH_MAADRL2 = macAddr[4];
    R8_ETH_MAADRL3 = macAddr[3];
    R8_ETH_MAADRL4 = macAddr[2];
    R8_ETH_MAADRL5 = macAddr[1];
    R8_ETH_MAADRL6 = macAddr[0];

    //Filter mode, limit packet type
    R8_ETH_MACON1 |= RB_ETH_MACON1_MARXEN;                              //MAC receive enable
    R8_ETH_MACON2 &= ~RB_ETH_MACON2_PADCFG;
    R8_ETH_MACON2 |= PADCFG_AUTO_3;                                     //All short packets are automatically padded to 60 bytes
    R8_ETH_MACON2 |= RB_ETH_MACON2_TXCRCEN;                             //Hardware padded CRC
    R8_ETH_MACON2 &= ~RB_ETH_MACON2_HFRMEN;                             //Jumbo frames are not received
    R16_ETH_MAMXFL = ETH_MAX_PACKET_SIZE;
}

/*********************************************************************
 * @fn      ETH_TxPktChainMode
 *
 * @brief   MAC send a Ethernet frame in chain mode.
 *
 * @param   Send length
 *
 * @return  Send status.
 */
uint32_t ETH_TxPktChainMode(uint16_t len, uint32_t *pBuff )
{
     /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if( DMATxDescToSet->Status & ETH_DMATxDesc_OWN )
    {
        /* Return ERROR: OWN bit set */
        return ETH_ERROR;
    }
    DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;
    R16_ETH_ETXLN = len;
    R16_ETH_ETXST = (uint32_t)pBuff;
    R8_ETH_ECON1 |= RB_ETH_ECON1_TXRTS;                               //start sending
    /* Update the ETHERNET DMA global Tx descriptor with next Tx descriptor */
    /* Chained Mode */
    /* Selects the next DMA Tx descriptor list for next buffer to send */
    DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);
    /* Return SUCCESS */
    return ETH_SUCCESS;
}

void ETH_PHYLink( void )
{
    uint16_t RegValue;
    uint32_t phy_stat, phy_anlpar;

    phy_anlpar = ReadPHYReg(PHY_ANLPAR);
    phy_stat = ReadPHYReg(PHY_BMSR);                            //Read PHY Status Register

    if((phy_stat&(PHY_Linked_Status))&&(phy_anlpar == 0)){      //restart negotiation
        RegValue = ReadPHYReg(PHY_BMCR);
        RegValue |= PHY_Reset;
        WritePHYReg(PHY_BMCR, RegValue);
        EXTEN->EXTEN_CTR &= ~EXTEN_ETH_10M_EN;
        Delay_Ms(500);
        EXTEN->EXTEN_CTR |= EXTEN_ETH_10M_EN;
		PHY_NEGOTIATION_PARAM_INIT();
        return;
    }
    WCHNET_PhyStatus( phy_stat );

    if( (phy_stat&(PHY_Linked_Status)) && (phy_stat&PHY_AutoNego_Complete) )
    {
        if( phy_anlpar&(1<<6) )
        {
            R8_ETH_MACON2 |= RB_ETH_MACON2_FULDPX;
        }
        else
        {
            R8_ETH_MACON2 &= ~RB_ETH_MACON2_FULDPX;
        }
        phyStatus = PHY_Linked_Status;
    }
    else
    {
		PHY_NEGOTIATION_PARAM_INIT();
    }
}

/*********************************************************************
 * @fn      WCHNET_ETHIsr
 *
 * @brief
 *
 * @return  none
 */
void WCHNET_ETHIsr( void )
{
    uint8_t eth_irq_flag, estat_regval;

    eth_irq_flag = R8_ETH_EIR;
    if(eth_irq_flag&RB_ETH_EIR_RXIF)                                //Receive complete
    {
        R8_ETH_EIR = RB_ETH_EIR_RXIF;
        /* Check if the descriptor is owned by the ETHERNET DMA */
        if( DMARxDescToGet->Status & ETH_DMARxDesc_OWN )
        {
            estat_regval = R8_ETH_ESTAT;
            if(estat_regval & \
                    (RB_ETH_ESTAT_BUFER | RB_ETH_ESTAT_RXCRCER | RB_ETH_ESTAT_RXNIBBLE | RB_ETH_ESTAT_RXMORE))
            {
                return;
            }
            if( ((ETH_DMADESCTypeDef*)(DMARxDescToGet->Buffer2NextDescAddr))->Status& ETH_DMARxDesc_OWN )
            {
                DMARxDescToGet->Status &= ~ETH_DMARxDesc_OWN;
                DMARxDescToGet->Status &= ~ETH_DMARxDesc_ES;
                DMARxDescToGet->Status |= (ETH_DMARxDesc_FS|ETH_DMARxDesc_LS);
                DMARxDescToGet->Status &= ~ETH_DMARxDesc_FL;
                DMARxDescToGet->Status |= ((R16_ETH_ERXLN+4)<<ETH_DMARxDesc_FrameLengthShift);
                /* Update the ETHERNET DMA global Rx descriptor with next Rx descriptor */
                /* Selects the next DMA Rx descriptor list for next buffer to read */
                DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);
                R16_ETH_ERXST = DMARxDescToGet->Buffer1Addr;
            }
        }
    }
    if(eth_irq_flag&RB_ETH_EIR_TXIF)                                //send completed
    {
        DMATxDescToSet->Status &= ~ETH_DMATxDesc_OWN;
        R8_ETH_EIR = RB_ETH_EIR_TXIF;
    }
    if(eth_irq_flag&RB_ETH_EIR_LINKIF)                              //Link change
    {
        ETH_PHYLink();
        R8_ETH_EIR = RB_ETH_EIR_LINKIF;
    }
    if(eth_irq_flag&RB_ETH_EIR_TXERIF)                              //send error
    {
        DMATxDescToSet->Status &= ~ETH_DMATxDesc_OWN;
        R8_ETH_EIR = RB_ETH_EIR_TXERIF;
    }
    if(eth_irq_flag&RB_ETH_EIR_RXERIF)                              //receive error
    {
        R8_ETH_EIR = RB_ETH_EIR_RXERIF;
    }
}

/*********************************************************************
 * @fn      ETH_Init
 *
 * @brief   Ethernet initialization.
 *
 * @return  none
 */
void ETH_Init( uint8_t *macAddr )
{
    ETH_LedConfiguration( );
    Delay_Ms(100);
    ETH_Configuration( macAddr );
    ETH_DMATxDescChainInit(DMATxDscrTab, MACTxBuf, ETH_TXBUFNB);
    ETH_DMARxDescChainInit(DMARxDscrTab, MACRxBuf, ETH_RXBUFNB);
    pDMARxSet = DMARxDscrTab;
    ETH_Start( );
    NVIC_EnableIRQ(ETH_IRQn);
}

/*********************************************************************
 * @fn      ETH_LibInit
 *
 * @brief   Ethernet library initialization program
 *
 * @return  command status
 */
uint8_t ETH_LibInit( uint8_t *ip, uint8_t *gwip, uint8_t *mask, uint8_t *macaddr )
{
    uint8_t s;
    struct _WCH_CFG  cfg;

    memset(&cfg,0,sizeof(cfg));
    cfg.TxBufSize = ETH_TX_BUF_SZE;
    cfg.TCPMss   = WCHNET_TCP_MSS;
    cfg.HeapSize = WCHNET_MEM_HEAP_SIZE;
    cfg.ARPTableNum = WCHNET_NUM_ARP_TABLE;
    cfg.MiscConfig0 = WCHNET_MISC_CONFIG0;
    cfg.MiscConfig1 = WCHNET_MISC_CONFIG1;
    cfg.led_link = ETH_LedLinkSet;
    cfg.led_data = ETH_LedDataSet;
    cfg.net_send = ETH_TxPktChainMode;
    cfg.CheckValid = WCHNET_CFG_VALID;
    s = WCHNET_ConfigLIB(&cfg);
    if(s){
       return (s);
    }
    s = WCHNET_Init(ip,gwip,mask,macaddr);
    ETH_Init(macaddr);
    return (s);
}

#endif

/******************************** endfile @ eth_driver ******************************/
