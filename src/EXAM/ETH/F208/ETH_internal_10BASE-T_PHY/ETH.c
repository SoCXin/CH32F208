/********************************** (C) COPYRIGHT *******************************
* File Name          : ETH.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/08/08
* Description        : Ethernet driver..
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "debug.h"
#include "eth.h"
#include "string.h"

__attribute__( ( aligned( 4 ) ) )  uint8_t  MACRxBuf[RX_QUEUE_NUM][RX_BUF_SIZE];                    /* MAC接收缓冲区，4字节对齐 */
__attribute__( ( aligned( 4 ) ) )  uint8_t  MACTxBuf[TX_QUEUE_NUM][TX_BUF_SIZE];                    /* MAC发送缓冲区，4字节对齐 */

RXBUFST RxCtrl;   //接收管理参数
TXBUFST TXCtrl;   //发送管理参数

uint8_t  link_sta = 0;
uint8_t  MACAddr[MACADDR_LEN] = {0x84, 0xc2, 0xe4, 0x02, 0x03, 0x04};


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
void WritePHYReg( uint8_t reg_add, uint16_t reg_val )
{
    uint8_t reg_op = 0;

    R16_ETH_MIWR = reg_val;
    reg_op = ( reg_add & RB_ETH_MIREGADR_MIRDL );
    R8_ETH_MIREGADR = reg_op;
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
uint16_t ReadPHYReg( uint8_t reg_add )
{
    uint8_t reg_op = 0;
    uint16_t read_reg_val = 0xffff;

    reg_op = reg_add & RB_ETH_MIREGADR_MIRDL;
    R8_ETH_MIREGADR = RB_ETH_MIREGADR_MIIWR | reg_op;
    read_reg_val = R16_ETH_MIRD;

    return read_reg_val;
}

/*********************************************************************
 * @fn      ETHParaInit
 *
 * @brief   MAC work process parameters initialize.
 *
 * @param   reg_add - PHY address
 *
 * @return  none
 */
void ETHParaInit( void )
{
    uint8_t i;

    memset( ( char * )&MACRxBuf[0][0], 0, sizeof( MACRxBuf ) );
    memset( ( char * )&MACTxBuf[0][0], 0, sizeof( MACTxBuf ) );
    memset( ( char * )&RxCtrl, 0, sizeof( RxCtrl ) );
    memset( ( char * )&TXCtrl, 0, sizeof( TXCtrl ) );
    for( i = 0; i < RX_QUEUE_NUM; i++ ){
        RxCtrl.RxBufAddr[i] = ( uint32_t )( &MACRxBuf[i][0] );
    }
    for( i = 0; i < TX_QUEUE_NUM; i++ ) {
        TXCtrl.TxBufAddr[i] = ( uint32_t )( &MACTxBuf[i][0] );
    }
    RxCtrl.RecvEn = 0;                                                     //发送接收使能关闭
    TXCtrl.SendEn = 0;

}

/*********************************************************************
 * @fn      ETHInit
 *
 * @brief   ETH initialize.
 *
 * @param   LenthControl - MAC frame length,
 *
 * @return  none
 */
void ETHInit( uint16_t LenthControl )
{
    ETHParaInit();

    R8_ETH_EIE = 0;
    R8_ETH_EIE |= RB_ETH_EIE_INTIE |
                  RB_ETH_EIE_RXIE |
                  RB_ETH_EIE_LINKIE |
                  RB_ETH_EIE_TXIE  |
                  RB_ETH_EIE_TXERIE |
                  RB_ETH_EIE_RXERIE;                                            //开启所有中断

    R8_ETH_EIE |= RB_ETH_EIE_R_EN50;                                            //开启50欧上拉

    R8_ETH_EIR = 0xff;               					                        //清除中断标志
    R8_ETH_ESTAT |= RB_ETH_ESTAT_INT | RB_ETH_ESTAT_BUFER;                      //清除状态

    R8_ETH_ECON2 &= ~0x0F;
    R8_ETH_ECON2 |= ( 2 << 1 );

    R8_ETH_ECON1 |= ( RB_ETH_ECON1_TXRST | RB_ETH_ECON1_RXRST );                //收发模块复位
    R8_ETH_ECON1 &= ~( RB_ETH_ECON1_TXRST | RB_ETH_ECON1_RXRST );

    //过滤模式，接收包类型
    R8_ETH_ERXFCON = 0;

    //过滤模式，限制包类型
    R8_ETH_MACON1 |= RB_ETH_MACON1_MARXEN;       		                        //MAC接收使能
    R8_ETH_MACON2 &= ~RB_ETH_MACON2_PADCFG;
    R8_ETH_MACON2 |= PADCFG_AUTO_3;                                             //所有短包自动填充到60
    R8_ETH_MACON2 |= RB_ETH_MACON2_TXCRCEN;                                     //硬件填充CRC
    R8_ETH_MACON2 &= ~RB_ETH_MACON2_HFRMEN;                                     //不接收巨型帧

    R8_ETH_MACON2 |= RB_ETH_MACON2_FULDPX;                                      //全双工
    R16_ETH_MAMXFL = MAC_MAX_LEN;

    R8_ETH_MAADRL1 = 0x84;                                                 //MAC赋值
    R8_ETH_MAADRL2 = 0xc2;
    R8_ETH_MAADRL3 = 0xe4;
    R8_ETH_MAADRL4 = 0x01;
    R8_ETH_MAADRL4 = 0x02;
    R8_ETH_MAADRL5 = 0x03;

    R16_ETH_ERXST = ( uint16_t )RxCtrl.RxBufAddr[RxCtrl.RecvIndex];   //当前接收缓存

    R8_ETH_ECON1 |= RB_ETH_ECON1_RXEN;                                      //接收使能
}

/*********************************************************************
 * @fn      ETH_IRQ_Rec_Deal
 *
 * @brief   ETH receive interrupt deal program.
 *
 * @return  none
 */
void ETH_IRQ_Rec_Deal( void )
{
    uint16_t rec_len;

    //硬件自动覆盖
    rec_len = R16_ETH_ERXLN;
    if( RxCtrl.RemainCount < ( RX_QUEUE_NUM - 1 ) )                        //有队列空闲，保留最后一个随便覆盖，也使用这个队列里的数据
    {
        RxCtrl.RxBufStat[RxCtrl.RecvIndex] = 1;
        RxCtrl.RxBufLen[RxCtrl.RecvIndex] = rec_len;
        RxCtrl.RemainCount++;
        RxCtrl.RecvIndex++;
        if( RxCtrl.RecvIndex >= RX_QUEUE_NUM )
        {
            RxCtrl.RecvIndex = 0;
        }
        R16_ETH_ERXST = ( uint16_t )RxCtrl.RxBufAddr[RxCtrl.RecvIndex]; //更新下一个接收地址
    }
    else                                                                        //覆盖最新的包
    {
        RxCtrl.RxBufStat[RxCtrl.RecvIndex] = 1;
        RxCtrl.RxBufLen[RxCtrl.RecvIndex] = rec_len;
        /*
        所有队列都有数据时，最后一个队列是不用的，直到一个新的空闲队列产生后，并接收一个新的数据包才会读取这个数据。
        */
    }
}

/*********************************************************************
 * @fn      ETH_IRQ_Send_Deal
 *
 * @brief   ETH transmit interrupt deal program.
 *
 * @return  none
 */
void ETH_IRQ_Send_Deal( void )
{
    if( TXCtrl.TxQueueCnt )                                               //队列里还有数据包没有发送
    {
        R16_ETH_ETXLN = TXCtrl.TxBufLen[TXCtrl.SendIndex];
        R16_ETH_ETXST = TXCtrl.TxBufAddr[TXCtrl.SendIndex];
        TXCtrl.TxBufStat[TXCtrl.SendIndex] = 0;                     //空闲
        R8_ETH_ECON1 |= RB_ETH_ECON1_TXRTS;                                     //开始发送
        TXCtrl.SendIndex++;
        if( TXCtrl.SendIndex >= TX_QUEUE_NUM )
        {
            TXCtrl.SendIndex = 0;
        }
        TXCtrl.TxQueueCnt--;
    }
}

/*********************************************************************
 * @fn      ETH_IRQ_LinkChange_Deal
 *
 * @brief   ETH link-status change interrupt deal program.
 *
 * @param   ink-status.
 *
 * @return  none
 */
void ETH_IRQ_LinkChange_Deal( uint8_t link_sta )
{
#if 0
    uint8_t s;
#endif
    if( link_sta )                                                              //连接
    {
        TXCtrl.SendEn = 1;

#if 0
        //发送ARP应答
        printf( "send \n" );
        s = ETHTestSend( test_buf1, 42 );
        printf( "s=%02x\n", ( uint16_t )s );
#endif


    }
    else		                                                                //断开
    {
        TXCtrl.SendEn = 0;
    }
}

/*********************************************************************
 * @fn      ETH_IRQ_Deal
 *
 * @brief   ETH link-status change interrupt deal program.
 *
 * @return  none
 */
void ETH_IRQ_Deal( void )
{
    uint8_t eth_irq_flag;

    eth_irq_flag = R8_ETH_EIR;

    if( eth_irq_flag & RB_ETH_EIR_RXIF )                                        //接收完成
    {
        ETH_IRQ_Rec_Deal();                                                     //接收处理
        R8_ETH_EIR = RB_ETH_EIR_RXIF;
    }

    if( eth_irq_flag & RB_ETH_EIR_TXIF )                                        //发送完成
    {
        ETH_IRQ_Send_Deal();                                                    //发送处理
        R8_ETH_EIR = RB_ETH_EIR_TXIF;
    }

    if( eth_irq_flag & RB_ETH_EIR_LINKIF )                                      //Link 变化
    {
        uint16_t phy_reg;

        printf( "link\n" );
        phy_reg = ReadPHYReg( PHY_BMSR );                                       //读取PHY状态寄存器
        if( phy_reg & 0x04 )
        {
            link_sta = 1;    //link status
        }
        else
        {
            link_sta = 0;
        }
        ETH_IRQ_LinkChange_Deal( link_sta );                                    //处理连接事件
        printf( "link_sta:%x\r\n", link_sta );
        R8_ETH_EIR = RB_ETH_EIR_LINKIF;
        if( link_sta )
        {
            GPIOA->OUTDR &= ~( 1 << 0 );
            printf( "TIM3 CNT:%d ms.\n", TIM3->CNT );
            TIM_Cmd( TIM3, DISABLE );
            TIM3->CNT = 0;
        }
    }

    if( eth_irq_flag & ( RB_ETH_EIR_TXERIF | RB_ETH_EIR_RXERIF ) )              //收发出错
    {
        R8_ETH_EIR = ( RB_ETH_EIR_TXERIF | RB_ETH_EIR_RXERIF );
    }
}

/*********************************************************************
 * @fn      ETHSend
 *
 * @brief   ETH transmit.
 *
 * @param   p_send_buf - transmit pointer.
 *          send_len - transmit length.
 *
 * @return  none
 */
void ETHSend( uint8_t *p_send_buf, uint16_t send_len )
{
    uint16_t len, i;
    uint8_t *p_data, *p_tx_buf;

    len = send_len;
    p_data = p_send_buf;
    p_tx_buf = ( uint8_t * )TXCtrl.TxBufAddr[TXCtrl.SendIndex];
    for( i = 0; i < len; i++ )
    {
        *p_tx_buf++ = *p_data++;
    }
    NVIC_DisableIRQ( ETH_IRQn ); //R8_ETH_EIE &= ~RB_ETH_EIE_INTIE;    //关中断

    R16_ETH_ETXLN = len;
    R16_ETH_ETXST = ( uint16_t )TXCtrl.TxBufAddr[TXCtrl.SendIndex]; //写入地址
    R8_ETH_ECON1 |= RB_ETH_ECON1_TXRTS;                                     //开始发送

    NVIC_EnableIRQ( ETH_IRQn );
}

/*********************************************************************
 * @fn      ETHRec
 *
 * @brief   ETH transmit.
 *
 * @param   p_rec_buf - receive pointer.
 *
 * @return  receive length.
 */
uint16_t ETHRec( uint8_t *p_rec_buf )
{
    uint16_t rec_len = 0;
    uint16_t i;
    uint8_t  *p_data;
    uint8_t  *p_rx_buf;

    if( !RxCtrl.RemainCount )
    {
        return 0;
    }

    p_data = p_rec_buf;
    p_rx_buf = ( uint8_t * )RxCtrl.RxBufAddr[RxCtrl.ReadIndex];
    rec_len = RxCtrl.RxBufLen[RxCtrl.ReadIndex];
    for( i = 0; i < rec_len; i++ )
    {
        *( p_data + i ) = *( p_rx_buf + i );
    }

    RxCtrl.RxBufStat[RxCtrl.ReadIndex] = RX_BUF_IDLE;
    RxCtrl.ReadIndex++;
    if( RxCtrl.ReadIndex >= RX_QUEUE_NUM )
    {
        RxCtrl.ReadIndex = 0;
    }

    NVIC_DisableIRQ( ETH_IRQn );
    R8_ETH_EIE &= ~RB_ETH_EIE_INTIE;                                            //关中断
    RxCtrl.RemainCount--;
    NVIC_EnableIRQ( ETH_IRQn );
    R8_ETH_EIE |= RB_ETH_EIE_INTIE;

    return rec_len;
}


