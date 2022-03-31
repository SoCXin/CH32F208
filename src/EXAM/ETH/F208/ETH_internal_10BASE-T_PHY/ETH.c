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

__attribute__( ( aligned( 4 ) ) )  uint8_t  MACRxBuf[RX_QUEUE_NUM][RX_BUF_SIZE];                    /* MAC���ջ�������4�ֽڶ��� */
__attribute__( ( aligned( 4 ) ) )  uint8_t  MACTxBuf[TX_QUEUE_NUM][TX_BUF_SIZE];                    /* MAC���ͻ�������4�ֽڶ��� */

RXBUFST RxCtrl;   //���չ������
TXBUFST TXCtrl;   //���͹������

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
    RxCtrl.RecvEn = 0;                                                     //���ͽ���ʹ�ܹر�
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
                  RB_ETH_EIE_RXERIE;                                            //���������ж�

    R8_ETH_EIE |= RB_ETH_EIE_R_EN50;                                            //����50ŷ����

    R8_ETH_EIR = 0xff;               					                        //����жϱ�־
    R8_ETH_ESTAT |= RB_ETH_ESTAT_INT | RB_ETH_ESTAT_BUFER;                      //���״̬

    R8_ETH_ECON2 &= ~0x0F;
    R8_ETH_ECON2 |= ( 2 << 1 );

    R8_ETH_ECON1 |= ( RB_ETH_ECON1_TXRST | RB_ETH_ECON1_RXRST );                //�շ�ģ�鸴λ
    R8_ETH_ECON1 &= ~( RB_ETH_ECON1_TXRST | RB_ETH_ECON1_RXRST );

    //����ģʽ�����հ�����
    R8_ETH_ERXFCON = 0;

    //����ģʽ�����ư�����
    R8_ETH_MACON1 |= RB_ETH_MACON1_MARXEN;       		                        //MAC����ʹ��
    R8_ETH_MACON2 &= ~RB_ETH_MACON2_PADCFG;
    R8_ETH_MACON2 |= PADCFG_AUTO_3;                                             //���ж̰��Զ���䵽60
    R8_ETH_MACON2 |= RB_ETH_MACON2_TXCRCEN;                                     //Ӳ�����CRC
    R8_ETH_MACON2 &= ~RB_ETH_MACON2_HFRMEN;                                     //�����վ���֡

    R8_ETH_MACON2 |= RB_ETH_MACON2_FULDPX;                                      //ȫ˫��
    R16_ETH_MAMXFL = MAC_MAX_LEN;

    R8_ETH_MAADRL1 = 0x84;                                                 //MAC��ֵ
    R8_ETH_MAADRL2 = 0xc2;
    R8_ETH_MAADRL3 = 0xe4;
    R8_ETH_MAADRL4 = 0x01;
    R8_ETH_MAADRL4 = 0x02;
    R8_ETH_MAADRL5 = 0x03;

    R16_ETH_ERXST = ( uint16_t )RxCtrl.RxBufAddr[RxCtrl.RecvIndex];   //��ǰ���ջ���

    R8_ETH_ECON1 |= RB_ETH_ECON1_RXEN;                                      //����ʹ��
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

    //Ӳ���Զ�����
    rec_len = R16_ETH_ERXLN;
    if( RxCtrl.RemainCount < ( RX_QUEUE_NUM - 1 ) )                        //�ж��п��У��������һ����㸲�ǣ�Ҳʹ����������������
    {
        RxCtrl.RxBufStat[RxCtrl.RecvIndex] = 1;
        RxCtrl.RxBufLen[RxCtrl.RecvIndex] = rec_len;
        RxCtrl.RemainCount++;
        RxCtrl.RecvIndex++;
        if( RxCtrl.RecvIndex >= RX_QUEUE_NUM )
        {
            RxCtrl.RecvIndex = 0;
        }
        R16_ETH_ERXST = ( uint16_t )RxCtrl.RxBufAddr[RxCtrl.RecvIndex]; //������һ�����յ�ַ
    }
    else                                                                        //�������µİ�
    {
        RxCtrl.RxBufStat[RxCtrl.RecvIndex] = 1;
        RxCtrl.RxBufLen[RxCtrl.RecvIndex] = rec_len;
        /*
        ���ж��ж�������ʱ�����һ�������ǲ��õģ�ֱ��һ���µĿ��ж��в����󣬲�����һ���µ����ݰ��Ż��ȡ������ݡ�
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
    if( TXCtrl.TxQueueCnt )                                               //�����ﻹ�����ݰ�û�з���
    {
        R16_ETH_ETXLN = TXCtrl.TxBufLen[TXCtrl.SendIndex];
        R16_ETH_ETXST = TXCtrl.TxBufAddr[TXCtrl.SendIndex];
        TXCtrl.TxBufStat[TXCtrl.SendIndex] = 0;                     //����
        R8_ETH_ECON1 |= RB_ETH_ECON1_TXRTS;                                     //��ʼ����
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
    if( link_sta )                                                              //����
    {
        TXCtrl.SendEn = 1;

#if 0
        //����ARPӦ��
        printf( "send \n" );
        s = ETHTestSend( test_buf1, 42 );
        printf( "s=%02x\n", ( uint16_t )s );
#endif


    }
    else		                                                                //�Ͽ�
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

    if( eth_irq_flag & RB_ETH_EIR_RXIF )                                        //�������
    {
        ETH_IRQ_Rec_Deal();                                                     //���մ���
        R8_ETH_EIR = RB_ETH_EIR_RXIF;
    }

    if( eth_irq_flag & RB_ETH_EIR_TXIF )                                        //�������
    {
        ETH_IRQ_Send_Deal();                                                    //���ʹ���
        R8_ETH_EIR = RB_ETH_EIR_TXIF;
    }

    if( eth_irq_flag & RB_ETH_EIR_LINKIF )                                      //Link �仯
    {
        uint16_t phy_reg;

        printf( "link\n" );
        phy_reg = ReadPHYReg( PHY_BMSR );                                       //��ȡPHY״̬�Ĵ���
        if( phy_reg & 0x04 )
        {
            link_sta = 1;    //link status
        }
        else
        {
            link_sta = 0;
        }
        ETH_IRQ_LinkChange_Deal( link_sta );                                    //���������¼�
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

    if( eth_irq_flag & ( RB_ETH_EIR_TXERIF | RB_ETH_EIR_RXERIF ) )              //�շ�����
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
    NVIC_DisableIRQ( ETH_IRQn ); //R8_ETH_EIE &= ~RB_ETH_EIE_INTIE;    //���ж�

    R16_ETH_ETXLN = len;
    R16_ETH_ETXST = ( uint16_t )TXCtrl.TxBufAddr[TXCtrl.SendIndex]; //д���ַ
    R8_ETH_ECON1 |= RB_ETH_ECON1_TXRTS;                                     //��ʼ����

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
    R8_ETH_EIE &= ~RB_ETH_EIE_INTIE;                                            //���ж�
    RxCtrl.RemainCount--;
    NVIC_EnableIRQ( ETH_IRQn );
    R8_ETH_EIE |= RB_ETH_EIE_INTIE;

    return rec_len;
}


