/********************************** (C) COPYRIGHT *******************************
* File Name          : ETH.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Ethernet definition.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef __ETH_H__
#define __ETH_H__

#include "debug.h"

//�����շ�MAC������ 1536
#define RX_BUF_SIZE             1536                                            /* ���ջ����С */
#define TX_BUF_SIZE             1536                                            /* ���ͻ����С */
#define ETHER_HEAD_LEN          14                                              /* ����֡ͷ���� */
#define MACADDR_LEN             6
#define MAC_MAX_LEN             1536                                            /* MAC���������� */
#define RX_QUEUE_NUM            4                                               /* ���ջ����������� */
#define TX_QUEUE_NUM            2                                               /* ���ͻ����������� */

//�շ�������
typedef struct _RXBUFST
{
    uint8_t  RecvEn;                                                            /* ʹ�ܿ��� */
    uint8_t  RecvIndex;                                                         /* MAC����ջ����������� */
    uint8_t  ReadIndex;                                                         /* ��ȡ������������ֵ */
    uint8_t  RemainCount;                                                       /* ʣ��δ��ȡ�����ݰ� */
    uint8_t  RxBufStat[RX_QUEUE_NUM];                                           /* ���ջ�������״̬ */
    uint16_t RxBufLen[RX_QUEUE_NUM];                                            /* ���ջ��������ݵ���Ч���� */
    uint32_t RxBufAddr[RX_QUEUE_NUM];                                           /* ���ջ���������ʼ��ַ */
}RXBUFST;

typedef struct _TXBUFST
{
    uint8_t  SendEn;                                                            /* ʹ�ܿ��� */
    uint8_t  SendIndex;                                                         /* MAC�㷢�ͻ����������� */
    uint8_t  WriteIndex;                                                        /* ���ͻ��������е�����ֵ */
    uint8_t  TxQueueCnt;                                                        /* ���Ͷ������Ŷӵ����� */
    uint8_t  TxBufStat[TX_QUEUE_NUM];                                           /* ���ͻ�������״̬ */
    uint16_t TxBufLen[TX_QUEUE_NUM];                                            /* ���ͻ��������ݵ���Ч���� */
    uint32_t TxBufAddr[TX_QUEUE_NUM];                                          /* ���ͻ���������ʼ��ַ */
}TXBUFST;

/* ����/���ȫ���ж� */
#define SET_GOLB_INT(a,b)        (a |= b)
#define RESET_GOLB_INT(a,b)      (a &= ~b)

/* ����/���socket�ж� */
#define SET_SOCKET_INT(a,b)      ((a->IntStatus)|= b)
#define RESET_SOCKET_INT(a,b)    ((a->IntStatus)&= ~b)
#define ETH_BASE_ADDR  0x40028000 /* ��̫������ַ */

typedef struct
{
  __IO uint8_t reserved1;
  __IO uint8_t reserved2;
  __IO uint8_t reserved3;
  __IO uint8_t EIE;

  __IO uint8_t EIR;
  __IO uint8_t ESTAT;
  __IO uint8_t ECON2;
  __IO uint8_t ECON1;

  __IO uint16_t ETXST;
  __IO uint16_t ETXLN;

  __IO uint16_t ERXST;
  __IO uint16_t ERXLN;

  __IO uint32_t HTL;
  __IO uint32_t HTH;

  __IO uint8_t ERXFON;
  __IO uint8_t MACON1;
  __IO uint8_t MACON2;
  __IO uint8_t MABBIPG;

  __IO uint16_t EPAUS;
  __IO uint16_t MAMXFL;

  __IO uint16_t MIRD;
  __IO uint16_t reserved4;

  __IO uint8_t MIERGADR;
  __IO uint8_t MISTAT;
  __IO uint16_t MIWR;

  __IO uint32_t MAADRL;

  __IO uint16_t MAADRH;
  __IO uint16_t reserved5;
} ETH10M_TypeDef;

#define ETH10               ((ETH10M_TypeDef *) ETH_BASE_ADDR)

/* ETH register */
#define R8_ETH_EIE              (*((volatile uint8_t *)(ETH_BASE_ADDR+3))) /* �ж�ʹ�ܼĴ��� */
#define  RB_ETH_EIE_INTIE       0x80                  /* RW �ж�ʹ�� */
#define  RB_ETH_EIE_RXIE        0x40                  /* RW ��������ж�ʹ�� */
#define  RB_ETH_EIE_LINKIE      0x10                  /* RW Link �仯�ж�ʹ�� */
#define  RB_ETH_EIE_TXIE        0x08                  /* RW ��������ж�ʹ�� */
#define  RB_ETH_EIE_R_EN50      0x04                  /* RW TX 50��������ڡ�1��Ƭ�� 50������ 0��Ƭ�� 50���Ͽ� */
#define  RB_ETH_EIE_TXERIE      0x02                  /* RW ���ʹ����ж�ʹ�� */
#define  RB_ETH_EIE_RXERIE      0x01                  /* RW1 ���մ����־ */
//#define R32_ETH_CON             (*((volatile uint32_t *)(ETH_BASE_ADDR+4)))
#define R8_ETH_EIR              (*((volatile uint8_t *)(ETH_BASE_ADDR+4))) /* �жϱ�־�Ĵ��� */
#define  RB_ETH_EIR_RXIF        0x40                  /* RW1 ������ɱ�־ */
#define  RB_ETH_EIR_LINKIF      0x10                  /* RW1 Link �仯��־ */
#define  RB_ETH_EIR_TXIF        0x08                  /* RW1 ������ɱ�־ */
#define  RB_ETH_EIR_TXERIF      0x02                  /* RW1 ���ʹ����־ */
#define  RB_ETH_EIR_RXERIF      0x01
#define R8_ETH_ESTAT            (*((volatile uint8_t *)(ETH_BASE_ADDR+5))) /* ״̬�Ĵ��� */
#define  RB_ETH_ESTAT_INT       0x80                  /* RW1 �ж� */
#define  RB_ETH_ESTAT_BUFER     0x40                  /* RW1 Buffer ���������� mcu ��Ƶ̫�ͲŻᷢ�� */
#define  RB_ETH_ESTAT_RXCRCER   0x20                  /* RO ���� crc ���� */
#define  RB_ETH_ESTAT_RXNIBBLE  0x10                  /* RO ���� nibble ���� */
#define  RB_ETH_ESTAT_RXMORE    0x08                  /* RO ���ճ���������ݰ� */
#define  RB_ETH_ESTAT_RXBUSY    0x04                  /* RO ���ս����� */
#define  RB_ETH_ESTAT_TXABRT    0x02                  /* RO ���ͱ� mcu ��� */
#define R8_ETH_ECON2            (*((volatile uint8_t *)(ETH_BASE_ADDR+6))) /* ETH PHYģ��ģ����ƼĴ��� */
#define  RB_ETH_ECON2_RX        0x0E                  /* ����д��011 */
#define  RB_ETH_ECON2_TX        0x01
#define  RB_ETH_ECON2_MUST      0x06                  /* ����д��011 */
#define R8_ETH_ECON1            (*((volatile uint8_t *)(ETH_BASE_ADDR+7))) /* �շ����ƼĴ��� */
#define  RB_ETH_ECON1_TXRST     0x80                  /* RW ����ģ�鸴λ */
#define  RB_ETH_ECON1_RXRST     0x40                  /* RW ����ģ�鸴λ */
#define  RB_ETH_ECON1_TXRTS     0x08                  /* RW ���Ϳ�ʼ��������ɺ��Զ����㣬�����������ʹ���ʹ����־TXERIF��TXABRT��1 */
#define  RB_ETH_ECON1_RXEN      0x04                  /* RW ����ʹ�ܣ�����ʱ�����ڽ���������־RXERIF��1 */

#define R32_ETH_TX              (*((volatile uint32_t *)(ETH_BASE_ADDR+8))) /* ���Ϳ��� */
#define R16_ETH_ETXST           (*((volatile uint16_t *)(ETH_BASE_ADDR+8))) /* RW ���� DMA ��������ʼ��ַ */
#define R16_ETH_ETXLN           (*((volatile uint16_t *)(ETH_BASE_ADDR+0xA))) /* RW ���ͳ��� */
#define R32_ETH_RX              (*((volatile uint32_t *)(ETH_BASE_ADDR+0xC))) /* ���տ��� */
#define R16_ETH_ERXST           (*((volatile uint16_t *)(ETH_BASE_ADDR+0xC))) /* RW ���� DMA ��������ʼ��ַ */
#define R16_ETH_ERXLN           (*((volatile uint16_t *)(ETH_BASE_ADDR+0xE))) /* RO ���ճ��� */

#define R32_ETH_HTL             (*((volatile uint32_t *)(ETH_BASE_ADDR+0x10)))
#define R8_ETH_EHT0             (*((volatile uint8_t *)(ETH_BASE_ADDR+0x10))) /* RW Hash Table Byte0 */
#define R8_ETH_EHT1             (*((volatile uint8_t *)(ETH_BASE_ADDR+0x11))) /* RW Hash Table Byte1 */
#define R8_ETH_EHT2             (*((volatile uint8_t *)(ETH_BASE_ADDR+0x12))) /* RW Hash Table Byte2 */
#define R8_ETH_EHT3             (*((volatile uint8_t *)(ETH_BASE_ADDR+0x13))) /* RW Hash Table Byte3 */
#define R32_ETH_HTH             (*((volatile uint32_t *)(ETH_BASE_ADDR+0x14)))
#define R8_ETH_EHT4             (*((volatile uint8_t *)(ETH_BASE_ADDR+0x14))) /* RW Hash Table Byte4 */
#define R8_ETH_EHT5             (*((volatile uint8_t *)(ETH_BASE_ADDR+0x15))) /* RW Hash Table Byte5 */
#define R8_ETH_EHT6             (*((volatile uint8_t *)(ETH_BASE_ADDR+0x16))) /* RW Hash Table Byte6 */
#define R8_ETH_EHT7             (*((volatile uint8_t *)(ETH_BASE_ADDR+0x17))) /* RW Hash Table Byte7 */

#define R32_ETH_MACON           (*((volatile uint32_t *)(ETH_BASE_ADDR+0x18)))
#define R8_ETH_ERXFCON          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x18))) /* ���հ����˿��ƼĴ��� */
#define  RB_ETH_ERXFCON_UCEN    0x80                  /* RW 0=�����øù���������1=��ANDOR=1��Ŀ���ַ��ƥ�佫�����ˣ���ANDOR=0��Ŀ���ַƥ�佫������ */
#define  RB_ETH_ERXFCON_ANDOR   0x40                  /* RW 1=AND�����й���������������ű����� 0=OR����һ��������������������� */
#define  RB_ETH_ERXFCON_CRCEN   0x20                  /* RW 0=�����øù���������1=��ANDOR=1��CRCУ��������ˣ���ANDOR=0��CRCУ����ȷ�������� */
#define  RB_ETH_ERXFCON_MPEN    0x08                  /* RW 0=�����øù���������1=��ANDOR=1����ħ�����������ˣ���ANDOR=0��ħ������������ */
#define  RB_ETH_ERXFCON_HTEN    0x04                  /* RW 0=�����øù���������1=��ANDOR=1��hash table��ƥ�佫�����ˣ���ANDOR=0��hash tableƥ�佫������ */
#define  RB_ETH_ERXFCON_MCEN    0x02                  /* RW 0=�����øù���������1=��ANDOR=1���鲥����ƥ�佫�����ˣ���ANDOR=0���鲥��ƥ�佫������ */
#define  RB_ETH_ERXFCON_BCEN    0x01                  /* RW 0=�����øù���������1=��ANDOR=1���ǹ㲥���������ˣ���ANDOR=0���㲥���������� */
#define R8_ETH_MACON1           (*((volatile uint8_t *)(ETH_BASE_ADDR+0x19))) /* Mac �������ƼĴ��� */
#define  RB_ETH_MACON1_FCEN     0x30                  /* RW ��FULDPX=0����Ч����FULDPX=1��11=����0 timer��ͣ֡��Ȼ��ֹͣ���ͣ�10=�����Է�����ͣ֡��01=����һ����ͣ֡��Ȼ��ֹͣ���ͣ�00=ֹͣ������ͣ֡ */
#define  RB_ETH_MACON1_TXPAUS   0x08                  /* RW ����pause֡ʹ�� */
#define  RB_ETH_MACON1_RXPAUS   0x04                  /* RW ����pause֡ʹ�� */
#define  RB_ETH_MACON1_PASSALL  0x02                  /* RW 1=û�����˵Ŀ���֡��д�뻺�棬0=����֡�������� */
#define  RB_ETH_MACON1_MARXEN   0x01                  /* RW MAC�����ʹ�� */
#define R8_ETH_MACON2           (*((volatile uint8_t *)(ETH_BASE_ADDR+0x1A))) /* Mac �������ƼĴ��� */
#define  RB_ETH_MACON2_PADCFG   0xE0                  /* RW �̰�������� */
#define  RB_ETH_MACON2_TXCRCEN  0x10                  /* RW �������crc��PADCFG������Ҫ���crc����λ��1 */
#define  RB_ETH_MACON2_PHDREN   0x08                  /* RW ����4�ֽڲ�����crcУ�� */
#define  RB_ETH_MACON2_HFRMEN   0x04                  /* RW ������վ���֡ */
#define  RB_ETH_MACON2_FULDPX   0x01                  /* RW ȫ˫�� */
#define R8_ETH_MABBIPG          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x1B))) /* ��С�������Ĵ��� */
#define  RB_ETH_MABBIPG_MABBIPG 0x7F                  /* RW ��С�������ֽ��� */

#define R32_ETH_TIM             (*((volatile uint32_t *)(ETH_BASE_ADDR+0x1C)))
#define R16_ETH_EPAUS           (*((volatile uint16_t *)(ETH_BASE_ADDR+0x1C))) /* RW ��������ͣ֡ʱ��Ĵ��� */
#define R16_ETH_MAMXFL          (*((volatile uint16_t *)(ETH_BASE_ADDR+0x1E))) /* RW �����հ����ȼĴ��� */
#define R16_ETH_MIRD            (*((volatile uint16_t *)(ETH_BASE_ADDR+0x20))) /* RW MII �����ݼĴ��� */

#define R32_ETH_MIWR            (*((volatile uint32_t *)(ETH_BASE_ADDR+0x24)))
#define R8_ETH_MIREGADR         (*((volatile uint8_t *)(ETH_BASE_ADDR+0x24))) /* MII ��ַ�Ĵ��� */
#define  RB_ETH_MIREGADR_MASK   0x1F                  /* RW PHY �Ĵ�����ַ���� */
#define R8_ETH_MISTAT           (*((volatile uint8_t *)(ETH_BASE_ADDR+0x25))) /* MII ״̬�Ĵ��� */
//#define  RB_ETH_MIREGADR_MIIWR  0x20                  /* WO MII д���� */
#define R16_ETH_MIWR            (*((volatile uint16_t *)(ETH_BASE_ADDR+0x26))) /* WO MII д���ݼĴ��� */
#define R32_ETH_MAADRL          (*((volatile uint32_t *)(ETH_BASE_ADDR+0x28))) /* RW MAC 1-4 */
#define R8_ETH_MAADRL1          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x28))) /* RW MAC 1 */
#define R8_ETH_MAADRL2          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x29))) /* RW MAC 2 */
#define R8_ETH_MAADRL3          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x2A))) /* RW MAC 3 */
#define R8_ETH_MAADRL4          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x2B))) /* RW MAC 4 */
#define R16_ETH_MAADRH          (*((volatile uint16_t *)(ETH_BASE_ADDR+0x2C))) /* RW MAC 5-6 */
#define R8_ETH_MAADRL5          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x2C))) /* RW MAC 4 */
#define R8_ETH_MAADRL6          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x2D))) /* RW MAC 4 */

//PHY��ַ 
#define PHY_BMCR                0x00                                            /* Control Register */
#define PHY_BMSR                0x01                                            /* Status Register */
#define PHY_ANAR                0x04                                            /* Auto-Negotiation Advertisement Register */
#define PHY_ANLPAR              0x05                                            /* Auto-Negotiation Link Partner Base  Page Ability Register*/
#define PHY_ANER                0x06                                            /* Auto-Negotiation Expansion Register */
#define PHY_MDIX                0x1e                                            /* �Զ��� MDIX ģʽ�Ĵ��� */
//�Զ���MDIXģʽ�Ĵ���  @PHY_MDIX
#define PN_NORMAL               0x04                                            /* ģ��p��n����ѡ�� */
#define MDIX_MODE_MASK          0x03                                            /* mdix���� */
#define MDIX_MODE_AUTO          0x00                                            /*  */
#define MDIX_MODE_MDIX          0x01
#define MDIX_MODE_MDI           0x02
//ECON2����ģʽ������  
#define RX_VCM_MODE_0
#define RX_VCM_MODE_1
#define RX_VCM_MODE_2
#define RX_VCM_MODE_3
//RX �ο���ѹֵ ����  @RX_REF
#define RX_REF_25mV             (0<<2)                                          /* 25mV */
#define RX_REF_49mV             (1<<2)                                          /* 49mV */
#define RX_REF_74mV             (2<<2)                                          /* 74mV */
#define RX_REF_98mV             (3<<2)                                          /* 98mV */
#define RX_REF_123mV            (4<<2)                                          /* 123mV */
#define RX_REF_148mV            (5<<2)                                          /* 148mV */
#define RX_REF_173mV            (6<<2)                                          /* 173mV */
#define RX_REF_198mV            (7<<2)                                          /* 198mV */
//TX DRIVER ƫ�õ���  @TX_AMP
#define TX_AMP_0                (0<<0)                                          /* 43mA   / 14.5mA   (1.4V/0.7V) */
#define TX_AMP_1                (1<<0)                                          /* 53.1mA / 18mA     (1.8V/0.9V) */
#define TX_AMP_2                (2<<0)                                          /* 75.6mA / 25.6mA   (2.6V/1.3V) */
#define TX_AMP_3                (3<<0)                                          /* 122mA  / 41.45mA  (4.1V/2.3V) */
//FCEN��ͣ֡����      @FCEN
#define FCEN_0_TIMER            (3<<4)                                          /* ���� 0 timer ��ͣ֡��Ȼ��ֹͣ���� */
#define FCEN_CYCLE              (2<<4)                                          /* �����Է�����ͣ֡ */
#define FCEN_ONCE               (1<<4)                                          /* ����һ����ͣ֡��Ȼ��ֹͣ���� */
#define FCEN_STOP               (0<<4)                                          /* ֹͣ������ͣ֡ */
//PADCFG�̰�����  @PADCFG
#define PADCFG_AUTO_0           (7<<5)                                          /* ���ж̰����00h��64�ֽڣ���4�ֽ�crc */
#define PADCFG_NO_ACT_0         (6<<5)                                          /* �����̰� */
#define PADCFG_DETE_AUTO        (5<<5)                                          /* ��⵽�ֶ�Ϊ8100h��VLAN������Զ����00h��64�ֽڣ�����̰����60�ֽ�0��������4�ֽ�crc */
#define PADCFG_NO_ACT_1         (4<<5)                                          /* �����̰� */
#define PADCFG_AUTO_1           (3<<5)                                          /* ͬ111 */
#define PADCFG_NO_ACT_2         (2<<5)                                          /* �����̰� */
#define PADCFG_AUTO_3           (1<<5)                                          /* ���ж̰����00h��60�ֽڣ���4�ֽ�crc */
#define PADCFG_NO_ACT_3         (0<<5)                                          /* �����̰� */

//MII����
//#define R16_ETH_MIWR            (*((volatile uint8_t *)(0x40009026)))    /* д���ݼĴ��� */
//#define R16_ETH_MIRD            (*((volatile uint8_t *)(0x40009020)))    /* ���Ĵ������ݼĴ��� */
//#define R8_ETH_MIREGADR         (*((volatile uint8_t *)(0x40009024-0x40009003+ETH_BASE_ADDR)))              /* MII �Ĵ�����ַ */
#define  RB_ETH_MIREGADR_MIIWR  0x20                                            /* WO MII д���� */
#define  RB_ETH_MIREGADR_MIRDL  0x1f                                            /* RW PHY �Ĵ�����ַ */    
/* PHY �������/״̬ */
#define PHY_DISCONN                          (1<<0)                  /* PHY�Ͽ� */
#define PHY_10M_FLL                          (1<<1)                  /* 10Mȫ˫�� */

//����
#define RX_BUF_IDLE 0


void WritePHYReg(uint8_t reg_add,uint16_t reg_val);
uint16_t ReadPHYReg(uint8_t reg_add);
void ETHInit( uint16_t LenthControl );
void ETH_IRQ_Rec_Deal(void);
void ETH_IRQ_Send_Deal(void);
void ETH_IRQ_LinkChange_Deal(uint8_t link_sta);
void ETH_IRQ_Deal(void);
uint8_t CH57xNET_GetMACAddr(void);
void ETHSend(uint8_t *p_send_buf, uint16_t send_len);
uint16_t ETHRec(uint8_t *p_rec_buf);
void ETHParaInit(void);
void ETH_IRQ_Deal(void);

#endif


