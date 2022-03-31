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

//网络收发MAC层配置 1536
#define RX_BUF_SIZE             1536                                            /* 接收缓存大小 */
#define TX_BUF_SIZE             1536                                            /* 发送缓存大小 */
#define ETHER_HEAD_LEN          14                                              /* 网络帧头长度 */
#define MACADDR_LEN             6
#define MAC_MAX_LEN             1536                                            /* MAC层最大包长度 */
#define RX_QUEUE_NUM            4                                               /* 接收缓冲区队列数 */
#define TX_QUEUE_NUM            2                                               /* 发送缓冲区队列数 */

//收发管理定义
typedef struct _RXBUFST
{
    uint8_t  RecvEn;                                                            /* 使能控制 */
    uint8_t  RecvIndex;                                                         /* MAC层接收缓冲区的索引 */
    uint8_t  ReadIndex;                                                         /* 读取缓冲区的索引值 */
    uint8_t  RemainCount;                                                       /* 剩余未读取的数据包 */
    uint8_t  RxBufStat[RX_QUEUE_NUM];                                           /* 接收缓冲区的状态 */
    uint16_t RxBufLen[RX_QUEUE_NUM];                                            /* 接收缓冲区数据的有效长度 */
    uint32_t RxBufAddr[RX_QUEUE_NUM];                                           /* 接收缓冲区的起始地址 */
}RXBUFST;

typedef struct _TXBUFST
{
    uint8_t  SendEn;                                                            /* 使能控制 */
    uint8_t  SendIndex;                                                         /* MAC层发送缓冲区的索引 */
    uint8_t  WriteIndex;                                                        /* 发送缓冲区空闲的索引值 */
    uint8_t  TxQueueCnt;                                                        /* 发送队列中排队的数量 */
    uint8_t  TxBufStat[TX_QUEUE_NUM];                                           /* 发送缓冲区的状态 */
    uint16_t TxBufLen[TX_QUEUE_NUM];                                            /* 发送缓冲区数据的有效长度 */
    uint32_t TxBufAddr[TX_QUEUE_NUM];                                          /* 发送缓冲区的起始地址 */
}TXBUFST;

/* 设置/清除全局中断 */
#define SET_GOLB_INT(a,b)        (a |= b)
#define RESET_GOLB_INT(a,b)      (a &= ~b)

/* 设置/清除socket中断 */
#define SET_SOCKET_INT(a,b)      ((a->IntStatus)|= b)
#define RESET_SOCKET_INT(a,b)    ((a->IntStatus)&= ~b)
#define ETH_BASE_ADDR  0x40028000 /* 以太网基地址 */

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
#define R8_ETH_EIE              (*((volatile uint8_t *)(ETH_BASE_ADDR+3))) /* 中断使能寄存器 */
#define  RB_ETH_EIE_INTIE       0x80                  /* RW 中断使能 */
#define  RB_ETH_EIE_RXIE        0x40                  /* RW 接收完成中断使能 */
#define  RB_ETH_EIE_LINKIE      0x10                  /* RW Link 变化中断使能 */
#define  RB_ETH_EIE_TXIE        0x08                  /* RW 发送完成中断使能 */
#define  RB_ETH_EIE_R_EN50      0x04                  /* RW TX 50Ω电阻调节。1：片内 50Ω连接 0：片内 50Ω断开 */
#define  RB_ETH_EIE_TXERIE      0x02                  /* RW 发送错误中断使能 */
#define  RB_ETH_EIE_RXERIE      0x01                  /* RW1 接收错误标志 */
//#define R32_ETH_CON             (*((volatile uint32_t *)(ETH_BASE_ADDR+4)))
#define R8_ETH_EIR              (*((volatile uint8_t *)(ETH_BASE_ADDR+4))) /* 中断标志寄存器 */
#define  RB_ETH_EIR_RXIF        0x40                  /* RW1 接收完成标志 */
#define  RB_ETH_EIR_LINKIF      0x10                  /* RW1 Link 变化标志 */
#define  RB_ETH_EIR_TXIF        0x08                  /* RW1 发送完成标志 */
#define  RB_ETH_EIR_TXERIF      0x02                  /* RW1 发送错误标志 */
#define  RB_ETH_EIR_RXERIF      0x01
#define R8_ETH_ESTAT            (*((volatile uint8_t *)(ETH_BASE_ADDR+5))) /* 状态寄存器 */
#define  RB_ETH_ESTAT_INT       0x80                  /* RW1 中断 */
#define  RB_ETH_ESTAT_BUFER     0x40                  /* RW1 Buffer 错误，理论上 mcu 主频太低才会发生 */
#define  RB_ETH_ESTAT_RXCRCER   0x20                  /* RO 接收 crc 出错 */
#define  RB_ETH_ESTAT_RXNIBBLE  0x10                  /* RO 接收 nibble 错误 */
#define  RB_ETH_ESTAT_RXMORE    0x08                  /* RO 接收超过最大数据包 */
#define  RB_ETH_ESTAT_RXBUSY    0x04                  /* RO 接收进行中 */
#define  RB_ETH_ESTAT_TXABRT    0x02                  /* RO 发送被 mcu 打断 */
#define R8_ETH_ECON2            (*((volatile uint8_t *)(ETH_BASE_ADDR+6))) /* ETH PHY模拟模块控制寄存器 */
#define  RB_ETH_ECON2_RX        0x0E                  /* 必须写入011 */
#define  RB_ETH_ECON2_TX        0x01
#define  RB_ETH_ECON2_MUST      0x06                  /* 必须写入011 */
#define R8_ETH_ECON1            (*((volatile uint8_t *)(ETH_BASE_ADDR+7))) /* 收发控制寄存器 */
#define  RB_ETH_ECON1_TXRST     0x80                  /* RW 发送模块复位 */
#define  RB_ETH_ECON1_RXRST     0x40                  /* RW 接收模块复位 */
#define  RB_ETH_ECON1_TXRTS     0x08                  /* RW 发送开始，发送完成后自动清零，如主动清零会使发送错误标志TXERIF和TXABRT变1 */
#define  RB_ETH_ECON1_RXEN      0x04                  /* RW 接收使能，清零时如正在接受则错误标志RXERIF变1 */

#define R32_ETH_TX              (*((volatile uint32_t *)(ETH_BASE_ADDR+8))) /* 发送控制 */
#define R16_ETH_ETXST           (*((volatile uint16_t *)(ETH_BASE_ADDR+8))) /* RW 发送 DMA 缓冲区起始地址 */
#define R16_ETH_ETXLN           (*((volatile uint16_t *)(ETH_BASE_ADDR+0xA))) /* RW 发送长度 */
#define R32_ETH_RX              (*((volatile uint32_t *)(ETH_BASE_ADDR+0xC))) /* 接收控制 */
#define R16_ETH_ERXST           (*((volatile uint16_t *)(ETH_BASE_ADDR+0xC))) /* RW 接收 DMA 缓冲区起始地址 */
#define R16_ETH_ERXLN           (*((volatile uint16_t *)(ETH_BASE_ADDR+0xE))) /* RO 接收长度 */

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
#define R8_ETH_ERXFCON          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x18))) /* 接收包过滤控制寄存器 */
#define  RB_ETH_ERXFCON_UCEN    0x80                  /* RW 0=不启用该过滤条件，1=当ANDOR=1，目标地址不匹配将被过滤，当ANDOR=0，目标地址匹配将被接收 */
#define  RB_ETH_ERXFCON_ANDOR   0x40                  /* RW 1=AND，所有过滤条件都满足包才被接收 0=OR，任一过滤条件满足包均被接收 */
#define  RB_ETH_ERXFCON_CRCEN   0x20                  /* RW 0=不启用该过滤条件，1=当ANDOR=1，CRC校验错将被过滤，当ANDOR=0，CRC校验正确将被接收 */
#define  RB_ETH_ERXFCON_MPEN    0x08                  /* RW 0=不启用该过滤条件，1=当ANDOR=1，非魔法包将被过滤，当ANDOR=0，魔法包将被接收 */
#define  RB_ETH_ERXFCON_HTEN    0x04                  /* RW 0=不启用该过滤条件，1=当ANDOR=1，hash table不匹配将被过滤，当ANDOR=0，hash table匹配将被接收 */
#define  RB_ETH_ERXFCON_MCEN    0x02                  /* RW 0=不启用该过滤条件，1=当ANDOR=1，组播包不匹配将被过滤，当ANDOR=0，组播包匹配将被接收 */
#define  RB_ETH_ERXFCON_BCEN    0x01                  /* RW 0=不启用该过滤条件，1=当ANDOR=1，非广播包将被过滤，当ANDOR=0，广播包将被接收 */
#define R8_ETH_MACON1           (*((volatile uint8_t *)(ETH_BASE_ADDR+0x19))) /* Mac 层流控制寄存器 */
#define  RB_ETH_MACON1_FCEN     0x30                  /* RW 当FULDPX=0均无效，当FULDPX=1，11=发送0 timer暂停帧，然后停止发送，10=周期性发送暂停帧，01=发送一次暂停帧，然后停止发送，00=停止发送暂停帧 */
#define  RB_ETH_MACON1_TXPAUS   0x08                  /* RW 发送pause帧使能 */
#define  RB_ETH_MACON1_RXPAUS   0x04                  /* RW 接收pause帧使能 */
#define  RB_ETH_MACON1_PASSALL  0x02                  /* RW 1=没被过滤的控制帧将写入缓存，0=控制帧将被过滤 */
#define  RB_ETH_MACON1_MARXEN   0x01                  /* RW MAC层接收使能 */
#define R8_ETH_MACON2           (*((volatile uint8_t *)(ETH_BASE_ADDR+0x1A))) /* Mac 层封包控制寄存器 */
#define  RB_ETH_MACON2_PADCFG   0xE0                  /* RW 短包填充设置 */
#define  RB_ETH_MACON2_TXCRCEN  0x10                  /* RW 发送添加crc，PADCFG中如需要添加crc，该位置1 */
#define  RB_ETH_MACON2_PHDREN   0x08                  /* RW 特殊4字节不参与crc校验 */
#define  RB_ETH_MACON2_HFRMEN   0x04                  /* RW 允许接收巨型帧 */
#define  RB_ETH_MACON2_FULDPX   0x01                  /* RW 全双工 */
#define R8_ETH_MABBIPG          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x1B))) /* 最小包间间隔寄存器 */
#define  RB_ETH_MABBIPG_MABBIPG 0x7F                  /* RW 最小包间间隔字节数 */

#define R32_ETH_TIM             (*((volatile uint32_t *)(ETH_BASE_ADDR+0x1C)))
#define R16_ETH_EPAUS           (*((volatile uint16_t *)(ETH_BASE_ADDR+0x1C))) /* RW 流控制暂停帧时间寄存器 */
#define R16_ETH_MAMXFL          (*((volatile uint16_t *)(ETH_BASE_ADDR+0x1E))) /* RW 最大接收包长度寄存器 */
#define R16_ETH_MIRD            (*((volatile uint16_t *)(ETH_BASE_ADDR+0x20))) /* RW MII 读数据寄存器 */

#define R32_ETH_MIWR            (*((volatile uint32_t *)(ETH_BASE_ADDR+0x24)))
#define R8_ETH_MIREGADR         (*((volatile uint8_t *)(ETH_BASE_ADDR+0x24))) /* MII 地址寄存器 */
#define  RB_ETH_MIREGADR_MASK   0x1F                  /* RW PHY 寄存器地址掩码 */
#define R8_ETH_MISTAT           (*((volatile uint8_t *)(ETH_BASE_ADDR+0x25))) /* MII 状态寄存器 */
//#define  RB_ETH_MIREGADR_MIIWR  0x20                  /* WO MII 写命令 */
#define R16_ETH_MIWR            (*((volatile uint16_t *)(ETH_BASE_ADDR+0x26))) /* WO MII 写数据寄存器 */
#define R32_ETH_MAADRL          (*((volatile uint32_t *)(ETH_BASE_ADDR+0x28))) /* RW MAC 1-4 */
#define R8_ETH_MAADRL1          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x28))) /* RW MAC 1 */
#define R8_ETH_MAADRL2          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x29))) /* RW MAC 2 */
#define R8_ETH_MAADRL3          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x2A))) /* RW MAC 3 */
#define R8_ETH_MAADRL4          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x2B))) /* RW MAC 4 */
#define R16_ETH_MAADRH          (*((volatile uint16_t *)(ETH_BASE_ADDR+0x2C))) /* RW MAC 5-6 */
#define R8_ETH_MAADRL5          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x2C))) /* RW MAC 4 */
#define R8_ETH_MAADRL6          (*((volatile uint8_t *)(ETH_BASE_ADDR+0x2D))) /* RW MAC 4 */

//PHY地址 
#define PHY_BMCR                0x00                                            /* Control Register */
#define PHY_BMSR                0x01                                            /* Status Register */
#define PHY_ANAR                0x04                                            /* Auto-Negotiation Advertisement Register */
#define PHY_ANLPAR              0x05                                            /* Auto-Negotiation Link Partner Base  Page Ability Register*/
#define PHY_ANER                0x06                                            /* Auto-Negotiation Expansion Register */
#define PHY_MDIX                0x1e                                            /* 自定义 MDIX 模式寄存器 */
//自定义MDIX模式寄存器  @PHY_MDIX
#define PN_NORMAL               0x04                                            /* 模拟p，n极性选择 */
#define MDIX_MODE_MASK          0x03                                            /* mdix设置 */
#define MDIX_MODE_AUTO          0x00                                            /*  */
#define MDIX_MODE_MDIX          0x01
#define MDIX_MODE_MDI           0x02
//ECON2测试模式，待定  
#define RX_VCM_MODE_0
#define RX_VCM_MODE_1
#define RX_VCM_MODE_2
#define RX_VCM_MODE_3
//RX 参考电压值 设置  @RX_REF
#define RX_REF_25mV             (0<<2)                                          /* 25mV */
#define RX_REF_49mV             (1<<2)                                          /* 49mV */
#define RX_REF_74mV             (2<<2)                                          /* 74mV */
#define RX_REF_98mV             (3<<2)                                          /* 98mV */
#define RX_REF_123mV            (4<<2)                                          /* 123mV */
#define RX_REF_148mV            (5<<2)                                          /* 148mV */
#define RX_REF_173mV            (6<<2)                                          /* 173mV */
#define RX_REF_198mV            (7<<2)                                          /* 198mV */
//TX DRIVER 偏置电流  @TX_AMP
#define TX_AMP_0                (0<<0)                                          /* 43mA   / 14.5mA   (1.4V/0.7V) */
#define TX_AMP_1                (1<<0)                                          /* 53.1mA / 18mA     (1.8V/0.9V) */
#define TX_AMP_2                (2<<0)                                          /* 75.6mA / 25.6mA   (2.6V/1.3V) */
#define TX_AMP_3                (3<<0)                                          /* 122mA  / 41.45mA  (4.1V/2.3V) */
//FCEN暂停帧控制      @FCEN
#define FCEN_0_TIMER            (3<<4)                                          /* 发送 0 timer 暂停帧，然后停止发送 */
#define FCEN_CYCLE              (2<<4)                                          /* 周期性发送暂停帧 */
#define FCEN_ONCE               (1<<4)                                          /* 发送一次暂停帧，然后停止发送 */
#define FCEN_STOP               (0<<4)                                          /* 停止发送暂停帧 */
//PADCFG短包控制  @PADCFG
#define PADCFG_AUTO_0           (7<<5)                                          /* 所有短包填充00h至64字节，再4字节crc */
#define PADCFG_NO_ACT_0         (6<<5)                                          /* 不填充短包 */
#define PADCFG_DETE_AUTO        (5<<5)                                          /* 检测到字段为8100h的VLAN网络包自动填充00h至64字节，否则短包填充60字节0，填充后再4字节crc */
#define PADCFG_NO_ACT_1         (4<<5)                                          /* 不填充短包 */
#define PADCFG_AUTO_1           (3<<5)                                          /* 同111 */
#define PADCFG_NO_ACT_2         (2<<5)                                          /* 不填充短包 */
#define PADCFG_AUTO_3           (1<<5)                                          /* 所有短包填充00h至60字节，再4字节crc */
#define PADCFG_NO_ACT_3         (0<<5)                                          /* 不填充短包 */

//MII控制
//#define R16_ETH_MIWR            (*((volatile uint8_t *)(0x40009026)))    /* 写数据寄存器 */
//#define R16_ETH_MIRD            (*((volatile uint8_t *)(0x40009020)))    /* 读寄存器数据寄存器 */
//#define R8_ETH_MIREGADR         (*((volatile uint8_t *)(0x40009024-0x40009003+ETH_BASE_ADDR)))              /* MII 寄存器地址 */
#define  RB_ETH_MIREGADR_MIIWR  0x20                                            /* WO MII 写命令 */
#define  RB_ETH_MIREGADR_MIRDL  0x1f                                            /* RW PHY 寄存器地址 */    
/* PHY 命令参数/状态 */
#define PHY_DISCONN                          (1<<0)                  /* PHY断开 */
#define PHY_10M_FLL                          (1<<1)                  /* 10M全双工 */

//其它
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


