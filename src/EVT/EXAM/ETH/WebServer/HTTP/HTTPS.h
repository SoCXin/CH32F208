/********************************** (C) COPYRIGHT *******************************
 * File Name          : HTTPS.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/05/31
 * Description        : HTTP related parameters.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef	__HTTPS_H__
#define	__HTTPS_H__
#include "debug.h"
#include "wchnet.h"

/*Address where configuration
 * information is stored*/
#define PAGE_WRITE_START_ADDR     ((uint32_t)0x0803FC00) /* Start from 255K */
#define PAGE_WRITE_END_ADDR       ((uint32_t)0x08040000) /* End at 256K */
#define FLASH_PAGE_SIZE           256

#define BASIC_CFG_ADDR            ((uint32_t)0x0803FC00)
#define PORT_CFG_ADDR             ((uint32_t)0x0803FD00)
#define LOGIN_CFG_ADDR            ((uint32_t)0x0803FE00)
#define BASIC_CFG_LEN             40
#define PORT_CFG_LEN              40
#define LOGIN_CFG_LEN             40


#define MAX_URL_SIZE              128
#define HTTP_SERVER_PORT          80

/* HTTP请求方法 */
#define	METHOD_ERR		          0
#define	METHOD_GET		          1
#define	METHOD_HEAD		          2
#define	METHOD_POST		          3

/* HTTP请求类型 */
#define	PTYPE_ERR		          0
#define	PTYPE_HTML	              1
#define	PTYPE_PNG		          2
#define	PTYPE_CSS		          3
#define PTYPE_GIF                 4

/*WCHNET网页通信协议定义*/
#define MODE_TCPSERVER            0
#define MODE_TCPCLIENT            1

/* HTML Doc. for ERROR */
#define RES_HTMLHEAD_OK	"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"      //HTML type response message, the last two \r\n must have

#define RES_PNGHEAD_OK	"HTTP/1.1 200 OK\r\nContent-Type: image/png\r\n\r\n"      //PNG type response message, the last two \r\n must have
  
#define RES_CSSHEAD_OK  "HTTP/1.1 200 OK\r\nContent-Type: text/css\r\n\r\n"       //CSS type response message, the last two \r\n must have

#define RES_GIFHEAD_OK  "HTTP/1.1 200 OK\r\nContent-Type: image/gif\r\n\r\n"      //GIF type response message, the last two \r\n must have
 /*-----------------*/

typedef struct Basic_Cfg                        //Basic configuration parameters
{
	uint8_t flag[2];                                 //Configuration information verification code: 0x57,0xab
	uint8_t mac[6];
	uint8_t ip[4];
	uint8_t mask[4];
	uint8_t gateway[4];
} *Basic_Cfg;


typedef struct Port_Cfg                         //Port configuration parameters
{
    uint8_t flag[2];                                 //Configuration information verification code: 0x57,0xab
    uint8_t mode;
    uint8_t src_port[2];
    uint8_t des_ip[4];
    uint8_t des_port[2];
} *Port_Cfg;


typedef struct Login_Cfg                        //Login configuration parameters
{
    uint8_t  flag[2];                                //Configuration information verification code: 0x57,0xab
    uint8_t  user[10];
    uint8_t  pass[10];
} *Login_Cfg;


typedef struct _st_http_request                 //Browser request information
{
	char	METHOD;					
	char	TYPE;					
	char	URL[MAX_URL_SIZE];
}st_http_request;

typedef struct Para_Tab                         //Configuration information parameter table
{
	char *para;                                 //Configuration item name
	char value[20];                             //Configuration item value
}Parameter;

extern Basic_Cfg Basic_CfgBuf;

extern Login_Cfg Login_CfgBuf;

extern Port_Cfg  Port_CfgBuf;

extern st_http_request *http_request;

extern uint8_t basicbuf[BASIC_CFG_LEN];

extern uint8_t portbuf[PORT_CFG_LEN];

extern uint8_t loginbuf[LOGIN_CFG_LEN];

extern uint8_t Basic_Default[BASIC_CFG_LEN];

extern uint8_t Login_Default[LOGIN_CFG_LEN];

extern uint8_t Port_Default[PORT_CFG_LEN];

extern uint8_t httpweb[200] ;

extern uint8_t RecvBuffer[];

extern uint8_t DealDataFlag;

extern uint8_t socket;

extern void ParseHttpRequest(st_http_request *, char *);	

extern void ParseURLType(char *, char *);

extern void MakeHttpResponse(unsigned char *, char);			

extern char *GetURLName(char* url);

extern char *DataLocate(char *buf,char *name);

extern void copy_flash(const char *html, u32 len);

extern void Init_Para_Tab(void) ;

extern void Web_Server(void);

extern void WEB_ERASE(u32 Page_Address, u32 Length );

extern FLASH_Status WEB_WRITE( u32 StartAddr, uint8_t *Buffer, u32 Length );

extern void WEB_READ( u32 StartAddr, uint8_t *Buffer, u32 Length );

#endif	
