#ifndef __MAIN_H__
#define __MAIN_H__

/*
 * FreeRTOS Kernel V10.0.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. If you wish to use our Amazon
 * FreeRTOS name, please do so in a fair use way that does not cause confusion.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* A TCP echo server which is implemented with LwIP under FreeRTOS.
   The server listen to port 80, IP address could configure statically
   to 192.168.1.2 or assign by DHCP server. This server replies
   "Hello World!!" if the received string is "nuvoton", otherwise
    reply "Wrong Password!!" to its client. */

#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Demo application includes. */
#include "partest.h"
#include "flash.h"
#include "flop.h"
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "dynamic.h"
#include "BlockQ.h"
#include "blocktim.h"
#include "countsem.h"
#include "GenQTest.h"
#include "QueueSet.h"
#include "recmutex.h"
#include "death.h"

/* Hardware and starter kit includes. */
#include "NuMicro.h"

#include "lwip/netifapi.h"
#include "lwip/tcpip.h"
#include "netif/ethernetif.h"
#include "tcp_echoserver-netconn.h"

#include "hid_kb.h"


/* Priorities for the demo application tasks. */
#if 0
#define mainFLASH_TASK_PRIORITY            ( tskIDLE_PRIORITY + 1UL )
#define mainQUEUE_POLL_PRIORITY            ( tskIDLE_PRIORITY + 2UL )
#define mainSEM_TEST_PRIORITY              ( tskIDLE_PRIORITY + 1UL )
#define mainBLOCK_Q_PRIORITY               ( tskIDLE_PRIORITY + 2UL )
#define mainCHECK_TASK_PRIORITY            ( tskIDLE_PRIORITY + 3UL )
#else
#define mainFLASH_TASK_PRIORITY            ( tskIDLE_PRIORITY + 1UL )
#define mainQUEUE_POLL_PRIORITY            ( tskIDLE_PRIORITY + 1UL )
#define mainSEM_TEST_PRIORITY              ( tskIDLE_PRIORITY + 1UL )
#define mainCHECK_TASK_PRIORITY            ( tskIDLE_PRIORITY + 3UL )
#endif

#define mainCHECK_TASK_STACK_SIZE            ( configMINIMAL_STACK_SIZE )

/* The time between cycles of the 'check' task. */
#define mainCHECK_DELAY                        ( ( portTickType ) 5000 / portTICK_RATE_MS )

/* The LED used by the check timer. */
#define mainCHECK_LED                         ( 3UL )

/* A block time of zero simply means "don't block". */
#define mainDONT_BLOCK                        ( 0UL )

/* The period after which the check timer will expire, in ms, provided no errors
have been reported by any of the standard demo tasks.  ms are converted to the
equivalent in ticks using the portTICK_RATE_MS constant. */
#define mainCHECK_TIMER_PERIOD_MS            ( 3000UL / portTICK_RATE_MS )

/* The period at which the check timer will expire, in ms, if an error has been
reported in one of the standard demo tasks.  ms are converted to the equivalent
in ticks using the portTICK_RATE_MS constant. */
#define mainERROR_CHECK_TIMER_PERIOD_MS     ( 200UL / portTICK_RATE_MS )

/* Set mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY to 1 to create a simple demo.
Set mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY to 0 to create a much more
comprehensive test application.  See the comments at the top of this file, and
the documentation page on the http://www.FreeRTOS.org web site for more
information. */
#define mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY        0

//#define USE_DHCP

#ifdef USE_DHCP
#include "lwip/dhcp.h"
#endif

/*
 *  User define Area
 */ 
#define USB_QUEUELENGTH 	10 //MAX 10Byte
#define USB_ITEMSIZE  		1 //1Byte
#define USB_THREAD_STACKSIZE    100
#define USB_TASK_PRIORITY            ( tskIDLE_PRIORITY + 2UL )

/*-----------------------------------------------------------*/

/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

typedef struct _ethernet_data_dummy{
	uint8_t buf[8];
	uint8_t time;
}ethernet_data_dummy;

unsigned char my_mac_addr[6] = {0x00, 0x00, 0x00, 0x55, 0x66, 0x77};
struct netif netif;
static void vTcpTask( void *pvParameters );
static void vUsbTask(void *pvParameters);

volatile uint32_t  g_tick_cnt;
uint8_t volatile g_u8EP2Ready = 0;
uint32_t u32TrimInit;
QueueHandle_t usb_q_handle;
SemaphoreHandle_t usb_s_handle;

#endif
