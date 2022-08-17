

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "string.h"
#include "tcp_echoserver-netconn.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "hid_kb.h"
#include "extern.h"
#include "stdlib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TCPECHOSERVER_THREAD_PRIO    ( tskIDLE_PRIORITY + 2UL )
#define TCPECHOSERVER_THREAD_STACKSIZE  200
#define FAIL_STATUS 0
#define PASS_STATUS 1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief serve TCP connection
  * @param conn: pointer on connection structure
  * @retval None
  */
err_t tcp_echoserver_serve(struct netconn *conn)
{
  struct netbuf *inbuf;
  char* buf;
  u16_t buflen;
  char string_pass[] = "Command_Verify!!";
  char string_fail[] = "Wrong_Command!!";
  uint8_t i;
  ethernet_data tx_data;
	err_t checker;
		
	BaseType_t xStatus;
	const TickType_t xTicksToWait = 10/portTICK_PERIOD_MS;

  printf("Wait for TCP data       ...");

  /* Read the data from the port, blocking if nothing yet there.
     We assume the request (the part we care about) is in one netbuf */
  
  checker = netconn_recv(conn,&inbuf);
  printf(" [OK] ...\n");
	  
	//if(checker != ERR_OK)
	//if(netconn_err(conn) != ERR_OK)
	if(checker != ERR_OK)
  {
	  printf("TCP Connect Failed      ...\n"); 
		
		printf("Close TCP connection    ...");
		netconn_close(conn);
		printf(" [OK] ...\n");
		
		printf("Delete TCP Data buf     ...");
		netbuf_delete(inbuf);
		printf(" [OK] ...\n");
			
		return FAIL_STATUS;
	}

  if(inbuf != NULL)
  {
    if(netconn_err(conn) == ERR_OK)
    {  
		  netbuf_data(inbuf, (void**)&buf, &buflen);
				
			if(strncmp(buf+3, "COMMAND", 7 ) == 0)
			{
			  tx_data.buf = *(buf);
				tx_data.time = atoi(buf+1);
				netconn_write(conn, (const unsigned char*)string_pass, (size_t)strlen(string_pass), NETCONN_NOFLAG);				
			  xQueueSend( usb_q_handle, &(tx_data), xTicksToWait);
				netbuf_delete(inbuf);
				}
			 else
			 { 
				 netconn_write(conn, (const unsigned char*)string_fail, (size_t)strlen(string_fail), NETCONN_NOFLAG);
				 netbuf_delete(inbuf);
			 }
		 }
   }
		
		 return PASS_STATUS;
}

/**
  * @brief  TCP echo server thread
  * @param arg pointer on argument(not used here)
  * @retval None
  */
static void tcp_echoserver_netconn_thread(void *arg)
{
  struct netconn *conn, *newconn;
  err_t err;
	err_t checker_server;
  BaseType_t sema_status;
	
  xSemaphoreTake(usb_s_handle, (TickType_t)100);
	
  printf("\n");
  printf("+--------------------------------------------------------+\n");
  printf("|                    TCP/IP Server Task                  |\n");
  printf("+--------------------------------------------------------+\n");
  /* Create a new TCP connection handle */	
  conn = netconn_new(NETCONN_TCP);
	
	xSemaphoreGive(usb_s_handle);
	
  if (conn!= NULL)
  {
    /* Bind to port 80 (HTTP) with default IP address */
    err = netconn_bind(conn, NULL, 80);

    if (err == ERR_OK)
    {
      /* Put the connection into LISTEN state */
      netconn_listen(conn);
		
			while(1)
	    {
        printf("Wait for TCP connection ...");
        /* accept any icoming connection */
        netconn_accept(conn,&newconn);
			  if(newconn)
		    {
				  printf(" [OK] ...\n");
		
				  while(1)
			    {
             /* serve connection */
             checker_server = tcp_echoserver_serve(newconn);
						
             /* delete connection */
             //netconn_delete(newconn);
				     if(checker_server == FAIL_STATUS)
				     {
					     printf("Delete TCP connection    ...");
					     netconn_delete(newconn);
						   printf(" [OK] ...\n");
						   break;
					   }
           } //while
			   } //if(newconn)
       }//while
		}
    else
    {
      printf("can not bind netconn");
    }
    
	}
  else
  {
     printf("can not create netconn");
  }

  
}


/**
  * @brief  Initialize the TCP server (start its thread)
  * @param  none
  * @retval None
  */
void tcp_echoserver_netconn_init()
{
    sys_thread_new("TCPECHO", tcp_echoserver_netconn_thread, NULL, TCPECHOSERVER_THREAD_STACKSIZE, TCPECHOSERVER_THREAD_PRIO);
}

