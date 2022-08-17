#ifndef __EXTERN_H_
#define __EXTERN_H_

//volatile uint32_t  g_tick_cnt;

typedef struct _ethernet_data{
	uint8_t buf;
	uint8_t time;
}ethernet_data;

extern uint8_t volatile g_u8EP2Ready;
extern uint32_t u32TrimInit;
extern QueueHandle_t usb_q_handle;
extern SemaphoreHandle_t usb_s_handle;

#endif
