/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb output transmits the error message.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *  V2.0.0     Mar-12-2021     YW              1. modify to fit this project
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>
#include "string.h"
//#include "gimbal_task.h"
//#include "gimbal_behaviour.h"
//#include "detect_task.h"



#include "debugio.h"
#include "buffer.h"
#include "ringbuf.h"

#include "CRCcrc_util.h"
#include "protocol_def.h"

static void usb_printf(const char *fmt,...);

static uint8_t usb_buf[256];
//static const char status[2][7] = {"OK", "ERROR!"};
//const error_t *error_list_usb_local;
extern UART_HandleTypeDef huart6;
float AI_R[2];
enum VisionCmdState recv_state = SOF;
struct VisionCommandFrame frame;
//float32_t AI_X;
//float32_t AI_Y;
IO_BufferTypeDef uart6RxBuffer;
extern float AI_t;

RingBuffer cmdBuffer;


/* Definitions for defaultTask */

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for USART6RxTask */
osThreadId_t USART6RxTaskHandle;
const osThreadAttr_t USART6RxTask_attributes = {
  .name = "USART6RxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for visionCmdRxTask */
osThreadId_t visionCmdRxTaskHandle;
const osThreadAttr_t visionCmdRxTask_attributes = {
  .name = "visionCmdRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DBGSerialMutex */
osMutexId_t DBGSerialMutexHandle;
const osMutexAttr_t DBGSerialMutex_attributes = {
  .name = "DBGSerialMutex"
};
/* Definitions for usart6RxBinarySemaphore */
osSemaphoreId_t usart6RxBinarySemaphoreHandle;
StaticSemaphore_t usart6RxBinarySemaphoreControlBlock;
const osSemaphoreAttr_t usart6RxBinarySemaphore_attributes = {
  .name = "usart6RxBinarySemaphore",
  .cb_mem = &usart6RxBinarySemaphoreControlBlock,
  .cb_size = sizeof(usart6RxBinarySemaphoreControlBlock),
};



void usb_task(void *argument)
{
    //MX_USB_DEVICE_Init();
	DBGSerialMutexHandle = osMutexNew(&DBGSerialMutex_attributes);
	usart6RxBinarySemaphoreHandle = osSemaphoreNew(1, 1, &usart6RxBinarySemaphore_attributes);
	RB_Init(&cmdBuffer, 512);
	IO_Buffer_Init(&uart6RxBuffer, 512, usart6RxBinarySemaphoreHandle);

//    error_list_usb_local = get_error_list_point();
     uint32_t res, res16;

     size_t recv_state_pos = 0;

     __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
     HAL_UART_Receive_DMA(&huart6, uart6RxBuffer.buffer, uart6RxBuffer.size);
     //initiate USB
     MX_USB_DEVICE_Init();

    while(1)
    {
    	IO_Buffer_Acquire(&uart6RxBuffer);

    	    dbgprintf("\nbufferlen=%d\n",uart6RxBuffer.len);
    	    dbgbuf(uart6RxBuffer.buffer, uart6RxBuffer.len);
    	    size_t i = 0;
    	    while (i < uart6RxBuffer.len)
    	    {
    	      uint8_t ch = uart6RxBuffer.buffer[i];
    	      switch (recv_state)
    	      {
    	      case SOF:
    	    	if (ch == 0x05)
    	    	{
    	    	  frame.SOF = ch;
    	    	  recv_state_pos = 0;
    	    	  recv_state = DLEN;
    	    	}
    	    	break;
    	      case DLEN:
    	    	*(((uint8_t *)&(frame.data_length))+recv_state_pos) = ch;
    	    	recv_state_pos++;
    	    	if (recv_state_pos >=2)
    	    	{
    	    	  recv_state_pos = 0;
    	    	  recv_state = SEQ;
    	    	}
    	    	break;
    	      case SEQ:
    	    	frame.sequence = ch;
    	    	recv_state_pos = 0;
    	    	recv_state = CRC8;
    	    	break;
    	      case CRC8:
    	    	frame.CRC8 = ch;
    	    	crc8compute((void *)&frame.SOF, 5, &res);
    	    	recv_state_pos = 0;
    	    	if (res == 0)
    	    	{
    	          recv_state = CMD;
    	    	}
    	    	else
    	    	{
    	          recv_state = SOF;
    	    	}
    	      	break;
    	      case CMD:
    	      	*(((uint8_t *)&(frame.cmd_id))+recv_state_pos) = ch;
    	      	recv_state_pos++;
    	      	if (recv_state_pos >=2)
    	      	{
    	      	  recv_state_pos = 0;
    	      	  recv_state = DATA;
    	      	}
    	    	break;
    	      case DATA:
    	        *(((uint8_t *)&(frame.data))+recv_state_pos) = ch;
    	        recv_state_pos++;
    	        if (recv_state_pos >= frame.data_length)
    	        {
    	//          recv_state_pos = 0;
    	          recv_state = CRC16;
    	        }
    	    	break;
    	      case CRC16:
    	//        *(((uint8_t *)&(frame.CRC16))+recv_state_pos) = ch;
    	    	*(((uint8_t *)&(frame.data))+recv_state_pos) = ch;
    	        recv_state_pos++;
    	        if (recv_state_pos >= frame.data_length + 2)
    	        {
    	          crc16compute((void *)&frame, (uint64_t)9+(uint64_t)frame.data_length, &res16);
    	          recv_state_pos = 0;
    	          if (res16 == 0)
    	          {
    	            // TODO: Process function goes here
    	        	//dbgprintf("\ndx = %.1f, dy = %.1f\n", frame.data.demoCmd.dx, frame.data.demoCmd.dy);
    	        	// Assign value to X and Y after receiving.

//    	        	 AI_X=frame.data.demoCmd.dx;
//    	        	 AI_Y=frame.data.demoCmd.dy;
    	        	 AI_t=0;
    	        	 pthread_t th;
    	          }
    	          recv_state = SOF;
    	        }
    	    	break;
    	      }

    	      i++;
    	    }

    	    HAL_UART_Receive_DMA(&huart6, uart6RxBuffer.buffer, uart6RxBuffer.size);
//    	HAL_UART_Transmit(&huart6,TxData,10,0xffff);
        osDelay(1);

//            status[error_list_usb_local[DBUS_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
//            status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[BOARD_MPU6500_TOE].error_exist],
//            status[error_list_usb_local[BOARD_IST8310_TOE].error_exist]
//        );
    }

}

static void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(usb_buf, len);
}
