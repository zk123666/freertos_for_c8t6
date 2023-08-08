#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sys.h"
/***************
STM32F103C8T6最小单片机的freertos移植实验
作者：ZSF
***************/

#define START_TASK_PRIO			1
#define START_STK_SIZE			128
TaskHandle_t StartTask_Handler;
void start_task(void *pvParameters);

#define LED0_TASK_PRIO			2
#define LED0_STK_SIZE				50
TaskHandle_t LED0Task_Handler;
void led0_task(void *pvParameters);

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	delay_init();
	uart_init(115200);
	LED_Init();

	LED0=0;
	xTaskCreate((TaskFunction_t		)start_task,
							(const char*			)"start_task",
							(uint16_t					)START_STK_SIZE,
							(void*						)NULL,
							(UBaseType_t			)START_TASK_PRIO,
							(TaskHandle_t*		)&StartTask_Handler);

	vTaskStartScheduler();
}

void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();
	
	xTaskCreate((TaskFunction_t		)led0_task,
							(const char*			)"led0_task",
							(uint16_t					)LED0_STK_SIZE,
							(void*						)NULL,
							(UBaseType_t			)LED0Task_Handler,
							(TaskHandle_t*		)&LED0Task_Handler);
	
	vTaskDelete(StartTask_Handler);
	taskEXIT_CRITICAL();

}

void led0_task(void *pvParameters)
{
	while(1)
	{
		//LED0 = ~LED0;
		if(USART_RX_BUF[0]==0x31)
		{
		taskENTER_CRITICAL();
		printf("FreeRTOS收到了1\r\n");
        USART_RX_STA=0;
        USART_RX_BUF[0]=0;
		taskEXIT_CRITICAL();
		}
        else if (USART_RX_BUF[0]==0x30)
        {
		taskENTER_CRITICAL();
		printf("FreeRTOS收到了0\r\n");
        USART_RX_STA=0;
        USART_RX_BUF[0]=0;
		taskEXIT_CRITICAL();
		}
        else
        {}
		vTaskDelay(10);
	
	}
}






