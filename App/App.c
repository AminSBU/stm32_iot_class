#include "main.h"

osThreadId_t TaskHandle;
osThreadId_t UARTTransmitTaskHandle;
osThreadId_t UARTReceiveTaskHandle;

uint8_t aTxStartMessage[] = "\n\r ****UART-Hyperterminal communication based on DMA****\n\r Enter 10 characters using keyboard :\n\r";
uint8_t aTxEndMessage[] = "\n\r Example Finished\n\r";

extern UART_HandleTypeDef huart5;

/* Buffer used for reception */
uint8_t aRxBuffer[1000];

const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t uartTransmitTask_attributes = {
  .name = "uartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t uartReceiveTask_attributes = {
  .name = "uartReceiveTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for myQueueUart */
osMessageQueueId_t myQueueUartHandle;
const osMessageQueueAttr_t myQueueUart_attributes = {
  .name = "myQueueUart"
};

void StartLEDTask(void *argument);
void StartUartTransmitTask(void *argument);
void StartUartReceiveTask(void *argument);

void app_init(void)
{
	MX_UART5_Init();
	
	/* create thread new */
	TaskHandle = osThreadNew(StartLEDTask, NULL, &ledTask_attributes);
	UARTTransmitTaskHandle = osThreadNew(StartUartTransmitTask, NULL, &uartTransmitTask_attributes);
	UARTReceiveTaskHandle = osThreadNew(StartUartReceiveTask, NULL, &uartReceiveTask_attributes);
	
	/* Create queue */
	myQueueUartHandle = osMessageQueueNew (16, sizeof(uint16_t), &myQueueUart_attributes);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	
	if(HAL_UART_Transmit_DMA(&huart5, (uint8_t*)aTxStartMessage, 200)!= HAL_OK)
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}
}

void StartUartTransmitTask(void *argument)
{
	for(;;)
	{
		HAL_UART_Transmit_DMA(&huart5, (uint8_t*)aTxStartMessage, 20);
		osDelay(pdMS_TO_TICKS(1000));
	}
}

void StartUartReceiveTask(void *argument)
{
	for(;;)
	{
		osDelay(pdMS_TO_TICKS(1000));
	}
}

void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  osDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END StartDefaultTask */
}

void app_process(void)
{
	
}
