#include "main.h"

osThreadId_t TaskHandle;
osThreadId_t UARTTaskHandle;

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

const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void StartLEDTask(void *argument);
void StartUartTask(void *argument);

void app_init(void)
{
	MX_UART5_Init();
	
	/* create thread new */
	TaskHandle = osThreadNew(StartLEDTask, NULL, &ledTask_attributes);
	UARTTaskHandle = osThreadNew(StartUartTask, NULL, &uartTask_attributes);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	
	if(HAL_UART_Transmit_DMA(&huart5, (uint8_t*)aTxStartMessage, 200)!= HAL_OK)
	{
		/* Transfer error in transmission process */
		Error_Handler();
	}
}

void StartUartTask(void *argument)
{
	for(;;)
	{
		HAL_UART_Transmit_DMA(&huart5, (uint8_t*)aTxStartMessage, 20);
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
