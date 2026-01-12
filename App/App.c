#include "main.h"

osThreadId_t defaultTaskHandle;

const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void StartLEDTask(void *argument);

void app_init(void)
{
	/* create thread new */
	defaultTaskHandle = osThreadNew(StartLEDTask, NULL, &ledTask_attributes);
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
