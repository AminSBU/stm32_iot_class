#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

void LwIPTask(void *argument)
{
  /* USER CODE BEGIN LwIPTask */
	MX_LWIP_Init();
  /* Infinite loop */
  for(;;)
  {
	  send_data_tcp();
    osDelay(1000);
  }
  /* USER CODE END LwIPTask */
}

void ledStartTask(void *argument)
{
  /* USER CODE BEGIN ledStartTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
    osDelay(1000);
  }
  /* USER CODE END ledStartTask */
}