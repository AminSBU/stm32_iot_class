#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"

void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    send_data_tcp();   // ? safe place
//	  RTT_Test();
//	  counter_rtt++;
//	  printf( "Hello from STM32H7 %u\r\n", counter++);
    osDelay(1000);
  }
  /* USER CODE END 5 */
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