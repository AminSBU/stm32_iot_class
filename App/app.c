#include "main.h"
#include "app.h"

smodbus_t smodbus;

extern UART_HandleTypeDef huart5;

void tx5_enable(void)
{ 
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);  
}
void rx5_enable(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}

void app_init(void)
{
	/* modbus initialize */
	smodbus.init.huart = &huart5;
    smodbus.init.mx_uartx_init = MX_UART5_Init;
    smodbus.init.tx_enable = tx5_enable;
    smodbus.init.rx_enable = rx5_enable;
    smodbus.init.rx_tx_type = MODBUS_DMA;
    smodbus_init(&smodbus);	
}
