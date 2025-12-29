// 1402/12/28 => 18 March 2024 15:00 => Tiny Modifications
#ifndef __SMODBUS_HAL_H
#define __SMODBUS_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "modbus.h"
#if MODBUS_WITH_RTOS
#include "cmsis_os.h"
#endif

// <<< Use Configuration Wizard in Context Menu >>>
// <q> IWDG_REFRESH_ON_MODBUS_ACTIVITY
#ifndef IWDG_REFRESH_ON_MODBUS_ACTIVITY
#define IWDG_REFRESH_ON_MODBUS_ACTIVITY 0
#endif

// <o> SMODBUS_RX_BUFF_SIZE
#ifndef SMODBUS_RX_BUFF_SIZE
#define SMODBUS_RX_BUFF_SIZE            512
#endif
// <o> SMODBUS_COIL_COUNT
#ifndef SMODBUS_COIL_COUNT
#define SMODBUS_COIL_COUNT              20
#endif
// <o> SMODBUS_DISCRETE_INPUT_COUNT
#ifndef SMODBUS_DISCRETE_INPUT_COUNT
#define SMODBUS_DISCRETE_INPUT_COUNT    20
#endif
// <o> SMODBUS_HOLDING_REGISTER_COUNT
#ifndef SMODBUS_HOLDING_REGISTER_COUNT
#define SMODBUS_HOLDING_REGISTER_COUNT  220
#endif
// <o> SMODBUS_INPUT_REGISTER_COUNT
#ifndef SMODBUS_INPUT_REGISTER_COUNT
#define SMODBUS_INPUT_REGISTER_COUNT    120
#endif
// <o> SMODBUS_FILE_COUNT
#ifndef SMODBUS_FILE_COUNT
#define SMODBUS_FILE_COUNT              2
#endif
// <o> SMODBUS_MAX_SLAVE_PER_UART
#ifndef SMODBUS_MAX_SLAVE_PER_UART
#define SMODBUS_MAX_SLAVE_PER_UART      20
#endif
// <<< end of configuration section >>>
//----------------------------------------
#if (SMODBUS_COIL_COUNT > 0)
typedef struct __smodbus_coil_t
{
    uint16_t                        slave_address;
    uint16_t                        register_address;
    bool                            *pointer;
    void (* read_callback)          (struct __smodbus_coil_t *coil);
    void (* write_callback)         (struct __smodbus_coil_t *coil);
} smodbus_coil_t;
#endif

#if (SMODBUS_DISCRETE_INPUT_COUNT > 0)
typedef struct __smodbus_discrete_input_t
{
    uint16_t                        slave_address;
    uint16_t                        register_address;
    bool                            *pointer;
    void (* read_callback)          (struct __smodbus_discrete_input_t *discrete_input);
} smodbus_discrete_input_t;
#endif

#if (SMODBUS_HOLDING_REGISTER_COUNT > 0)
typedef struct __smodbus_holding_register_t
{
    uint16_t                        slave_address;
    uint16_t                        register_address;
    uint16_t                        *pointer;
    void (* read_callback)          (struct __smodbus_holding_register_t *holding_register);
    void (* write_callback)         (struct __smodbus_holding_register_t *holding_register);
} smodbus_holding_register_t;
#endif

#if (SMODBUS_INPUT_REGISTER_COUNT > 0)
typedef struct __smodbus_input_register_t
{
    uint16_t                        slave_address;
    uint16_t                        register_address;
    uint16_t                        *pointer;
    void (* read_callback)          (struct __smodbus_input_register_t *input_register);
} smodbus_input_register_t;
#endif

#if (SMODBUS_FILE_COUNT > 0)
typedef struct __smodbus_file_t
{
    uint16_t                        slave_address;
    uint16_t                        file_number;
    //void (*read_record_callback)  (struct __smodbus_file_record_t *file, uint16_t record_number, uint8_t* data, uint8_t size); // Not Implemented Yet
    void (* write_record_callback)  (struct __smodbus_file_t *file, uint16_t record_number, uint8_t* data, uint8_t size);
} smodbus_file_t;
#endif

typedef struct
{
    struct
    {
        UART_HandleTypeDef         *huart;
        mx_uartx_init_t             mx_uartx_init;
        modbus_rx_tx_type_t         rx_tx_type;
        tx_enable_t                 tx_enable;
        rx_enable_t                 rx_enable;
    } init;
    modbus_rx_state_t               rx_state;
    uint16_t                        rx_head;
    uint16_t                        rx_tail;
    #if defined(STM32H7)
    ALIGN_32BYTES(uint8_t           rx_buff[SMODBUS_RX_BUFF_SIZE]);
    #else
    uint8_t                         rx_buff[SMODBUS_RX_BUFF_SIZE];
    #endif
    struct
    {
        uint16_t                    rx_ok_req;
        uint16_t                    tx_ok_res;
        uint16_t                    tx_exception_illegal_function;
        uint16_t                    tx_exception_illegal_data_address;
        uint16_t                    tx_exception_illegal_data_value;
        
        uint16_t                    idle;
        uint16_t                    wrong_slave;
        uint16_t                    uart_frame_error;
        uint16_t                    parse_byte_error;
    } cnt;
    modbus_message_t                message;
    #if MODBUS_WITH_RTOS
    osSemaphoreId_t                 tx_semaphore_id;
    osSemaphoreAttr_t               tx_semaphore_attributes;
    
    osSemaphoreId_t                 rx_semaphore_id;
    osSemaphoreAttr_t               rx_semaphore_attributes;
    
    osThreadId_t                    rx_thread_id;
    osThreadAttr_t                  rx_thread_attributes;
    #endif
    
    uint8_t                         slave_address_count;
    uint8_t                         slave_address_list[SMODBUS_MAX_SLAVE_PER_UART];
#if (SMODBUS_COIL_COUNT > 0)
    uint16_t                        coils_count;
    smodbus_coil_t                  coils[SMODBUS_COIL_COUNT];
#endif
#if (SMODBUS_DISCRETE_INPUT_COUNT > 0)
    uint16_t                        discrete_inputs_count;
    smodbus_discrete_input_t        discrete_inputs[SMODBUS_DISCRETE_INPUT_COUNT];
#endif
#if (SMODBUS_HOLDING_REGISTER_COUNT > 0)
    uint16_t                        holding_registers_count;
    smodbus_holding_register_t      holding_registers[SMODBUS_HOLDING_REGISTER_COUNT];
#endif
#if (SMODBUS_INPUT_REGISTER_COUNT > 0)
    uint16_t                        input_registers_count;
    smodbus_input_register_t        input_registers[SMODBUS_INPUT_REGISTER_COUNT];
#endif
#if (SMODBUS_FILE_COUNT > 0)
    uint16_t                        files_count;
    smodbus_file_t                  files[SMODBUS_FILE_COUNT];
#endif
} smodbus_t;
//----------------------------------------
#if (SMODBUS_COIL_COUNT > 0)
bool smodbus_add_coil(smodbus_t *smodbus, smodbus_coil_t *coil);
bool smodbus_add_coils(smodbus_t *smodbus, uint8_t slave_address, uint16_t first_register_address, uint8_t number_of_registers, bool *coil);
#endif

#if (SMODBUS_DISCRETE_INPUT_COUNT > 0)
bool smodbus_add_discrete_input(smodbus_t *smodbus, smodbus_discrete_input_t *discrete_input);
bool smodbus_add_discrete_inputs(smodbus_t *smodbus, uint8_t slave_address, uint16_t first_register_address, uint8_t number_of_registers, bool *discrete_inputs);
#endif

#if (SMODBUS_HOLDING_REGISTER_COUNT > 0)
bool smodbus_add_holding_register(smodbus_t *smodbus, smodbus_holding_register_t *holding_register);
bool smodbus_add_holding_registers(smodbus_t *smodbus, uint8_t slave_address, uint16_t first_register_address, uint8_t number_of_registers, uint16_t *holding_registers);
#endif

#if (SMODBUS_INPUT_REGISTER_COUNT > 0)
bool smodbus_add_input_register(smodbus_t *smodbus, smodbus_input_register_t *input_register);
bool smodbus_add_input_registers(smodbus_t *smodbus, uint8_t slave_address, uint16_t first_register_address, uint8_t number_of_registers, uint16_t *input_registers);
#endif

#if SMODBUS_FILE_COUNT
bool smodbus_add_file(smodbus_t *smodbus, smodbus_file_t *file);
#endif
//----------------------------------------

void smodbus_init(smodbus_t *smodbus);
#if (MODBUS_WITH_RTOS == 0)
void smodbus_rx_handler(smodbus_t *smodbus);
#endif
#ifdef __cplusplus
}
#endif

#endif
