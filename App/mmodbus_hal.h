// 1402/12/28 => 18 March 2024 15:00 => Tiny Modifications
#ifndef __MMODBUS_HAL_H
#define __MMODBUS_HAL_H

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
// <o> MMODBUS_RX_BUFF_SIZE
#ifndef MMODBUS_RX_BUFF_SIZE
#define MMODBUS_RX_BUFF_SIZE                          256
#endif
// <<< end of configuration section >>>

typedef struct mmodbus_tag
{
    struct
    {
        #if (MODBUS_WITH_RTOS == 0)
        TIM_HandleTypeDef                             *htim;
        uint16_t                                       htim_1ms_cnt;
        #endif
        UART_HandleTypeDef                            *huart;
        mx_uartx_init_t                                mx_uartx_init;
        modbus_rx_tx_type_t                            rx_tx_type;
        tx_enable_t                                    tx_enable;
        rx_enable_t                                    rx_enable;
    } init;
    modbus_rx_state_t                                  rx_state;
    uint16_t                                           rx_head;
    uint16_t                                           rx_tail;
    #if defined(STM32H7)
    ALIGN_32BYTES(uint8_t                              rx_buff[MMODBUS_RX_BUFF_SIZE]);
    #else
    uint8_t                                            rx_buff[MMODBUS_RX_BUFF_SIZE];
    #endif
    struct
    {
        uint16_t                                       tx_ok_req;
        uint16_t                                       rx_ok_res;
        uint16_t                                       rx_exception_illegal_function;
        uint16_t                                       rx_exception_illegal_data_address;
        uint16_t                                       rx_exception_illegal_data_value;
        
        uint16_t                                       idle;
        uint16_t                                       rx_timeout;
        uint16_t                                       uart_frame_error;
        uint16_t                                       parse_byte_error;
    } cnt;
    modbus_message_t                                   message;
    #if MODBUS_WITH_RTOS
    osSemaphoreId_t                                    tx_semaphore_id;
    osSemaphoreAttr_t                                  tx_semaphore_attributes;
    
    osSemaphoreId_t                                    rx_semaphore_id;
    osSemaphoreAttr_t                                  rx_semaphore_attributes;
    #else
    void (* notify_callback)                          (struct mmodbus_tag *mmodbus);
    bool                                              *p_coils;
    bool                                              *p_discrete_inputs;
    uint16_t                                          *p_holding_registers;
    uint16_t                                          *p_input_registers;
    bool                                              *p_file_record;
    bool                                               result_received;
    modbus_result_t                                    result;
    #endif
    void (* write_file_record_callback)               (struct mmodbus_tag *mmodbus, uint8_t *bytes, uint8_t bytes_length);
} mmodbus_t;

void mmodbus_init                                     (mmodbus_t *mmodbus);
#if (MODBUS_WITH_RTOS == 0)
modbus_result_t mmodbus_receive_res                   (mmodbus_t *mmodbus);
typedef void (* mmodbus_notify_callback_t)            (mmodbus_t *mmodbus);
#endif
modbus_result_t mmodbus_read_coils                    (mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, bool *coils,                 uint32_t rx_timeout);
modbus_result_t mmodbus_write_single_coil             (mmodbus_t *mmodbus, uint8_t slave_address, uint16_t       register_address,                               bool coil,                   uint32_t rx_timeout);
modbus_result_t mmodbus_write_multiple_coils          (mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, bool *coils,                 uint32_t rx_timeout);

modbus_result_t mmodbus_read_discrete_inputs          (mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, bool *discrete_inputs,       uint32_t rx_timeout);

modbus_result_t mmodbus_read_holding_registers        (mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, uint16_t *holding_registers, uint32_t rx_timeout);
modbus_result_t mmodbus_write_single_holding_register (mmodbus_t *mmodbus, uint8_t slave_address, uint16_t       register_address,                               uint16_t holding_register,   uint32_t rx_timeout);
modbus_result_t mmodbus_write_multiple_registers      (mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, uint16_t *holding_registers, uint32_t rx_timeout);

modbus_result_t mmodbus_read_input_registers          (mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, uint16_t *input_registers,   uint32_t rx_timeout);

modbus_result_t mmodbus_write_file_record             (mmodbus_t *mmodbus, uint8_t slave_address, uint8_t *bytes, uint8_t bytes_length, uint16_t file_number, uint16_t record_number,         uint32_t rx_timeout);

#ifdef __cplusplus
}
#endif

#endif
