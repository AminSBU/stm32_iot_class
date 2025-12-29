// 1402/12/28 => 18 March 2024 15:00 => Tiny Modifications
#include "main.h"
#include "smodbus_hal.h"
#if IWDG_REFRESH_ON_MODBUS_ACTIVITY
#if defined(STM32F4)
extern IWDG_HandleTypeDef hiwdg;
#elif defined(STM32H7)
extern IWDG_HandleTypeDef hiwdg1;
#define hiwdg hiwdg1
#endif
#endif

smodbus_t *smodbus_list[10];
uint8_t    smodbus_count = 0;
//#####################################################################################################
#if (SMODBUS_COIL_COUNT > 0)
static void smodbus_read_coils(smodbus_t *smodbus);
static void smodbus_write_single_coil(smodbus_t *smodbus);
static void smodbus_write_multiple_coils(smodbus_t *smodbus);
#endif
#if (SMODBUS_DISCRETE_INPUT_COUNT > 0)
static void smodbus_read_discrete_inputs(smodbus_t *smodbus);
#endif
#if (SMODBUS_HOLDING_REGISTER_COUNT > 0)
static void smodbus_read_holding_registers(smodbus_t *smodbus);
static void smodbus_write_single_holding_register(smodbus_t *smodbus);
static void smodbus_write_multiple_registers(smodbus_t *smodbus);
#endif
#if (SMODBUS_INPUT_REGISTER_COUNT > 0)
static void smodbus_read_input_registers(smodbus_t *smodbus);
#endif
static void smodbus_write_file_record(smodbus_t *smodbus);
//##################################################################################################
static inline smodbus_t *mmodbus_find(UART_HandleTypeDef *huart)
{
    smodbus_t *smodbus = NULL;
    for (uint8_t k = 0; k < smodbus_count; k++)
    {
        if (huart->Instance == smodbus_list[k]->init.huart->Instance)
        {
            smodbus = smodbus_list[k];
            break;
        }
    }
    return smodbus;
}
//#####################################################################################################
void smodbus_tx_cplt_callback(UART_HandleTypeDef *huart)
{
    smodbus_t *smodbus = mmodbus_find(huart);
    if (smodbus != NULL)
    {
        #if MODBUS_WITH_RTOS
        osSemaphoreRelease(smodbus->tx_semaphore_id);
        #endif
        if (smodbus->init.rx_enable != NULL)
        {
            smodbus->init.rx_enable();
        }
    }
    UNUSED(huart);
}
//##################################################################################################
static void smodbus_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    smodbus_t *smodbus = mmodbus_find(huart);
    if (smodbus != NULL)
    {
        if (huart->RxEventType != HAL_UART_RXEVENT_HT)
        {
            smodbus->cnt.idle++;
            smodbus->rx_head = Size;
            smodbus->rx_tail = 0;
            #if MODBUS_WITH_RTOS
            osSemaphoreRelease(smodbus->rx_semaphore_id);
            #else
            smodbus_rx_handler(smodbus);
            #endif
        }
    }
    UNUSED(huart);
    UNUSED(Size);
}
//##################################################################################################
void smodbus_error_callback(UART_HandleTypeDef *huart)
{
    smodbus_t *smodbus = mmodbus_find(huart);
    if (smodbus != NULL)
    {
        smodbus->cnt.uart_frame_error++;
        if (smodbus->init.rx_tx_type == MODBUS_DMA)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(smodbus->init.huart, smodbus->rx_buff, SMODBUS_RX_BUFF_SIZE);
        }
        else if (smodbus->init.rx_tx_type == MODBUS_IT)
        {
            HAL_UARTEx_ReceiveToIdle_IT(smodbus->init.huart, smodbus->rx_buff, SMODBUS_RX_BUFF_SIZE);
        }
    }
    UNUSED(huart);
}
//##################################################################################################
void smodbus_rx_thread(void *argument);
void smodbus_init(smodbus_t *smodbus)
{
    smodbus_list[smodbus_count++] = smodbus;
    smodbus->init.mx_uartx_init();
    if (smodbus->init.rx_tx_type == MODBUS_DMA)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(smodbus->init.huart, smodbus->rx_buff, SMODBUS_RX_BUFF_SIZE);
    }
    else if (smodbus->init.rx_tx_type == MODBUS_IT)
    {
        HAL_UARTEx_ReceiveToIdle_IT(smodbus->init.huart, smodbus->rx_buff, SMODBUS_RX_BUFF_SIZE);
    }
    if (smodbus->init.rx_enable != NULL)
    {
        smodbus->init.rx_enable();
    }
    smodbus->init.huart->TxCpltCallback = smodbus_tx_cplt_callback;
    smodbus->init.huart->RxEventCallback = smodbus_rx_event_callback;
    smodbus->init.huart->ErrorCallback = smodbus_error_callback;
    smodbus->slave_address_count = 0;
    
    #if MODBUS_WITH_RTOS
    smodbus->tx_semaphore_attributes.name = "smodbus_tx_semaphore";
    smodbus->rx_semaphore_attributes.name = "smodbus_rx_semaphore";
    smodbus->tx_semaphore_id = osSemaphoreNew(1, 0, &smodbus->tx_semaphore_attributes);
    smodbus->rx_semaphore_id = osSemaphoreNew(1, 0, &smodbus->rx_semaphore_attributes);
    
    smodbus->rx_thread_attributes.name = "smodbus_rx_thread";
    smodbus->rx_thread_attributes.stack_size = 256 * 4;
    smodbus->rx_thread_attributes.priority = (osPriority_t) osPriorityAboveNormal;
    smodbus->rx_thread_id = osThreadNew(smodbus_rx_thread, smodbus, &smodbus->rx_thread_attributes);
    #endif
}
//#####################################################################################################
static void smodbus_slave_address_list_add(smodbus_t *smodbus, uint8_t slave_address)
{
    bool slave_address_is_new = true;
    for (uint8_t k = 0; k < smodbus->slave_address_count; k++)
    {
        if (smodbus->slave_address_list[k] == slave_address)
        {
            slave_address_is_new = false;
        }
    }
    if (slave_address_is_new && (smodbus->slave_address_count < SMODBUS_MAX_SLAVE_PER_UART))
    {
        smodbus->slave_address_list[smodbus->slave_address_count] = slave_address;
        smodbus->slave_address_count++;
    }
}
//#####################################################################################################
static inline void smodbus_parse_byte(smodbus_t *smodbus, uint8_t b)
{
    static uint16_t case_cnt = 0;
    if (smodbus->message.req.buff_index < MODBUS_REQ_RES_SIZE)
    {
        smodbus->message.req.buff[smodbus->message.req.buff_index] = b;
        smodbus->message.req.buff_index++;
    }
    switch (smodbus->rx_state)
    {
        case WAIT_FOR_SLAVE_ADDRESS:
        {
            bool slave_address_is_correct = false;
            for (uint8_t k = 0; k < smodbus->slave_address_count; k++)
            {
                if (b == smodbus->slave_address_list[k])
                {
                    slave_address_is_correct = true;
                    break;
                }
            }
            if (!slave_address_is_correct)
            {
                smodbus->cnt.wrong_slave++;
                goto _slave_address_is_not_correct;
            }
            smodbus->message.req.slave_address = b;
            smodbus->rx_state = WAIT_FOR_FUNCTION_CODE;
        } break;
        case WAIT_FOR_FUNCTION_CODE:
        {
            smodbus->message.req.function_code = b;
            switch (b)
            {
                case read_coils:
                case read_discrete_inputs:
                case read_holding_registers:
                case read_input_registers:
                case write_single_coil:
                case write_single_holding_register:
                case write_multiple_coils:
                case write_multiple_registers:
                {
                    smodbus->rx_state = WAIT_FOR_FIRST_REGISTER_ADDRESS;
                } break;
                case write_file_record:
                {
                    smodbus->rx_state = WAIT_FOR_DATA_LENGTH;
                } break;
                default:
                {
                    goto _error;
                } //break;
            }
            case_cnt = 0;
        } break;
        case WAIT_FOR_EXCEPTION_CODE:
        {
            // Does not receive by modbus slave
        } break;
        // write_file_record
        case WAIT_FOR_DATA_LENGTH:
        {
            smodbus->message.req.data_length = b;
            smodbus->rx_state = WAIT_FOR_REFERENCE_TYPE;
        } break;
        case WAIT_FOR_REFERENCE_TYPE:
        {
            if (b != 6)
            {
                goto _error;
            }
            smodbus->message.req.reference_type = b;
            smodbus->rx_state = WAIT_FOR_FILE_NUMBER;
        } break;
        case WAIT_FOR_FILE_NUMBER:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&smodbus->message.req.file_number)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&smodbus->message.req.file_number)[0] = b;
                smodbus->rx_state = WAIT_FOR_RECORD_NUMBER;
                case_cnt = 0;
            }
        } break;
        case WAIT_FOR_RECORD_NUMBER:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&smodbus->message.req.record_number)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&smodbus->message.req.record_number)[0] = b;
                smodbus->rx_state = WAIT_FOR_RECORD_LENGTH;
                case_cnt = 0;
            }
        } break;
        case WAIT_FOR_RECORD_LENGTH:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&smodbus->message.req.record_length)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&smodbus->message.req.record_length)[0] = b;
                smodbus->rx_state = WAIT_FOR_RECORD_DATA;
                case_cnt = 0;
                if ((smodbus->message.req.data_length - 7) != (2 * smodbus->message.req.record_length))
                {
                    goto _error;
                }
            }
        } break;
        case WAIT_FOR_RECORD_DATA:
        {
            smodbus->message.req.data[case_cnt] = b;
            case_cnt++;
            if (case_cnt == (smodbus->message.req.data_length - 7))
            {
                smodbus->rx_state = WAIT_FOR_CRC;
                case_cnt = 0;
            }
        } break;
        // -----------------
        case WAIT_FOR_FIRST_REGISTER_ADDRESS:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&smodbus->message.req.first_register_address)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&smodbus->message.req.first_register_address)[0] = b;
                smodbus->rx_state = WAIT_FOR_NUMBER_OF_REGISTERS;
                case_cnt = 0;
            }
        } break;
        case WAIT_FOR_NUMBER_OF_REGISTERS:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&smodbus->message.req.number_of_registers)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&smodbus->message.req.number_of_registers)[0] = b;
                if ((smodbus->message.req.function_code == write_multiple_coils) ||
                    (smodbus->message.req.function_code == write_multiple_registers))
                {
                    smodbus->rx_state = WAIT_FOR_BYTE_COUNT;
                }
                else
                {
                    smodbus->rx_state = WAIT_FOR_CRC;
                }
                case_cnt = 0;
            }
        } break;
        // -----------------
        case WAIT_FOR_BYTE_COUNT:
        {
            if (b == 0)
            {
                 goto _error;
            }
            smodbus->message.req.byte_count = b;
            smodbus->rx_state = WAIT_FOR_DATA;
        } break;
        case WAIT_FOR_DATA:
        {
            smodbus->message.req.data[case_cnt] = b;
            case_cnt++;
            if (case_cnt == smodbus->message.req.byte_count)
            {
                smodbus->rx_state = WAIT_FOR_CRC;
                case_cnt = 0;
            }
        } break;
        // -----------------
        case WAIT_FOR_CRC:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&smodbus->message.req.crc_got)[0] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&smodbus->message.req.crc_got)[1] = b;
                smodbus->message.req.crc_cal = modbus_crc16(smodbus->message.req.buff, smodbus->message.req.buff_index - 2);
                if (smodbus->message.req.crc_cal == smodbus->message.req.crc_got)
                {
#if IWDG_REFRESH_ON_MODBUS_ACTIVITY
                    HAL_IWDG_Refresh(&hiwdg);
#endif
                    smodbus->cnt.rx_ok_req++;
                    switch (smodbus->message.req.function_code)
                    {
                        #if (SMODBUS_COIL_COUNT > 0)
                        case read_coils:
                        {
                            smodbus_read_coils(smodbus);
                        } break;
                        case write_single_coil:
                        {
                            smodbus_write_single_coil(smodbus);
                        } break;
                        case write_multiple_coils:
                        {
                            smodbus_write_multiple_coils(smodbus);
                        } break;
                        #endif
                        #if (SMODBUS_DISCRETE_INPUT_COUNT > 0)
                        case read_discrete_inputs:
                        {
                            smodbus_read_discrete_inputs(smodbus);
                        } break;
                        #endif
                        #if (SMODBUS_HOLDING_REGISTER_COUNT > 0)
                        case read_holding_registers:
                        {
                            smodbus_read_holding_registers(smodbus);
                        } break;
                        case write_single_holding_register:
                        {
                            smodbus_write_single_holding_register(smodbus);
                        } break;
                        case write_multiple_registers:
                        {
                            smodbus_write_multiple_registers(smodbus);
                        } break;
                        #endif
                        #if (SMODBUS_INPUT_REGISTER_COUNT > 0)
                        case read_input_registers:
                        {
                            smodbus_read_input_registers(smodbus);
                        } break;
                        #endif
                        case write_file_record:
                        {
                            smodbus_write_file_record(smodbus);
                        } break;
                        default:
                        {
                            goto _error;
                        } //break;
                    }
                    goto _repeat;
                }
                else
                {
                    goto _error;
                }
            }
        } break;
    }
    return;
    _error:
    smodbus->cnt.parse_byte_error++;
    _slave_address_is_not_correct:
    smodbus->rx_tail = (smodbus->rx_head - 1) % SMODBUS_RX_BUFF_SIZE;
    _repeat:
    smodbus->message.req.buff_index = 0;
    memset(smodbus->message.req.buff, 0, smodbus->message.req.buff_index);
    case_cnt = 0;
    smodbus->rx_state = WAIT_FOR_SLAVE_ADDRESS;
    return;
}
//#####################################################################################################
static void smodbus_send_raw(smodbus_t *smodbus, uint8_t *data, uint16_t size)
{
    uint32_t startTime = HAL_GetTick();
    if (smodbus->init.tx_enable != NULL)
    {
        smodbus->init.tx_enable();
    }
    if (smodbus->init.rx_tx_type == MODBUS_DMA)
    {
        #if defined(STM32H7)
        SCB_CleanDCache_by_Addr((uint32_t *)data, size);
        #endif
        HAL_UART_Transmit_DMA(smodbus->init.huart, data, size);
    }
    else if (smodbus->init.rx_tx_type == MODBUS_IT)
    {
        HAL_UART_Transmit_IT(smodbus->init.huart, data, size);
    }
    #if MODBUS_WITH_RTOS
    osSemaphoreAcquire(smodbus->tx_semaphore_id, 100);
    #endif
    if (data[1] & 0x80)
    {
        switch (data[2])
        {
            case MODBUS_RESULT_ILLEGAL_FUNCTION:
            {
                smodbus->cnt.tx_exception_illegal_function++;
            } break;
            case MODBUS_RESULT_ILLEGAL_DATA_ADDRESS:
            {
                smodbus->cnt.tx_exception_illegal_data_address++;
            } break;
            case MODBUS_RESULT_ILLEGAL_DATA_VALUE:
            {
                smodbus->cnt.tx_exception_illegal_data_value++;
            } break;
            default:
            {
                __NOP();
            } break;
        }
    }
    else
    {
        smodbus->cnt.tx_ok_res++;
    }
}
//##################################################################################################
#if MODBUS_WITH_RTOS
void smodbus_rx_thread(void *argument)
{
    smodbus_t *smodbus = (smodbus_t *)argument;
    while (1)
    {
        osSemaphoreAcquire(smodbus->rx_semaphore_id, osWaitForever);
#else
void smodbus_rx_handler(smodbus_t *smodbus)
{
    {
#endif
        if (smodbus->init.rx_tx_type == MODBUS_DMA)
        {
            #if defined(STM32H7)
            SCB_InvalidateDCache_by_Addr ((uint32_t *)&smodbus->rx_buff[smodbus->rx_tail], smodbus->rx_head - smodbus->rx_tail);
            #endif
        }
        while (smodbus->rx_tail != smodbus->rx_head)
        {
            smodbus_parse_byte(smodbus, smodbus->rx_buff[smodbus->rx_tail]);
            smodbus->rx_tail++;
            smodbus->rx_tail %= SMODBUS_RX_BUFF_SIZE;
        }
        if (smodbus->init.rx_tx_type == MODBUS_DMA)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(smodbus->init.huart, smodbus->rx_buff, SMODBUS_RX_BUFF_SIZE);
        }
        else if (smodbus->init.rx_tx_type == MODBUS_IT)
        {
            HAL_UARTEx_ReceiveToIdle_IT(smodbus->init.huart, smodbus->rx_buff, SMODBUS_RX_BUFF_SIZE);
        }
    }
}
//##################################################################################################
//##################################################################################################
#if (SMODBUS_COIL_COUNT > 0)
bool smodbus_add_coil(smodbus_t *smodbus, smodbus_coil_t *coil)
{
    if (smodbus->coils_count < SMODBUS_COIL_COUNT)
    {
        smodbus_slave_address_list_add(smodbus, coil->slave_address);
        smodbus->coils[smodbus->coils_count] = *coil;
        smodbus->coils_count++;
        return true;
    }
    else
    {
        return false;
    }
}
//##################################################################################################
bool smodbus_add_coils(smodbus_t *smodbus, uint8_t slave_address, uint16_t first_register_address, uint8_t number_of_registers, bool *coil)
{
    if ((smodbus->coils_count + number_of_registers) <= SMODBUS_COIL_COUNT)
    {
        smodbus_slave_address_list_add(smodbus, slave_address);
        for (uint16_t k = 0; k < number_of_registers; k++)
        {
            smodbus->coils[smodbus->coils_count].slave_address = slave_address;
            smodbus->coils[smodbus->coils_count].register_address = first_register_address + k;
            smodbus->coils[smodbus->coils_count].pointer = &coil[k];
            smodbus->coils[smodbus->coils_count].read_callback = NULL;
            smodbus->coils[smodbus->coils_count].write_callback = NULL;
            smodbus->coils_count++;
        }
        return true;
    }
    else
    {
        return false;
    }
}
//##################################################################################################
static void smodbus_send_exception(smodbus_t *smodbus, modbus_result_t result)
{
    smodbus->message.res.slave_address          = smodbus->message.req.slave_address;
    smodbus->message.res.function_code          = 0x80 | smodbus->message.req.function_code;
    smodbus->message.res.exception_code         = result;
    //--
    smodbus->message.res.buff_index = 0;
    smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.slave_address;
    smodbus->message.res.buff_index++;
    smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.function_code;
    smodbus->message.res.buff_index++;
    smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.exception_code;
    smodbus->message.res.buff_index++;
    //--
    smodbus->message.res.crc_cal                = modbus_crc16(smodbus->message.res.buff, smodbus->message.res.buff_index);
    memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&smodbus->message.res.crc_cal, 2);
    smodbus->message.res.buff_index += 2;
    smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
}
//##################################################################################################
static void smodbus_read_coils(smodbus_t *smodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    if (smodbus->coils_count == 0)
    {
        result = MODBUS_RESULT_ILLEGAL_FUNCTION;
    }
    else
    {
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            bool is_register_available = false;
            for (uint16_t k = 0; k < smodbus->coils_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->coils[k].slave_address) && (register_address == smodbus->coils[k].register_address))
                {
                    is_register_available = true;
                    break;
                }
            }
            if (!is_register_available)
            {
                result = MODBUS_RESULT_ILLEGAL_DATA_ADDRESS;
                break;
            }
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        bool     coil_values[smodbus->message.req.number_of_registers];
        uint16_t coil_index = 0;
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            for (uint16_t k = 0; k < smodbus->coils_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->coils[k].slave_address) && (register_address == smodbus->coils[k].register_address))
                {
                    if (smodbus->coils[k].read_callback != NULL)
                    {
                        smodbus->coils[k].read_callback(&smodbus->coils[k]);
                    }
                    coil_values[coil_index] = *smodbus->coils[k].pointer;
                    coil_index++;
                }
            }
        }
        smodbus->message.res.slave_address          = smodbus->message.req.slave_address;
        smodbus->message.res.function_code          = smodbus->message.req.function_code;
        smodbus->message.res.byte_count             = modbus_bool_array_to_byte_array(coil_values, smodbus->message.req.number_of_registers, smodbus->message.res.data);
        //--
        smodbus->message.res.buff_index = 0;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.slave_address;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.function_code;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.byte_count;
        smodbus->message.res.buff_index++;
        for (uint16_t k = 0; k < smodbus->message.res.byte_count; k++)
        {
            smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.data[k];
            smodbus->message.res.buff_index++;
        }
        //--
        smodbus->message.res.crc_cal                = modbus_crc16(smodbus->message.res.buff, smodbus->message.res.buff_index);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&smodbus->message.res.crc_cal, 2);
        smodbus->message.res.buff_index += 2;
        smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
    }
    else
    {
        smodbus_send_exception(smodbus, result);
    }
}
//##################################################################################################
static void smodbus_write_single_coil(smodbus_t *smodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    if (smodbus->coils_count == 0)
    {
        result = MODBUS_RESULT_ILLEGAL_FUNCTION;
    }
    else
    {
        bool is_register_available = false;
        for (uint16_t k = 0; k < smodbus->coils_count; k++)
        {
            if ((smodbus->message.req.slave_address == smodbus->coils[k].slave_address) && (smodbus->message.req.first_register_address == smodbus->coils[k].register_address))
            {
                is_register_available = true;
                break;
            }
        }
        if (!is_register_available)
        {
            result = MODBUS_RESULT_ILLEGAL_DATA_ADDRESS;
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        for (uint16_t k = 0; k < smodbus->coils_count; k++)
        {
            if ((smodbus->message.req.slave_address == smodbus->coils[k].slave_address) && (smodbus->message.req.first_register_address == smodbus->coils[k].register_address))
            {
                *smodbus->coils[k].pointer = (smodbus->message.req.number_of_registers != 0);
            }
        }
        smodbus->message.res.buff_index = smodbus->message.req.buff_index;
        memcpy(smodbus->message.res.buff, smodbus->message.req.buff, smodbus->message.req.buff_index);
        smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
        for (uint16_t k = 0; k < smodbus->coils_count; k++)
        {
            if (smodbus->message.req.first_register_address == smodbus->coils[k].register_address)
            {
                if (smodbus->coils[k].write_callback != NULL)
                {
                    smodbus->coils[k].write_callback(&smodbus->coils[k]);
                }
            }
        }
    }
    else
    {
        smodbus_send_exception(smodbus, result);
    }
}
//##################################################################################################
static void smodbus_write_multiple_coils(smodbus_t *smodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    if (smodbus->coils_count == 0)
    {
        result = MODBUS_RESULT_ILLEGAL_FUNCTION;
    }
    else
    {
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            bool is_register_available = false;
            for (uint16_t k = 0; k < smodbus->coils_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->coils[k].slave_address) && (register_address == smodbus->coils[k].register_address))
                {
                    is_register_available = true;
                    break;
                }
            }
            if (!is_register_available)
            {
                result = MODBUS_RESULT_ILLEGAL_DATA_ADDRESS;
                break;
            }
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        bool     coil_values[smodbus->message.req.number_of_registers];
        modbus_byte_array_to_bool_array(smodbus->message.req.data, smodbus->message.req.byte_count, coil_values, smodbus->message.req.number_of_registers);
        uint16_t coil_index = 0;
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            for (uint16_t k = 0; k < smodbus->coils_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->coils[k].slave_address) && (register_address == smodbus->coils[k].register_address))
                {
                    *smodbus->coils[k].pointer = coil_values[coil_index];
                    coil_index++;
                }
            }
        }
        smodbus->message.res.slave_address          = smodbus->message.req.slave_address;
        smodbus->message.res.function_code          = smodbus->message.req.function_code;
        smodbus->message.res.first_register_address = smodbus->message.req.first_register_address;
        smodbus->message.res.number_of_registers    = smodbus->message.req.number_of_registers;
        //--
        smodbus->message.res.buff_index = 0;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.slave_address;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.function_code;
        smodbus->message.res.buff_index++;
        uint16_t temp = SWAP16(smodbus->message.res.first_register_address);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&temp, 2);
        smodbus->message.res.buff_index += 2;
        temp = SWAP16(smodbus->message.res.number_of_registers);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&temp, 2);
        smodbus->message.res.buff_index += 2;
        //--
        smodbus->message.res.crc_cal                = modbus_crc16(smodbus->message.res.buff, smodbus->message.res.buff_index);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&smodbus->message.res.crc_cal, 2);
        smodbus->message.res.buff_index += 2;
        smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            for (uint16_t k = 0; k < smodbus->coils_count; k++)
            {
                if (register_address == smodbus->coils[k].register_address)
                {
                    if (smodbus->coils[k].write_callback != NULL)
                    {
                        smodbus->coils[k].write_callback(&smodbus->coils[k]);
                    }
                }
            }
        }
    }
    else
    {
        smodbus_send_exception(smodbus, result);
    }
}
#endif
//##################################################################################################
//##################################################################################################
#if (SMODBUS_DISCRETE_INPUT_COUNT > 0)
bool smodbus_add_discrete_input(smodbus_t *smodbus, smodbus_discrete_input_t *discrete_input)
{
    if (smodbus->discrete_inputs_count < SMODBUS_DISCRETE_INPUT_COUNT)
    {
        smodbus_slave_address_list_add(smodbus, discrete_input->slave_address);
        smodbus->discrete_inputs[smodbus->discrete_inputs_count] = *discrete_input;
        smodbus->discrete_inputs_count++;
        return true;
    }
    else
    {
        return false;
    }
}
//##################################################################################################
bool smodbus_add_discrete_inputs(smodbus_t *smodbus, uint8_t slave_address, uint16_t first_register_address, uint8_t number_of_registers, bool *discrete_inputs)
{
    if ((smodbus->discrete_inputs_count + number_of_registers) <= SMODBUS_DISCRETE_INPUT_COUNT)
    {
        smodbus_slave_address_list_add(smodbus, slave_address);
        for (uint16_t k = 0; k < number_of_registers; k++)
        {
            smodbus->discrete_inputs[smodbus->discrete_inputs_count].slave_address = slave_address;
            smodbus->discrete_inputs[smodbus->discrete_inputs_count].register_address = first_register_address + k;
            smodbus->discrete_inputs[smodbus->discrete_inputs_count].pointer = &discrete_inputs[k];
            smodbus->discrete_inputs[smodbus->discrete_inputs_count].read_callback = NULL;
            smodbus->discrete_inputs_count++;
        }
        return true;
    }
    else
    {
        return false;
    }
}
//##################################################################################################
static void smodbus_read_discrete_inputs(smodbus_t *smodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    if (smodbus->discrete_inputs_count == 0)
    {
        result = MODBUS_RESULT_ILLEGAL_FUNCTION;
    }
    else
    {
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            bool is_register_available = false;
            for (uint16_t k = 0; k < smodbus->discrete_inputs_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->discrete_inputs[k].slave_address) && (register_address == smodbus->discrete_inputs[k].register_address))
                {
                    is_register_available = true;
                    break;
                }
            }
            if (!is_register_available)
            {
                result = MODBUS_RESULT_ILLEGAL_DATA_ADDRESS;
                break;
            }
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        bool     discrete_input_values[smodbus->message.req.number_of_registers];
        uint16_t discrete_input_index = 0;
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            for (uint16_t k = 0; k < smodbus->discrete_inputs_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->discrete_inputs[k].slave_address) && (register_address == smodbus->discrete_inputs[k].register_address))
                {
                    if (smodbus->discrete_inputs[k].read_callback != NULL)
                    {
                        smodbus->discrete_inputs[k].read_callback(&smodbus->discrete_inputs[k]);
                    }
                    discrete_input_values[discrete_input_index] = *smodbus->discrete_inputs[k].pointer;
                    discrete_input_index++;
                }
            }
        }
        smodbus->message.res.slave_address          = smodbus->message.req.slave_address;
        smodbus->message.res.function_code          = smodbus->message.req.function_code;
        smodbus->message.res.byte_count             = modbus_bool_array_to_byte_array(discrete_input_values, smodbus->message.req.number_of_registers, smodbus->message.res.data);
        smodbus->message.res.first_register_address = smodbus->message.req.first_register_address;
        smodbus->message.res.number_of_registers    = smodbus->message.req.number_of_registers;
        smodbus->message.res.crc_cal                = 0;
        //--
        smodbus->message.res.buff_index = 0;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.slave_address;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.function_code;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.byte_count;
        smodbus->message.res.buff_index++;
        for (uint16_t k = 0; k < smodbus->message.res.byte_count; k++)
        {
            smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.data[k];
            smodbus->message.res.buff_index++;
        }
        //--
        smodbus->message.res.crc_cal                = modbus_crc16(smodbus->message.res.buff, smodbus->message.res.buff_index);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&smodbus->message.res.crc_cal, 2);
        smodbus->message.res.buff_index += 2;
        smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
    }
    else
    {
        smodbus_send_exception(smodbus, result);
    }
}
#endif
//##################################################################################################
//##################################################################################################
#if (SMODBUS_HOLDING_REGISTER_COUNT > 0)
bool smodbus_add_holding_register(smodbus_t *smodbus, smodbus_holding_register_t *holding_register)
{
    if (smodbus->holding_registers_count < SMODBUS_HOLDING_REGISTER_COUNT)
    {
        smodbus_slave_address_list_add(smodbus, holding_register->slave_address);
        smodbus->holding_registers[smodbus->holding_registers_count] = *holding_register;
        smodbus->holding_registers_count++;
        return true;
    }
    else
    {
        return false;
    }
}
//##################################################################################################
bool smodbus_add_holding_registers(smodbus_t *smodbus, uint8_t slave_address, uint16_t first_register_address, uint8_t number_of_registers, uint16_t *holding_registers)
{
    if ((smodbus->holding_registers_count + number_of_registers) <= SMODBUS_HOLDING_REGISTER_COUNT)
    {
        smodbus_slave_address_list_add(smodbus, slave_address);
        for (uint16_t k = 0; k < number_of_registers; k++)
        {
            smodbus->holding_registers[smodbus->holding_registers_count].slave_address = slave_address;
            smodbus->holding_registers[smodbus->holding_registers_count].register_address = first_register_address + k;
            smodbus->holding_registers[smodbus->holding_registers_count].pointer = &holding_registers[k];
            smodbus->holding_registers[smodbus->holding_registers_count].read_callback = NULL;
            smodbus->holding_registers[smodbus->holding_registers_count].write_callback = NULL;
            smodbus->holding_registers_count++;
        }
        return true;
    }
    else
    {
        return false;
    }
}
//##################################################################################################
static void smodbus_read_holding_registers(smodbus_t *smodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    if (smodbus->holding_registers_count == 0)
    {
        result = MODBUS_RESULT_ILLEGAL_FUNCTION;
    }
    else
    {
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            bool is_register_available = false;
            for (uint16_t k = 0; k < smodbus->holding_registers_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->holding_registers[k].slave_address) && (register_address == smodbus->holding_registers[k].register_address))
                {
                    is_register_available = true;
                    break;
                }
            }
            if (!is_register_available)
            {
                result = MODBUS_RESULT_ILLEGAL_DATA_ADDRESS;
                break;
            }
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        smodbus->message.res.number_of_registers = 0;
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            for (uint16_t k = 0; k < smodbus->holding_registers_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->holding_registers[k].slave_address) && (register_address == smodbus->holding_registers[k].register_address))
                {
                    if (smodbus->holding_registers[k].read_callback != NULL)
                    {
                        smodbus->holding_registers[k].read_callback(&smodbus->holding_registers[k]);
                    }
                    uint16_t temp = SWAP16(*smodbus->holding_registers[k].pointer);
                    memcpy(&smodbus->message.res.data[2 * smodbus->message.res.number_of_registers], (uint8_t *)&temp, 2);
                    smodbus->message.res.number_of_registers++;
                }
            }
        }
        smodbus->message.res.slave_address          = smodbus->message.req.slave_address;
        smodbus->message.res.function_code          = smodbus->message.req.function_code;
        smodbus->message.res.byte_count             = 2 * smodbus->message.req.number_of_registers;
        //--
        smodbus->message.res.buff_index = 0;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.slave_address;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.function_code;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.byte_count;
        smodbus->message.res.buff_index++;
        for (uint16_t k = 0; k < smodbus->message.res.byte_count; k++)
        {
            smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.data[k];
            smodbus->message.res.buff_index++;
        }
        //--
        smodbus->message.res.crc_cal                = modbus_crc16(smodbus->message.res.buff, smodbus->message.res.buff_index);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&smodbus->message.res.crc_cal, 2);
        smodbus->message.res.buff_index += 2;
        smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
    }
    else
    {
        smodbus_send_exception(smodbus, result);
    }
}
//##################################################################################################
static void smodbus_write_single_holding_register(smodbus_t *smodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    if (smodbus->holding_registers_count == 0)
    {
        result = MODBUS_RESULT_ILLEGAL_FUNCTION;
    }
    else
    {
        bool is_register_available = false;
        for (uint16_t k = 0; k < smodbus->holding_registers_count; k++)
        {
            if ((smodbus->message.req.slave_address == smodbus->holding_registers[k].slave_address) && (smodbus->message.req.first_register_address == smodbus->holding_registers[k].register_address))
            {
                is_register_available = true;
                break;
            }
        }
        if (!is_register_available)
        {
            result = MODBUS_RESULT_ILLEGAL_DATA_ADDRESS;
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        for (uint16_t k = 0; k < smodbus->holding_registers_count; k++)
        {
            if ((smodbus->message.req.slave_address == smodbus->holding_registers[k].slave_address) && (smodbus->message.req.first_register_address == smodbus->holding_registers[k].register_address))
            {
                *smodbus->holding_registers[k].pointer = smodbus->message.req.number_of_registers;
            }
        }
        smodbus->message.res.buff_index = smodbus->message.req.buff_index;
        memcpy(smodbus->message.res.buff, smodbus->message.req.buff, smodbus->message.req.buff_index);
        smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
        for (uint16_t k = 0; k < smodbus->holding_registers_count; k++)
        {
            if (smodbus->message.req.first_register_address == smodbus->holding_registers[k].register_address)
            {
                if (smodbus->holding_registers[k].write_callback != NULL)
                {
                    smodbus->holding_registers[k].write_callback(&smodbus->holding_registers[k]);
                }
            }
        }
    }
    else
    {
        smodbus_send_exception(smodbus, result);
    }
}
//##################################################################################################
static void smodbus_write_multiple_registers(smodbus_t *smodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    if (smodbus->holding_registers_count == 0)
    {
        result = MODBUS_RESULT_ILLEGAL_FUNCTION;
    }
    else
    {
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            bool is_register_available = false;
            for (uint16_t k = 0; k < smodbus->holding_registers_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->holding_registers[k].slave_address) && (register_address == smodbus->holding_registers[k].register_address))
                {
                    is_register_available = true;
                    break;
                }
            }
            if (!is_register_available)
            {
                result = MODBUS_RESULT_ILLEGAL_DATA_ADDRESS;
                break;
            }
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        smodbus->message.res.number_of_registers = 0;
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            for (uint16_t k = 0; k < smodbus->holding_registers_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->holding_registers[k].slave_address) && (register_address == smodbus->holding_registers[k].register_address))
                {
                    *smodbus->holding_registers[k].pointer = SWAP16(*(uint16_t *)&smodbus->message.req.data[2 * smodbus->message.res.number_of_registers]);
                    smodbus->message.res.number_of_registers++;
                }
            }
        }
        smodbus->message.res.slave_address          = smodbus->message.req.slave_address;
        smodbus->message.res.function_code          = smodbus->message.req.function_code;
        smodbus->message.res.first_register_address = smodbus->message.req.first_register_address;
        smodbus->message.res.number_of_registers    = smodbus->message.req.number_of_registers;
        //--
        smodbus->message.res.buff_index = 0;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.slave_address;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.function_code;
        smodbus->message.res.buff_index++;
        uint16_t temp = SWAP16(smodbus->message.res.first_register_address);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&temp, 2);
        smodbus->message.res.buff_index += 2;
        temp = SWAP16(smodbus->message.res.number_of_registers);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&temp, 2);
        smodbus->message.res.buff_index += 2;
        //--
        smodbus->message.res.crc_cal                = modbus_crc16(smodbus->message.res.buff, smodbus->message.res.buff_index);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&smodbus->message.res.crc_cal, 2);
        smodbus->message.res.buff_index += 2;
        smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            for (uint16_t k = 0; k < smodbus->holding_registers_count; k++)
            {
                if (register_address == smodbus->holding_registers[k].register_address)
                {
                    if (smodbus->holding_registers[k].write_callback != NULL)
                    {
                        smodbus->holding_registers[k].write_callback(&smodbus->holding_registers[k]);
                    }
                }
            }
        }
    }
    else
    {
        smodbus_send_exception(smodbus, result);
    }
}
#endif
//##################################################################################################
//##################################################################################################
#if (SMODBUS_INPUT_REGISTER_COUNT > 0)
bool smodbus_add_input_register(smodbus_t *smodbus, smodbus_input_register_t *input_register)
{
    if (smodbus->input_registers_count < SMODBUS_INPUT_REGISTER_COUNT)
    {
        smodbus_slave_address_list_add(smodbus, input_register->slave_address);
        smodbus->input_registers[smodbus->input_registers_count] = *input_register;
        smodbus->input_registers_count++;
        return true;
    }
    else
    {
        return false;
    }
}
//##################################################################################################
bool smodbus_add_input_registers(smodbus_t *smodbus, uint8_t slave_address, uint16_t first_register_address, uint8_t number_of_registers, uint16_t *input_registers)
{
    if ((smodbus->input_registers_count + number_of_registers) <= SMODBUS_INPUT_REGISTER_COUNT)
    {
        smodbus_slave_address_list_add(smodbus, slave_address);
        for (uint16_t k = 0; k < number_of_registers; k++)
        {
            smodbus->input_registers[smodbus->input_registers_count].slave_address = slave_address;
            smodbus->input_registers[smodbus->input_registers_count].register_address = first_register_address + k;
            smodbus->input_registers[smodbus->input_registers_count].pointer = &input_registers[k];
            smodbus->input_registers[smodbus->input_registers_count].read_callback = NULL;
            smodbus->input_registers_count++;
        }
        return true;
    }
    else
    {
        return false;
    }
}
//##################################################################################################
static void smodbus_read_input_registers(smodbus_t *smodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    if (smodbus->input_registers_count == 0)
    {
        result = MODBUS_RESULT_ILLEGAL_FUNCTION;
    }
    else
    {
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            bool is_register_available = false;
            for (uint16_t k = 0; k < smodbus->input_registers_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->input_registers[k].slave_address) && (register_address == smodbus->input_registers[k].register_address))
                {
                    is_register_available = true;
                    break;
                }
            }
            if (!is_register_available)
            {
                result = MODBUS_RESULT_ILLEGAL_DATA_ADDRESS;
                break;
            }
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        smodbus->message.res.number_of_registers = 0;
        for (uint16_t register_address = smodbus->message.req.first_register_address;
                      register_address < (smodbus->message.req.first_register_address + smodbus->message.req.number_of_registers);
                      register_address++)
        {
            for (uint16_t k = 0; k < smodbus->input_registers_count; k++)
            {
                if ((smodbus->message.req.slave_address == smodbus->input_registers[k].slave_address) && (register_address == smodbus->input_registers[k].register_address))
                {
                    if (smodbus->input_registers[k].read_callback != NULL)
                    {
                        smodbus->input_registers[k].read_callback(&smodbus->input_registers[k]);
                    }
                    uint16_t temp = SWAP16(*smodbus->input_registers[k].pointer);
                    memcpy(&smodbus->message.res.data[2 * smodbus->message.res.number_of_registers], (uint8_t *)&temp, 2);
                    smodbus->message.res.number_of_registers++;
                }
            }
        }
        smodbus->message.res.slave_address          = smodbus->message.req.slave_address;
        smodbus->message.res.function_code          = smodbus->message.req.function_code;
        smodbus->message.res.byte_count             = 2 * smodbus->message.req.number_of_registers;
        //--
        smodbus->message.res.buff_index = 0;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.slave_address;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.function_code;
        smodbus->message.res.buff_index++;
        smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.byte_count;
        smodbus->message.res.buff_index++;
        for (uint16_t k = 0; k < smodbus->message.res.byte_count; k++)
        {
            smodbus->message.res.buff[smodbus->message.res.buff_index] = smodbus->message.res.data[k];
            smodbus->message.res.buff_index++;
        }
        //--
        smodbus->message.res.crc_cal                = modbus_crc16(smodbus->message.res.buff, smodbus->message.res.buff_index);
        memcpy(&smodbus->message.res.buff[smodbus->message.res.buff_index], (uint8_t *)&smodbus->message.res.crc_cal, 2);
        smodbus->message.res.buff_index += 2;
        smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
    }
    else
    {
        smodbus_send_exception(smodbus, result);
    }
}
#endif
//##################################################################################################
//##################################################################################################
#if SMODBUS_FILE_COUNT
bool smodbus_add_file(smodbus_t *smodbus, smodbus_file_t *file)
{
    if (smodbus->files_count < SMODBUS_FILE_COUNT)
    {
        smodbus_slave_address_list_add(smodbus, file->slave_address);
        smodbus->files[smodbus->files_count] = *file;
        smodbus->files_count++;
        return true;
    }
    else
    {
        return false;
    }
}
//##################################################################################################
static void smodbus_write_file_record(smodbus_t *smodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    if (smodbus->files_count == 0)
    {
        result = MODBUS_RESULT_ILLEGAL_FUNCTION;
    }
    else
    {
        for (uint16_t k = 0; k < smodbus->files_count; k++)
        {
            if ((smodbus->message.req.slave_address == smodbus->files[k].slave_address) && (smodbus->message.req.file_number == smodbus->files[k].file_number))
            {
                uint16_t record_number = smodbus->message.req.record_number;
                uint8_t *data = smodbus->message.req.data;
                uint8_t size = smodbus->message.req.data_length - 7;

                if (smodbus->files[k].write_record_callback != NULL)
                {
                    smodbus->files[k].write_record_callback(&smodbus->files[k], record_number, data, size);
                }
                break;
            }
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        smodbus->message.res.buff_index = smodbus->message.req.buff_index;
        memcpy(smodbus->message.res.buff, smodbus->message.req.buff, smodbus->message.req.buff_index);
        smodbus_send_raw(smodbus, smodbus->message.res.buff, smodbus->message.res.buff_index);
    }
    else
    {
        smodbus_send_exception(smodbus, result);
    }
}
#endif
