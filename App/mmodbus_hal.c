// 1402/12/28 => 18 March 2024 15:00 => Tiny Modifications
#include "main.h"
#include "mmodbus_hal.h"
mmodbus_t *mmodbus_list[10];
uint8_t    mmodbus_count = 0;

static inline void mmodbus_parse_byte(mmodbus_t *mmodbus, uint8_t b);

#if (MODBUS_WITH_RTOS == 0)
static modbus_result_t mmodbus_rx_handler(mmodbus_t *mmodbus);

void mmodbus_period_elapsed_callback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop(htim);
    for (uint8_t k = 0; k < mmodbus_count; k++)
    {
        if (mmodbus_list[k]->init.htim == htim)
        {
            mmodbus_list[k]->result_received = true;
            mmodbus_list[k]->result = MODBUS_RESULT_TIMEOUT;
            mmodbus_list[k]->cnt.rx_timeout++;
            if (mmodbus_list[k]->notify_callback != NULL)
            {
                mmodbus_list[k]->notify_callback(mmodbus_list[k]);
                HAL_UART_AbortReceive(mmodbus_list[k]->init.huart);
            }
        }
    }
}
#endif
//##################################################################################################
static inline mmodbus_t *mmodbus_find(UART_HandleTypeDef *huart)
{
    mmodbus_t *mmodbus = NULL;
    for (uint8_t k = 0; k < mmodbus_count; k++)
    {
        if (huart->Instance == mmodbus_list[k]->init.huart->Instance)
        {
            mmodbus = mmodbus_list[k];
            break;
        }
    }
    return mmodbus;
}
//##################################################################################################
void mmodbus_tx_cplt_callback(UART_HandleTypeDef *huart)
{
    mmodbus_t *mmodbus = mmodbus_find(huart);
    if (mmodbus != NULL)
    {
        #if MODBUS_WITH_RTOS
        osSemaphoreRelease(mmodbus->tx_semaphore_id);
        #endif
        if (mmodbus->init.rx_enable != NULL)
        {
            mmodbus->init.rx_enable();
        }
        if (mmodbus->init.rx_tx_type == MODBUS_DMA)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(mmodbus->init.huart, mmodbus->rx_buff, MMODBUS_RX_BUFF_SIZE);
        }
        else if (mmodbus->init.rx_tx_type == MODBUS_IT)
        {
            HAL_UARTEx_ReceiveToIdle_IT(mmodbus->init.huart, mmodbus->rx_buff, MMODBUS_RX_BUFF_SIZE);
        }
    }
    UNUSED(huart);
}
//##################################################################################################
static void mmodbus_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    mmodbus_t *mmodbus = mmodbus_find(huart);
    if (mmodbus != NULL)
    {
        if (huart->RxEventType != HAL_UART_RXEVENT_HT)
        {
            mmodbus->cnt.idle++;
            mmodbus->rx_head = Size;
            mmodbus->rx_tail = 0;
            #if MODBUS_WITH_RTOS
            osSemaphoreRelease(mmodbus->rx_semaphore_id);
            #else
            mmodbus_rx_handler(mmodbus);
            #endif
        }
    }
    UNUSED(huart);
    UNUSED(Size);
}
//##################################################################################################
void mmodbus_error_callback(UART_HandleTypeDef *huart)
{
    mmodbus_t *mmodbus = mmodbus_find(huart);
    if (mmodbus != NULL)
    {
        mmodbus->cnt.uart_frame_error++;
    }
    UNUSED(huart);
}
//##################################################################################################
void mmodbus_init(mmodbus_t *mmodbus)
{
    mmodbus_list[mmodbus_count++] = mmodbus;
    mmodbus->init.mx_uartx_init();
    if (mmodbus->init.rx_enable != NULL)
    {
        mmodbus->init.rx_enable();
    }
    mmodbus->init.huart->TxCpltCallback = mmodbus_tx_cplt_callback;
    mmodbus->init.huart->RxEventCallback = mmodbus_rx_event_callback;
    mmodbus->init.huart->ErrorCallback = mmodbus_error_callback;
#if MODBUS_WITH_RTOS
    mmodbus->tx_semaphore_attributes.name = "mmodbus_tx_semaphore";
    mmodbus->rx_semaphore_attributes.name = "mmodbus_rx_semaphore";
    mmodbus->tx_semaphore_id = osSemaphoreNew(1, 0, &mmodbus->tx_semaphore_attributes);
    mmodbus->rx_semaphore_id = osSemaphoreNew(1, 0, &mmodbus->rx_semaphore_attributes);
#else
    mmodbus->result_received = true;
    mmodbus->init.htim->PeriodElapsedCallback = mmodbus_period_elapsed_callback;
    __HAL_TIM_CLEAR_FLAG(mmodbus->init.htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(mmodbus->init.htim, TIM_IT_UPDATE);
#endif
}
//#####################################################################################################
static inline void mmodbus_parse_byte(mmodbus_t *mmodbus, uint8_t b)
{
    static uint16_t case_cnt = 0;
    if (mmodbus->message.res.buff_index < MODBUS_REQ_RES_SIZE)
    {
        mmodbus->message.res.buff[mmodbus->message.res.buff_index] = b;
        mmodbus->message.res.buff_index++;
    }
    switch (mmodbus->rx_state)
    {
        case WAIT_FOR_SLAVE_ADDRESS:
        {
            mmodbus->message.res.exception_code = 0;
            
            mmodbus->message.res.slave_address = b;
            mmodbus->rx_state = WAIT_FOR_FUNCTION_CODE;
            
        } break;
        case WAIT_FOR_FUNCTION_CODE:
        {
            mmodbus->message.res.function_code = b;
            switch (b)
            {
                case read_coils:
                case read_discrete_inputs:
                case read_holding_registers:
                case read_input_registers:
                {
                    mmodbus->rx_state = WAIT_FOR_BYTE_COUNT;
                } break;
                case write_single_holding_register:
                case write_single_coil:
                case write_multiple_coils:
                case write_multiple_registers:
                {
                    mmodbus->rx_state = WAIT_FOR_FIRST_REGISTER_ADDRESS;
                } break;
                case write_file_record:
                {
                    mmodbus->rx_state = WAIT_FOR_DATA_LENGTH;
                } break;
                
                case (0x80 | read_coils):
                case (0x80 | read_discrete_inputs):
                case (0x80 | read_holding_registers):
                case (0x80 | read_input_registers):
                case (0x80 | write_single_holding_register):
                case (0x80 | write_single_coil):
                case (0x80 | write_multiple_coils):
                case (0x80 | write_multiple_registers):
                case (0x80 | write_file_record):
                {
                    mmodbus->rx_state = WAIT_FOR_EXCEPTION_CODE;
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
            mmodbus->message.res.exception_code = b;
            mmodbus->rx_state = WAIT_FOR_CRC;
        } break;
        // write_file_record
        case WAIT_FOR_DATA_LENGTH:
        {
            mmodbus->message.res.data_length = b;
            mmodbus->rx_state = WAIT_FOR_REFERENCE_TYPE;
        } break;
        case WAIT_FOR_REFERENCE_TYPE:
        {
            mmodbus->message.res.reference_type = b;
            mmodbus->rx_state = WAIT_FOR_FILE_NUMBER;
        } break;
        case WAIT_FOR_FILE_NUMBER:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&mmodbus->message.res.file_number)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&mmodbus->message.res.file_number)[0] = b;
                mmodbus->rx_state = WAIT_FOR_RECORD_NUMBER;
                case_cnt = 0;
            }
        } break;
        case WAIT_FOR_RECORD_NUMBER:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&mmodbus->message.res.record_number)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&mmodbus->message.res.record_number)[0] = b;
                mmodbus->rx_state = WAIT_FOR_RECORD_LENGTH;
                case_cnt = 0;
            }
        } break;
        case WAIT_FOR_RECORD_LENGTH:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&mmodbus->message.res.record_length)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&mmodbus->message.res.record_length)[0] = b;
                mmodbus->rx_state = WAIT_FOR_RECORD_DATA;
                case_cnt = 0;
            }
        } break;
        case WAIT_FOR_RECORD_DATA:
        {
            mmodbus->message.req.data[case_cnt] = b;
            case_cnt++;
            if (case_cnt == (mmodbus->message.req.data_length - 7))
            {
                mmodbus->rx_state = WAIT_FOR_CRC;
                case_cnt = 0;
            }
        } break;
        // -----------------
        case WAIT_FOR_FIRST_REGISTER_ADDRESS:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&mmodbus->message.res.first_register_address)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&mmodbus->message.res.first_register_address)[0] = b;
                mmodbus->rx_state = WAIT_FOR_NUMBER_OF_REGISTERS;
                case_cnt = 0;
            }
        } break;
        case WAIT_FOR_NUMBER_OF_REGISTERS:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&mmodbus->message.res.number_of_registers)[1] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&mmodbus->message.res.number_of_registers)[0] = b;
                mmodbus->rx_state = WAIT_FOR_CRC;
                case_cnt = 0;
            }
        } break;
        // -----------------
        case WAIT_FOR_BYTE_COUNT:
        {
            mmodbus->message.res.byte_count = b;
            mmodbus->rx_state = WAIT_FOR_DATA;
        } break;
        case WAIT_FOR_DATA:
        {
            mmodbus->message.res.data[case_cnt] = b;
            case_cnt++;
            if (case_cnt == mmodbus->message.res.byte_count)
            {
                mmodbus->rx_state = WAIT_FOR_CRC;
                case_cnt = 0;
            }
        } break;
        // -----------------
        case WAIT_FOR_CRC:
        {
            if (case_cnt == 0)
            {
                ((uint8_t *)&mmodbus->message.res.crc_got)[0] = b;
                case_cnt++;
            }
            else
            {
                ((uint8_t *)&mmodbus->message.res.crc_got)[1] = b;
                mmodbus->message.res.crc_cal = modbus_crc16(mmodbus->message.res.buff, mmodbus->message.res.buff_index - 2);
                if (mmodbus->message.res.crc_cal == mmodbus->message.res.crc_got)
                {
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
    mmodbus->cnt.parse_byte_error++;
    _repeat:
    case_cnt = 0;
    mmodbus->rx_state = WAIT_FOR_SLAVE_ADDRESS;
    return;
}
//##################################################################################################
static void mmodbus_send_raw(mmodbus_t *mmodbus, uint8_t *data, uint16_t size)
{
    uint32_t startTime = HAL_GetTick();
    if (mmodbus->init.tx_enable != NULL)
    {
        mmodbus->init.tx_enable();
    }
    if (mmodbus->init.rx_tx_type == MODBUS_DMA)
    {
        while (__HAL_DMA_GET_COUNTER(mmodbus->init.huart->hdmatx))
        {
        }
        #if defined(STM32H7)
        SCB_CleanDCache_by_Addr((uint32_t *)data, size);
        #endif
        HAL_UART_Transmit_DMA(mmodbus->init.huart, data, size);
    }
    else if (mmodbus->init.rx_tx_type == MODBUS_IT)
    {
        HAL_UART_Transmit_IT(mmodbus->init.huart, data, size);
    }
    #if MODBUS_WITH_RTOS
    osSemaphoreAcquire(mmodbus->tx_semaphore_id, 100);
    #endif
}
//##################################################################################################
static modbus_result_t mmodbus_send_req(mmodbus_t *mmodbus)
{
    modbus_result_t result = MODBUS_RESULT_OK;
    
    memset(mmodbus->message.req.buff, 0, MODBUS_REQ_RES_SIZE);
    mmodbus->message.req.buff_index = 0;
    memset(mmodbus->message.res.buff, 0, MODBUS_REQ_RES_SIZE);
    mmodbus->message.res.buff_index = 0;
    
    if (mmodbus->message.req.function_code != write_file_record)
    {
        if (mmodbus->message.req.first_register_address >= 9999)
        {
            result = MODBUS_RESULT_INVALID_REQ;
        }
    }
    if (result == MODBUS_RESULT_OK)
    {
        mmodbus->message.req.buff[0] = mmodbus->message.req.slave_address;
        uint8_t crc_index = 0;
        switch (mmodbus->message.req.function_code)
        {
            case read_coils:
            case read_discrete_inputs:
            case read_holding_registers:
            case read_input_registers:
            case write_single_coil:
            case write_single_holding_register:
            {
                mmodbus->message.req.buff[1] = mmodbus->message.req.function_code;
                *(uint16_t *)&mmodbus->message.req.buff[2] = SWAP16(mmodbus->message.req.first_register_address);
                *(uint16_t *)&mmodbus->message.req.buff[4] = SWAP16(mmodbus->message.req.number_of_registers);
                crc_index = 6;
            } break;
            case write_multiple_coils:
            case write_multiple_registers:
            {
                mmodbus->message.req.buff[1] = mmodbus->message.req.function_code;
                *(uint16_t *)&mmodbus->message.req.buff[2] = SWAP16(mmodbus->message.req.first_register_address);
                *(uint16_t *)&mmodbus->message.req.buff[4] = SWAP16(mmodbus->message.req.number_of_registers);
                mmodbus->message.req.buff[6] = mmodbus->message.req.byte_count;
                memcpy(&mmodbus->message.req.buff[7], mmodbus->message.req.data, mmodbus->message.req.byte_count);
                crc_index = 7 + mmodbus->message.req.byte_count;
            } break;
            case write_file_record:
            {
                mmodbus->message.req.buff[1] = mmodbus->message.req.function_code;
                mmodbus->message.req.buff[2] = mmodbus->message.req.data_length;
                mmodbus->message.req.buff[3] = mmodbus->message.req.reference_type;
                *(uint16_t *)&mmodbus->message.req.buff[4] = SWAP16(mmodbus->message.req.file_number);
                *(uint16_t *)&mmodbus->message.req.buff[6] = SWAP16(mmodbus->message.req.record_number);
                *(uint16_t *)&mmodbus->message.req.buff[8] = SWAP16(mmodbus->message.req.record_length);
                memcpy(&mmodbus->message.req.buff[10], mmodbus->message.req.data, mmodbus->message.req.data_length - 7);
                crc_index = 3 + mmodbus->message.req.data_length;
            } break;
            default:
            {
                result = MODBUS_RESULT_ILLEGAL_FUNCTION;
            } break;
        }
        mmodbus->message.req.crc_cal = modbus_crc16(mmodbus->message.req.buff, crc_index);
        memcpy(&mmodbus->message.req.buff[crc_index], (uint8_t *)&mmodbus->message.req.crc_cal, 2);
        mmodbus->message.req.buff_index = crc_index + 2;
        if (result == MODBUS_RESULT_OK)
        {
            mmodbus->rx_state = WAIT_FOR_SLAVE_ADDRESS;
            mmodbus->message.res.buff_index = 0;
            mmodbus->cnt.tx_ok_req++;
            mmodbus_send_raw(mmodbus, mmodbus->message.req.buff, mmodbus->message.req.buff_index);
        }
    }
    return result;
}
//##################################################################################################
#if MODBUS_WITH_RTOS
modbus_result_t mmodbus_rx_handler(mmodbus_t *mmodbus, uint32_t rx_timeout)
{
    osStatus_t osStatus = osSemaphoreAcquire(mmodbus->rx_semaphore_id, rx_timeout);
    if (osStatus != osOK)
    {
        HAL_UART_AbortReceive(mmodbus->init.huart);
        mmodbus->cnt.rx_timeout++;
        return MODBUS_RESULT_TIMEOUT;
    }
#else
static modbus_result_t mmodbus_rx_handler(mmodbus_t *mmodbus)
{
#endif
    modbus_result_t result = MODBUS_RESULT_OK;
    if (mmodbus->init.rx_tx_type == MODBUS_DMA)
    {
        #if defined(STM32H7)
        SCB_InvalidateDCache_by_Addr ((uint32_t *)&mmodbus->rx_buff[mmodbus->rx_tail], mmodbus->rx_head - mmodbus->rx_tail);
        #endif
    }
    while (mmodbus->rx_tail != mmodbus->rx_head)
    {
        mmodbus_parse_byte(mmodbus, mmodbus->rx_buff[mmodbus->rx_tail]);
        mmodbus->rx_tail++;
        mmodbus->rx_tail %= MMODBUS_RX_BUFF_SIZE;
    }
    if (mmodbus->rx_state != WAIT_FOR_SLAVE_ADDRESS)
    {
        #if MODBUS_WITH_RTOS
        return MODBUS_RESULT_INCOMPLETE_RES;
        #else
        goto __end;
        #endif
    }
    switch (mmodbus->message.res.function_code)
    {
        case read_coils:
        case read_discrete_inputs:
        case read_holding_registers:
        case read_input_registers:
        {
            if ((mmodbus->message.res.slave_address != mmodbus->message.req.slave_address) ||
                (mmodbus->message.res.function_code != mmodbus->message.req.function_code) ||
                (mmodbus->message.res.crc_cal       != mmodbus->message.res.crc_got))
            {
                result = MODBUS_RESULT_INVALID_RES;
            }
        } break;
        case write_single_coil:
        case write_single_holding_register:
        case write_file_record:
        {
            if (memcmp(mmodbus->message.req.buff, mmodbus->message.res.buff, 8) != 0)
            {
                result = MODBUS_RESULT_INVALID_RES;
            }
        } break;
        case write_multiple_coils:
        case write_multiple_registers:
        {
            if ((mmodbus->message.res.slave_address          != mmodbus->message.req.slave_address)          ||
                (mmodbus->message.res.function_code          != mmodbus->message.req.function_code)          ||
                (mmodbus->message.res.first_register_address != mmodbus->message.req.first_register_address) ||
                (mmodbus->message.res.number_of_registers    != mmodbus->message.req.number_of_registers)    ||
                (mmodbus->message.res.crc_cal                != mmodbus->message.res.crc_got))
            {
                result = MODBUS_RESULT_INVALID_RES;
            }
        } break;
        
        case (0x80 | read_coils):
        case (0x80 | read_discrete_inputs):
        case (0x80 | read_holding_registers):
        case (0x80 | read_input_registers):
        case (0x80 | write_single_coil):
        case (0x80 | write_single_holding_register):
        case (0x80 | write_file_record):
        case (0x80 | write_multiple_coils):
        case (0x80 | write_multiple_registers):
        {
            result = (modbus_result_t)mmodbus->message.res.exception_code;
            switch (result)
            {
                case MODBUS_RESULT_ILLEGAL_FUNCTION:
                {
                    mmodbus->cnt.rx_exception_illegal_function++;
                } break;
                case MODBUS_RESULT_ILLEGAL_DATA_ADDRESS:
                {
                    mmodbus->cnt.rx_exception_illegal_data_address++;
                } break;
                case MODBUS_RESULT_ILLEGAL_DATA_VALUE:
                {
                    mmodbus->cnt.rx_exception_illegal_data_value++;
                } break;
                default:
                {
                    __NOP();
                } break;
            }
        } break;
        
        default:
        {
            result = MODBUS_RESULT_ILLEGAL_FUNCTION;
        } break;
    }
    if (result == MODBUS_RESULT_OK)
    {
        mmodbus->cnt.rx_ok_res++;
    }
#if MODBUS_WITH_RTOS
    if ((result == MODBUS_RESULT_OK) && (mmodbus->message.req.function_code == write_file_record))
    {
        mmodbus->write_file_record_callback(mmodbus, mmodbus->message.req.data, 2 * mmodbus->message.req.record_length);
    }
#else
    if (result == MODBUS_RESULT_OK)
    {
        switch (mmodbus->message.req.function_code)
        {
            case read_coils:
            {
                modbus_byte_array_to_bool_array(mmodbus->message.res.data, mmodbus->message.res.byte_count, mmodbus->p_coils, mmodbus->message.req.number_of_registers);
            } break;
            case read_discrete_inputs:
            {
                modbus_byte_array_to_bool_array(mmodbus->message.res.data, mmodbus->message.res.byte_count, mmodbus->p_discrete_inputs, mmodbus->message.req.number_of_registers);
            } break;
            case read_holding_registers:
            {
                modbus_byte_array_to_u16_array(mmodbus->message.res.data, mmodbus->message.res.byte_count, mmodbus->p_holding_registers, modmus_16bit_order_BA);
            } break;
            case read_input_registers:
            {
                modbus_byte_array_to_u16_array(mmodbus->message.res.data, mmodbus->message.res.byte_count, mmodbus->p_input_registers, modmus_16bit_order_BA);
            } break;
            case write_single_coil:
            {
                //coil = (mmodbus->message.res.number_of_registers != 0);
            } break;
            case write_single_holding_register:
            {
                // holding_register = mmodbus->message.res.number_of_registers;
            } break;
            case write_file_record:
            {
                mmodbus->write_file_record_callback(mmodbus, mmodbus->message.req.data, 2 * mmodbus->message.req.record_length);
            } break;
            case write_multiple_coils:
            {
            } break;
            case write_multiple_registers:
            {
            } break;
        }
    }
    __end:
    HAL_TIM_Base_Stop(mmodbus->init.htim);
    mmodbus->result_received = true;
    mmodbus->result = result;
    if (mmodbus->notify_callback != NULL)
    {
        mmodbus->notify_callback(mmodbus);
    }
#endif
    return result;
}
//##################################################################################################
//##################################################################################################
#if (MODBUS_WITH_RTOS == 0)
static void mmodbus_initiate_rx_tiomeout(mmodbus_t *mmodbus, uint32_t rx_timeout)
{
    mmodbus->result_received = false;
    mmodbus->init.htim->Instance->CNT = 0;
    mmodbus->init.htim->Instance->ARR = mmodbus->init.htim_1ms_cnt * rx_timeout;
    __HAL_TIM_CLEAR_FLAG(mmodbus->init.htim, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start(mmodbus->init.htim);
}
#endif
//##################################################################################################
//##################################################################################################
modbus_result_t mmodbus_read_coils(mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, bool *coils, uint32_t rx_timeout)
{
    #if (MODBUS_WITH_RTOS == 0)
    while (mmodbus->result_received == false){}
    #endif
    modbus_result_t result = MODBUS_RESULT_OK;
    
    mmodbus->message.req.slave_address = slave_address;
    mmodbus->message.req.function_code = read_coils;
    mmodbus->message.req.first_register_address = first_register_address;
    mmodbus->message.req.number_of_registers = number_of_registers;
    result = mmodbus_send_req(mmodbus);
    if (result == MODBUS_RESULT_OK)
    {
        #if MODBUS_WITH_RTOS
        result = mmodbus_rx_handler(mmodbus, rx_timeout);
        if (result == MODBUS_RESULT_OK)
        {
            modbus_byte_array_to_bool_array(mmodbus->message.res.data, mmodbus->message.res.byte_count, coils, mmodbus->message.req.number_of_registers);
        }
        #else
        mmodbus->p_coils = coils;
        mmodbus_initiate_rx_tiomeout(mmodbus, rx_timeout);
        #endif
    }
    return result;
}
//##################################################################################################
modbus_result_t mmodbus_write_single_coil(mmodbus_t *mmodbus, uint8_t slave_address, uint16_t register_address, bool coil, uint32_t rx_timeout)
{
    #if (MODBUS_WITH_RTOS == 0)
    while (mmodbus->result_received == false){}
    #endif
    modbus_result_t result = MODBUS_RESULT_OK;
    
    mmodbus->message.req.slave_address = slave_address;
    mmodbus->message.req.first_register_address = register_address;
    mmodbus->message.req.function_code = write_single_coil;
    if (!coil)
    {
        mmodbus->message.req.number_of_registers = 0x0000;
    }
    else
    {
        mmodbus->message.req.number_of_registers = 0xFF00;
    }
    result = mmodbus_send_req(mmodbus);
    if (result == MODBUS_RESULT_OK)
    {
        #if MODBUS_WITH_RTOS
        result = mmodbus_rx_handler(mmodbus, rx_timeout);
        #else
        //mmodbus->p_coils = coils;
        mmodbus_initiate_rx_tiomeout(mmodbus, rx_timeout);
        #endif
    }
    return result;
}
//##################################################################################################
modbus_result_t mmodbus_write_multiple_coils(mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, bool *coils, uint32_t rx_timeout)
{
    #if (MODBUS_WITH_RTOS == 0)
    while (mmodbus->result_received == false){}
    #endif
    modbus_result_t result = MODBUS_RESULT_OK;
    
    mmodbus->message.req.slave_address = slave_address;
    mmodbus->message.req.first_register_address = first_register_address;
    mmodbus->message.req.function_code = write_multiple_coils;
    mmodbus->message.req.number_of_registers = number_of_registers;
    mmodbus->message.req.byte_count = modbus_bool_array_to_byte_array(coils, number_of_registers, mmodbus->message.req.data);
    result = mmodbus_send_req(mmodbus);
    if (result == MODBUS_RESULT_OK)
    {
        #if MODBUS_WITH_RTOS
        result = mmodbus_rx_handler(mmodbus, rx_timeout);
        #else
        mmodbus->p_coils = coils;
        mmodbus_initiate_rx_tiomeout(mmodbus, rx_timeout);
        #endif
    }
    return result;
}
//##################################################################################################
//##################################################################################################
modbus_result_t mmodbus_read_discrete_inputs(mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, bool *discrete_inputs, uint32_t rx_timeout)
{
    #if (MODBUS_WITH_RTOS == 0)
    while (mmodbus->result_received == false){}
    #endif
    modbus_result_t result = MODBUS_RESULT_OK;
    
    mmodbus->message.req.slave_address = slave_address;
    mmodbus->message.req.function_code = read_discrete_inputs;
    mmodbus->message.req.first_register_address = first_register_address;
    mmodbus->message.req.number_of_registers = number_of_registers;
    result = mmodbus_send_req(mmodbus);
    if (result == MODBUS_RESULT_OK)
    {
        #if MODBUS_WITH_RTOS
        result = mmodbus_rx_handler(mmodbus, rx_timeout);
        if (result == MODBUS_RESULT_OK)
        {
            modbus_byte_array_to_bool_array(mmodbus->message.res.data, mmodbus->message.res.byte_count, discrete_inputs, mmodbus->message.req.number_of_registers);
        }
        #else
        mmodbus->p_discrete_inputs = discrete_inputs;
        mmodbus_initiate_rx_tiomeout(mmodbus, rx_timeout);
        #endif
    }
    return result;
}
//##################################################################################################
//##################################################################################################
modbus_result_t mmodbus_read_holding_registers(mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, uint16_t *holding_registers, uint32_t rx_timeout)
{
    #if (MODBUS_WITH_RTOS == 0)
    while (mmodbus->result_received == false){}
    #endif
    modbus_result_t result = MODBUS_RESULT_OK;
    
    mmodbus->message.req.slave_address = slave_address;
    mmodbus->message.req.function_code = read_holding_registers;
    mmodbus->message.req.first_register_address = first_register_address;
    mmodbus->message.req.number_of_registers = number_of_registers;
    result = mmodbus_send_req(mmodbus);
    if (result == MODBUS_RESULT_OK)
    {
        #if MODBUS_WITH_RTOS
        result = mmodbus_rx_handler(mmodbus, rx_timeout);
        if (result == MODBUS_RESULT_OK)
        {
            modbus_byte_array_to_u16_array(mmodbus->message.res.data, mmodbus->message.res.byte_count, holding_registers, modmus_16bit_order_BA);
        }
        #else
        mmodbus->p_holding_registers = holding_registers;
        mmodbus_initiate_rx_tiomeout(mmodbus, rx_timeout);
        #endif
    }
    return result;
}
//##################################################################################################
modbus_result_t mmodbus_write_single_holding_register(mmodbus_t *mmodbus, uint8_t slave_address, uint16_t register_address, uint16_t holding_register, uint32_t rx_timeout)
{
    #if (MODBUS_WITH_RTOS == 0)
    while (mmodbus->result_received == false){}
    #endif
    modbus_result_t result = MODBUS_RESULT_OK;
    
    mmodbus->message.req.slave_address = slave_address;
    mmodbus->message.req.function_code = write_single_holding_register;
    mmodbus->message.req.first_register_address = register_address;
    mmodbus->message.req.number_of_registers = holding_register;
    result = mmodbus_send_req(mmodbus);
    if (result == MODBUS_RESULT_OK)
    {
        #if MODBUS_WITH_RTOS
        result = mmodbus_rx_handler(mmodbus, rx_timeout);
        #else
        //mmodbus->p_holding_registers = holding_registers;
        mmodbus_initiate_rx_tiomeout(mmodbus, rx_timeout);
        #endif
    }
    return result;
}
//##################################################################################################
modbus_result_t mmodbus_write_multiple_registers(mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, uint16_t *holding_registers, uint32_t rx_timeout)
{
    #if (MODBUS_WITH_RTOS == 0)
    while (mmodbus->result_received == false){}
    #endif
    modbus_result_t result = MODBUS_RESULT_OK;
    
    mmodbus->message.req.slave_address = slave_address;
    mmodbus->message.req.first_register_address = first_register_address;
    mmodbus->message.req.function_code = write_multiple_registers;
    mmodbus->message.req.number_of_registers = number_of_registers;
    mmodbus->message.req.byte_count = 2 * number_of_registers;
    modbus_byte_array_to_u16_array((uint8_t *)holding_registers, mmodbus->message.req.byte_count, (uint16_t *)mmodbus->message.req.data, modmus_16bit_order_BA);
    result = mmodbus_send_req(mmodbus);
    if (result == MODBUS_RESULT_OK)
    {
        #if MODBUS_WITH_RTOS
        result = mmodbus_rx_handler(mmodbus, rx_timeout);
        #else
        mmodbus->p_holding_registers = holding_registers;
        mmodbus_initiate_rx_tiomeout(mmodbus, rx_timeout);
        #endif
    }
    return result;
}
//##################################################################################################
//##################################################################################################
modbus_result_t mmodbus_read_input_registers(mmodbus_t *mmodbus, uint8_t slave_address, uint16_t first_register_address, uint16_t number_of_registers, uint16_t *input_registers, uint32_t rx_timeout)
{
    #if (MODBUS_WITH_RTOS == 0)
    while (mmodbus->result_received == false){}
    #endif
    modbus_result_t result = MODBUS_RESULT_OK;
    
    mmodbus->message.req.slave_address = slave_address;
    mmodbus->message.req.function_code = read_input_registers;
    mmodbus->message.req.first_register_address = first_register_address;
    mmodbus->message.req.number_of_registers = number_of_registers;
    result = mmodbus_send_req(mmodbus);
    if (result == MODBUS_RESULT_OK)
    {
        #if MODBUS_WITH_RTOS
        result = mmodbus_rx_handler(mmodbus, rx_timeout);
        if (result == MODBUS_RESULT_OK)
        {
            modbus_byte_array_to_u16_array(mmodbus->message.res.data, mmodbus->message.res.byte_count, input_registers, modmus_16bit_order_BA);
        }
        #else
        mmodbus->p_input_registers = input_registers;
        mmodbus_initiate_rx_tiomeout(mmodbus, rx_timeout);
        #endif
    }
    return result;
}
//##################################################################################################
//##################################################################################################
modbus_result_t mmodbus_write_file_record(mmodbus_t *mmodbus, uint8_t slave_address, uint8_t *bytes, uint8_t bytes_length, uint16_t file_number, uint16_t record_number, uint32_t rx_timeout)
{
    #if (MODBUS_WITH_RTOS == 0)
    while (mmodbus->result_received == false){}
    #endif
    
    modbus_result_t result = ((bytes_length % 2) == 0) ? MODBUS_RESULT_OK : MODBUS_RESULT_INVALID_REQ;
    if (result == MODBUS_RESULT_OK)
    {
        mmodbus->message.req.slave_address = slave_address;
        mmodbus->message.req.function_code = write_file_record;
        mmodbus->message.req.data_length = 7 + bytes_length;
        mmodbus->message.req.reference_type = 6;
        mmodbus->message.req.file_number = file_number;
        mmodbus->message.req.record_number = record_number;
        mmodbus->message.req.record_length = bytes_length / 2;
        memcpy(mmodbus->message.req.data, bytes, bytes_length);
        result = mmodbus_send_req(mmodbus);
        if (result == MODBUS_RESULT_OK)
        {
            #if MODBUS_WITH_RTOS
            result = mmodbus_rx_handler(mmodbus, rx_timeout);
            #else
            //mmodbus->p_file_record = file_record;
            mmodbus_initiate_rx_tiomeout(mmodbus, rx_timeout);
            #endif
        }
    }
    return result;
}
