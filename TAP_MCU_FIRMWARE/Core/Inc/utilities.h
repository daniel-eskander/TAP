/*
 * utilities.h
 *
 *  Created on: Jan 9, 2024
 *      Author: danie
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_




#endif /* INC_UTILITIES_H_ */

#include "common.h"



#define ADC_buffer_size 10
#define initialized 10
#define not_initialized 0


void VCP_Transmit_32(uint32_t data);
void VCP_Transmit_64(uint64_t data);
void VCP_Transmit(uint8_t* data,uint8_t size);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
uint16_t Create_Input_Vector(uint16_t* matrix);
void USART_SEND(uint8_t uart_byte);
void buffer_copy(uint8_t* destination,uint8_t* source, uint16_t size );

