/*
 * global variables.c
 *
 *  Created on: Jan 9, 2024
 *      Author: danie
 */
#include "common.h"

uint32_t ADC_buffer_entropy[ADC_BUFFER_SIZE]={0};
//__attribute__((section("RAMD3"), aligned(32)))
 uint32_t ADC_buffer_certification[ADC_BUFFER_SIZE]={0};

volatile uint32_t* current_ADC_buffer_ptr =ADC_buffer_entropy+(sizeof(ADC_buffer_entropy)/2);
uint8_t post_processed_data_buffer[post_processed_data_buffer_size] ={0};
uint8_t* current_post_processed_data_ptr = post_processed_data_buffer;

uint32_t start_time_1 =0;
uint32_t end_time_1 =0;
uint32_t elapsed_time_1 =0;

uint32_t start_time_2 =0;
uint32_t end_time_2 =0;
uint32_t elapsed_time_2=0;


volatile HASH_buffer_status_typedef     HASH_buffer_status = HASH_buffer_ready;
volatile HASH_processor_status_typedef  HASH_processor_status = HASH_Digest_Done;
volatile ADC_buffer_status_typedef      ADC_buffer_status = ADC_BUFFER_EMPTY;
volatile uint8_t uart_frame[UART_FRAME_SIZE]={0};







