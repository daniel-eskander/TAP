/*
 * common.h
 *
 *  Created on: Jan 15, 2024
 *      Author: danie
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_
//----------Includes-----------------//

#include "stm32h7xx_hal.h"
#include "stdint.h"
#include "stdio.h"



//------------Defines----------------//
#define initialized 10
#define not_initialized 0
#define next_line 5
#define OUTPUT_PACKET_SIZE 32

#define ADC_BUFFER_SIZE 32*100
#define HALF_ADC_BUFFER_SIZE (ADC_BUFFER_SIZE/2)
#define post_processed_data_buffer_size  65535
#define not_sent 1
#define sent 0
#define START_BYTE 0xAA
#define END_BYTE   0xCC
#define ADC_TRANSFER_SIZE 1000
#define UART_FRAME_SIZE ADC_TRANSFER_SIZE + 4





//------------------------Type_defines--------------//

typedef enum{
	HASH_buffer_Input_complete,
	HASH_buffer_busy,
	HASH_buffer_ready,

}HASH_buffer_status_typedef;

typedef enum{
HASH_Digest_Done,
HASH_Digest_calculating
}HASH_processor_status_typedef;


typedef enum{
	NOT_SENT,
    SENT
}UART_DATA_FLAG_typedef;

typedef  enum{
ADC_BUFFER_EMPTY,
ADC_BUFFER_READY,
ADC_BUFFER_HALF_COMPLETE,
ADC_BUFFER_COMPLETE,
ADC_BUFFER_HASHED
}ADC_buffer_status_typedef;



//-------------------Global variables-------------//
extern uint32_t ADC_buffer_entropy[ADC_BUFFER_SIZE];

//__attribute__((section("RAMD3"), aligned(32)))
extern  uint32_t ADC_buffer_certification[ADC_BUFFER_SIZE];


extern volatile uint32_t* current_ADC_buffer_ptr;
extern uint8_t  post_processed_data_buffer[post_processed_data_buffer_size];
extern uint8_t* current_post_processed_data_ptr;
extern uint8_t  done_flag ;
extern uint8_t  DMA_buffer_counter_flag;
extern uint32_t start_time_1;
extern uint32_t end_time_1;
extern uint32_t elapsed_time_1;
extern uint32_t start_time_2;
extern uint32_t end_time_2;
extern uint32_t elapsed_time_2;
extern volatile HASH_buffer_status_typedef     HASH_buffer_status;
extern volatile HASH_processor_status_typedef  HASH_processor_status;
extern volatile ADC_buffer_status_typedef      ADC_buffer_status;
extern volatile uint8_t uart_frame[UART_FRAME_SIZE];



//--------------fucntions----------------------//


void arm_and_u32(
  const uint32_t * pSrcA,
  const uint32_t * pSrcB,
        uint32_t * pDst,
        uint32_t blockSize);


void arm_xor_u32(
  const uint32_t * pSrcA,
  const uint32_t * pSrcB,
        uint32_t * pDst,
        uint32_t blockSize);



#endif /* INC_COMMON_H_ */
