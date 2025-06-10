
#include"utilities.h"
#include "Look_Up_Table.h"






uint16_t __attribute__((optimize("-Ofast"))) Create_Input_Vector(uint16_t* input_vector){



static uint16_t ADC_buffer_counter =0;
uint16_t buffer_counter=0;
uint16_t entropy_signal= 0;
float  entropy_sum=0 ;
float entropy_per_sample =0;



  while(entropy_sum<256){



			if(!(ADC_buffer_counter<ADC_BUFFER_SIZE)){

				ADC_buffer_counter = 0;
				ADC_buffer_status = ADC_BUFFER_HASHED;
			}

	    	entropy_per_sample= Look_Up_Table[certification_signal_extract(ADC_buffer_counter)];
			ADC_buffer_counter++;

				if(entropy_per_sample!=0){

					input_vector[buffer_counter]=entropy_signal_extract(ADC_buffer_counter);
					entropy_sum+=entropy_per_sample;
					buffer_counter++;
									}




 }

   return buffer_counter*2;

}



void VCP_Transmit(uint8_t* data_buffer, uint8_t size)
  {

UART_DATA_FLAG_typedef  UART_DATA_FLAG = NOT_SENT;

	    //Set the TE bit in USART_CR1 to send an idle frame as first transmission
	    USART1->CR1 |= USART_CR1_TE_Msk;


	   for(int i =0; i<size ;i++){

uint8_t  LSB_R1  = (data_buffer[i]&0xFF);
uint8_t	 MSB_R1 = ((data_buffer[i]>>8)&0xFF);
uint8_t	 LSB_R2 = ((data_buffer[i]>>16)&0xFF);
uint8_t	 MSB_R2 = ((data_buffer[i]>>24)&0xFF);

		USART_SEND( START_BYTE);
	    USART_SEND( MSB_R1);
		USART_SEND( LSB_R1 );
		USART_SEND( MSB_R2 );
		USART_SEND( LSB_R2 );
		USART_SEND( END_BYTE);

	   }
	   UART_DATA_FLAG = SENT;



	  }


void USART_SEND(uint8_t uart_byte){

	 while(!(USART1->ISR & USART_ISR_TC_Msk)){


	 }

		  USART1 -> TDR = uart_byte;

}


void buffer_copy(uint8_t* destination,uint8_t* source, uint16_t size ){


	for (int i = 0; i < size; i++) {

		destination[i]= source[i];

	}
}

