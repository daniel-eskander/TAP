#include <cheri.hh>
#include<compartment.h>
#include <debug.hh>  // Required for debug_print()
#include <stdint.h>
#include <thread.h>
#include<platform-uart.hh>
#include<platform-pinmux.hh>
#include<platform/arty-a7/platform-entropy.hh>

using DEBUGMAIN = ConditionalDebug<true, "MAIN">;

typedef enum {
    IDLE,
    UART_PROCESSING,
    HEALTH_TEST_PROCESSING,
    DISPLAY_RESULTS
    }DataStatus_t;
DataStatus_t status = IDLE; 

typedef enum{
   PASS, 
   FAIL
}Health_test_status_t;
Health_test_status_t RTC_status = FAIL ;
Health_test_status_t APT_status = FAIL ;


#define DATA_SIZE 1000      
#define RX_BUFFER_SIZE DATA_SIZE 
#define UART_FRAME_SIZE RX_BUFFER_SIZE + 6 // 2 start bytes and 2 end bytes and 2 bytes for health tests reuslt 
#define ENTROPY_BUFFER_SIZE  DATA_SIZE/2
#define WINDOW_SIZE 20
#define CUTT_OFF_VALUE 6
#define TIMEOUT 5000

uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
uint16_t entropy_buffer[ENTROPY_BUFFER_SIZE]={0};
uint8_t uart_frame[UART_FRAME_SIZE] = {0};





Health_test_status_t health_test_RTC(uint8_t* data_buffer,size_t buffer_size,size_t Window_size, uint32_t cutoff){

    if (buffer_size == 0)
    {
        DEBUGMAIN::log("Buffer is empty, test Failed");
        return FAIL ;
    }

    uint8_t A = data_buffer[0]; // Initialize with the first noise value
    uint32_t B = 1;        // Count repetitions of the same value

    for (size_t i = 1; i < Window_size; i++)
    {
        uint8_t X = data_buffer[i];
        if (X == A)
        {
            B++;
            if (B >= cutoff)
            {
                // DEBUGMAIN::log("Test failed: Cutoff = {}, Repetition = {}", cutoff, B);
                return FAIL;
            }
        }
        else
        {
            A = X; // Update to the new value
            B = 1; // Reset repetition count
        }
    }

    return PASS ;  
}



Health_test_status_t health_test_APT(uint8_t* data_buffer,size_t buffer_size,size_t Window_size, uint32_t cutoff){



      if (buffer_size == 0)
    {
        DEBUGMAIN::log("Buffer is empty, test Failed");
        return FAIL ;
    }

    uint16_t A = data_buffer[0];
    uint16_t B =0;
    for (size_t i = 0; i < buffer_size/Window_size ; i++)
    {
        A = data_buffer[i * Window_size];
        for (size_t i = 1; i < Window_size; i++)
            {
                uint8_t X = data_buffer[i];
                if (X == A)
                {
                    B++;
                    if (B >= cutoff)
                    {
                        // DEBUGMAIN::log("Test failed: Cutoff = {}, Repetition = {}", cutoff, B);
                        return FAIL;
                    }
                }

            }
    

    }
    return PASS;
}




void extract_entropy_data(uint8_t* rx_buffer, uint16_t* entropy_buffer, size_t entropy_buffer_size) {
    for (size_t i = 0; i < entropy_buffer_size; i++) {
        entropy_buffer[i] = rx_buffer[(i * 2)] | (rx_buffer[(i * 2) + 1] << 8);
    }
}












