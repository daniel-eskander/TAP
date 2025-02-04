#include <cheri.hh>
#include<compartment.h>
#include <debug.hh>  // Required for debug_print()
#include <stdint.h>
#include <thread.h>
#include<platform-uart.hh>
#include<platform-pinmux.hh>
#include"common.hh"



using namespace SonataPinmux;



extern "C" [[noreturn]] void __cheri_compartment("MAIN") UART2_entry(){


    DEBUGMAIN::log("uart2 init ") ;


    auto pinSinks   = MMIO_CAPABILITY(PinSinks, pinmux_pins_sinks);
    auto blockSinks = MMIO_CAPABILITY(BlockSinks, pinmux_block_sinks);

// UART_1 on ser1_tx (DONE)
    //  pinSinks->get(PinSink::ser1_tx).select(1);  // UART1_TX 
    //  blockSinks->get(BlockSink::uart_1_rx).select(1); // ser1 tx 

// // UART_1 on  arduino shield (DONE)
    // pinSinks->get(PinSink::ah_tmpio1).select(1);  // UART1_TX 
    // blockSinks->get(BlockSink::uart_1_rx).select(3); // ahtmpio0


// UART 1 on PMOD connector
    pinSinks->get(PinSink::pmod0_2).select(4);  // UART1_TX 
    blockSinks->get(BlockSink::uart_1_rx).select(5); // Pmod0_3


// UART 2 on ser_1

    pinSinks->get(PinSink::ser1_tx).select(2);  // UART1_TX 
    blockSinks->get(BlockSink::uart_2_rx).select(1); // ser1 tx 

    auto uart1 = MMIO_CAPABILITY(OpenTitanUart, uart1);
    auto uart2 = MMIO_CAPABILITY(OpenTitanUart, uart2);



    uart_frame[UART_FRAME_SIZE-1]=0XCC; // last 2 bytes after the data are end bytes
    uart_frame[UART_FRAME_SIZE-2]=0XCC;
    DEBUGMAIN::log("ready ") ;


    while (true)
    {
        /* code */


            // UART_1 on  PMOD  (DONE)

        uart1-> init();
        
        if (status == IDLE)
        {
            status = UART_PROCESSING;
            // DEBUGMAIN::log("status:{}",status);

            if (status == UART_PROCESSING)
            {
                thread_millisecond_wait(100);
                if(uart1->blocking_read_buffer(rx_buffer, DATA_SIZE ,TIMEOUT)==true){
                    DEBUGMAIN::log("UART_DATA_VALID");
                }
                else{  
                    //try again 
                        if(uart1->blocking_read_buffer(rx_buffer, DATA_SIZE ,TIMEOUT)==true){
                            DEBUGMAIN::log("UART_DATA_VALID");
                            }
                        else{
                                DEBUGMAIN::log("INVALID_DATA");
                            }


                }

                thread_millisecond_wait(100);
                extract_entropy_data(rx_buffer, entropy_buffer, ENTROPY_BUFFER_SIZE);



                // DEBUGMAIN::log("status:{}",status);
            } 
            status=HEALTH_TEST_PROCESSING;
            thread_millisecond_wait(10);
            DEBUGMAIN::log("status:{}",status);
            
        }
        
        if (status == HEALTH_TEST_PROCESSING)
        {    

            RTC_status = health_test_RTC(rx_buffer,RX_BUFFER_SIZE,WINDOW_SIZE,CUTT_OFF_VALUE);
            APT_status = health_test_APT(rx_buffer,RX_BUFFER_SIZE,WINDOW_SIZE,CUTT_OFF_VALUE);
            status = DISPLAY_RESULTS;
    
        }

        if (status == DISPLAY_RESULTS)
        {
             thread_millisecond_wait(10);
             DEBUGMAIN::log("status:{}",status);
            thread_millisecond_wait(100);
            DEBUGMAIN::log("initialize uart2");
            uart2-> init();
            DEBUGMAIN::log("copy memory buffer");
            memcpy(uart_frame+2, rx_buffer, DATA_SIZE);

            uart_frame[0]=0XAA; // last 2 bytes after the data are end bytes
            uart_frame[1]=0XAA;
            uart_frame[UART_FRAME_SIZE-3]=RTC_status;
            uart_frame[UART_FRAME_SIZE-4]=APT_status;
            uart2->blocking_write_buffer(uart_frame,UART_FRAME_SIZE);
            DEBUGMAIN::log("DONE");
            thread_millisecond_wait(1000);
            status = IDLE;
            DEBUGMAIN::log("status:{}",status);
        }
    
    

}
}


