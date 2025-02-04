#include"common.hh"


extern "C" [[noreturn]] void __cheri_compartment("HEALTH_TESTS") health_test_entry(){


        using DEBUGHEALTH = ConditionalDebug<true, "HEALTH_TESTS">;
while (true)
{   
    // DEBUGHEALTH::log("status:{}",status);
    //     if (status == HEALTH_TEST_PROCESSING)
    // {
    //      DEBUGHEALTH::log("status:{}",status);
    //     for (size_t i = 0; i < sizeof(rx_buffer); i++)
    //     {
    //     rx_buffer[i]= 0xFF ;      /* code */
    //     }
    //     status = DATA_PRINT;
    //      DEBUGHEALTH::log("status:{}",status);
    // }
    thread_millisecond_wait(1000);
    /* code */
}   

}


