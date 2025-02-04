#include <compartment.h>
#include <cheri.hh>
#include <debug.hh>  
#include <stdint.h>
#include <thread.h>
#include<platform-spi.hh>
#include<platform-pinmux.hh>

#define CLEAR_BIT(REG, BIT) (REG = REG & (~(1U << (BIT))))
#define SET_BIT(REG, BIT)   (REG = REG | (1U << (BIT)))


using namespace SonataPinmux;

using DEBUGSPI = ConditionalDebug<true, "SPI_TEST_3">;


extern "C" [[noreturn]] void __cheri_compartment("SPI_TEST_3") spi_loopback_entry(){
    

    using DEBUGSPI = ConditionalDebug<true, "SPI_TEST_3">;

    uint8_t data[] ={0xAA,0xBB,0xAA,0xBB,0xAA,0xBB,0xAA 
    ,0xBB,0xAA,0xBB,0xAA ,0xBB,0xAA ,0xBB,0xAA ,0xBB,0xAA ,
    0xBB,0xAA , 0xBB,0xAA , 0xBB};

    auto pinSinks   = MMIO_CAPABILITY(PinSinks, pinmux_pins_sinks);
    auto blockSinks = MMIO_CAPABILITY(BlockSinks, pinmux_block_sinks);

    // SPI1 on PMOD0: Configuring Chip Select, CIPO, COPI, SCLK
    pinSinks->get(PinSink::pmod0_1).select(2);  // CS0
    blockSinks->get(BlockSink::spi_1_cipo).select(3); // CIPO
    pinSinks->get(PinSink::pmod0_2).select(2);  // COPI
    pinSinks->get(PinSink::pmod0_4).select(2);  // SCLK


    auto spi1 = MMIO_CAPABILITY(SonataSpi, spi1);

    while (true) {

        spi1->init(false, false, true, 0);
        DEBUGSPI::log("SPI intialized"); 
        CLEAR_BIT(spi1->cs, 0);
        DEBUGSPI::log("CS LOW");
        thread_millisecond_wait(100);
        DEBUGSPI::log("PERFORM TEST USING cheriotRTOS:platform_spi.hh driver")
        spi1->blocking_write(data,sizeof(data));
        DEBUGSPI::log("TEST_DONE") ;
        thread_millisecond_wait(3000);  // Wait e seconds and repeat test 
    }
}
