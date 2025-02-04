#pragma once

#include <compartment.h>
#include <cheri.hh>
#include <debug.hh>  // Required for debug_print()
#include <stdint.h>
#include <thread.h>
#include<platform-spi.hh>
#include<platform-pinmux.hh>

using namespace SonataPinmux;
using DebugSPI = ConditionalDebug<true, "SPI_TEST">;


static void configure_spi1_pinmux()
{
    auto pinSinks   = MMIO_CAPABILITY(PinSinks, pinmux_pins_sinks);
    auto blockSinks = MMIO_CAPABILITY(BlockSinks, pinmux_block_sinks);

    // SPI1 on PMOD0: Configuring Chip Select, CIPO, COPI, SCLK
    pinSinks->get(PinSink::pmod0_1).select(2);  // CS0
    blockSinks->get(BlockSink::spi_1_cipo).select(3); // CIPO
    pinSinks->get(PinSink::pmod0_2).select(2);  // COPI
    pinSinks->get(PinSink::pmod0_4).select(2);  // SCLK
}

/**
 * Simple SPI Loopback Test
 */
static void spi_loopback_test(volatile SonataSpi* spi)
{

    constexpr size_t len = 16;  // Test data length
    uint8_t tx_buffer[len] = {0xA5, 0x5A, 0x3C, 0xC3, 0xF0, 0x0F, 0xAA, 0x55,
                               0x11, 0x22, 0x33, 0x44, 0x66, 0x77, 0x88, 0x99};
    uint8_t rx_buffer[len] = {0};



    spi->wait_idle();
    spi->init(false, false, true, 0); // CPOL=0, CPHA=0, MSB first, 1 clock cycle for half period aka 20Mhz SCLCK
    spi->control = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable | SonataSpi::ControlInternalLoopback;
    spi->start = len;
   // DebugSPI::log("start value {}:", spi->start);
    size_t bytes_read = 0;
    size_t bytes_sent = 0;

    while (bytes_read < len) {
        if (bytes_sent < len && !(SonataSpi::StatusTxFifoFull & spi->status)) {
            spi->transmitFifo = tx_buffer[bytes_sent];
            DebugSPI::log("tx_value {}:", spi->transmitFifo);
            bytes_sent++;
        }
        if (!(SonataSpi::StatusRxFifoEmpty & spi->status)) {
             DebugSPI::log("rx_value {}:", spi->receiveFifo);
            rx_buffer[bytes_read] = spi->receiveFifo;
            bytes_read++;
        }
        DebugSPI::log("spi status {}",spi->status);
    }

    // Print test result
    bool success = true;
    for (size_t i = 0; i < len; i++) {
        if (tx_buffer[i] != rx_buffer[i]) {
            success = false;
            break;
        }
    }

    if (success) {
        DebugSPI::log("SPI Loopback Test Passed\n");
    } else {
        DebugSPI::log("SPI Loopback Test Failed\n");
    }
        // DebugSPI::log("control_value {}:", spi->control);
        // DebugSPI::log("start value {}:", spi->start);
        // DebugSPI::log("spi status after finish {}:", spi->status);
    }
