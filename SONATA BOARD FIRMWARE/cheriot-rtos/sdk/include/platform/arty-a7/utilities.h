
#define DATA_SIZE 1000      
#define RX_BUFFER_SIZE DATA_SIZE 
#define UART_FRAME_SIZE RX_BUFFER_SIZE + 6 // 2 start bytes and 2 end bytes and 2 bytes for health tests reuslt 
#define ENTROPY_BUFFER_SIZE  DATA_SIZE/2
#define WINDOW_SIZE 20
#define CUTT_OFF_VALUE 6
#define TIMEOUT 5000


// #pragma once 
// void extract_entropy_data(uint8_t* rx_buffer, uint16_t* entropy_buffer, size_t entropy_buffer_size) {
//     for (size_t i = 0; i < ENTROPY_BUFFER_SIZE; i++) {
//         entropy_buffer[i] = rx_buffer[(i * 2)] | (rx_buffer[(i * 2) + 1] << 8);
//     }
// }





