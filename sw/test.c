/*
* Hello world for SweRVolf
*
* Prints a fancy string and exits afterwards
*
*/
#include <stdint.h>
#define SYSCON_BASE         ((volatile uint8_t*)(0x00003000))
#define CONSOLE_OFFSET      0x8
#define HALT_OFFSET         0x9
#define CONSOLE_WRITE(c)    SYSCON_BASE[CONSOLE_OFFSET]=c
#define HALT_CPU            SYSCON_BASE[HALT_OFFSET]=0

#define UART_BASE           ((volatile uint32_t*)(0x00001000))
#define UART_TX_OFFSET      0x0
#define UART_RX_OFFSET      0x0
#define UART_LSR_OFFSET     0x5

#define UART_LSR_DR    (1<<0)  // Data ready
#define UART_LSR_OE    (1<<1)  // Overrun Error
#define UART_LSR_PE    (1<<2)  // Parity Error
#define UART_LSR_FE    (1<<3)  // Framing Error
#define UART_LSR_BI    (1<<4)  // Break interrupt
#define UART_LSR_THRE  (1<<5)  // Transmit FIFO is empty
#define UART_LSR_TEMT  (1<<6)  // Transmitter Empty indicator
#define UART_LSR_EI    (1<<7)  // Error in receive FIFO


#define UART_TX(c)          UART_BASE[UART_TX_OFFSET] = c
#define UART_RX             UART_BASE[UART_RX_OFFSET]
#define UART_LSR            UART_BASE[UART_LSR_OFFSET]


#define NTP_BASE            ((volatile uint8_t*)0x00002000)


void askHalt(void) {
    char *str = " Send any char to halt\n";
    char *s = str;
    while (*s++) {
        UART_TX(*s);
    }

    while (!(UART_LSR & UART_LSR_DR)) {
    }

    HALT_CPU;

}

void echo_num(uint32_t n){
    for (int i = 0; i < n; i++){
        while (!(UART_LSR & UART_LSR_DR));
        UART_TX(UART_RX);
    }
}

int main( void )
{
    uint8_t *str = {" This was printed from within a c program!\n"};
    uint8_t *s = str;
    while (*s++) {
        CONSOLE_WRITE(*s);
        UART_TX(*s);
    }

    for (int i = 0; i < 16; i++){
        CONSOLE_WRITE('\n');
        ((uint8_t *)NTP_BASE)[i] = 0x08;
        ((uint16_t*)NTP_BASE)[i] = 0x1616;
        ((uint32_t*)NTP_BASE)[i] = 0x32323232;
        ((uint64_t*)NTP_BASE)[i] = 0x6464646464646464;
        break;
    }

    echo_num(10);
    
    askHalt();

    HALT_CPU;
    return 0;
}


