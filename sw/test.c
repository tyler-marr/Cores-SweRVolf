/*
* Hello world for SweRVolf
*
* Prints a fancy string and exits afterwards
*
*/
#include <stdint.h>
#define SYSCON_BASE         ((volatile uint8_t *)(0x00003000))
#define CONSOLE_OFFSET      0x8
#define HALT_OFFSET         0x9
#define CONSOLE_TX(c)       SYSCON_BASE[CONSOLE_OFFSET]=c
#define HALT_CPU            SYSCON_BASE[HALT_OFFSET]=0

#define UART_BASE           ((volatile uint32_t volatile*)(0x00001000))
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


// #define UART_TX(c)          UART_BASE[UART_TX_OFFSET] = c
// #define UART_RX             UART_BASE[UART_RX_OFFSET]
// #define UART_LSR            UART_BASE[UART_LSR_OFFSET]

#define UART_TX(c)          (*((volatile uint32_t *)(0x00001000))) = c 
#define UART_RX             (*((volatile uint32_t *)(0x00001000)))
#define UART_LSR            (*((volatile uint32_t *)(0x00001014)))


#define NTP_BASE            ((volatile uint8_t*)0x00002000)

void putC(uint8_t c){
    UART_TX(c);
}

void putS(uint8_t *s){
    while (*s) {
        //should need to wait here for ready char but DPI is ALWAYS ready
        putC(*s);
        s++;
    }
}

void con_putC(uint8_t c){
    CONSOLE_TX(c);
}

void con_putS(uint8_t *s){
    while (*s) {
        con_putC(*s);
        s++;
    }
}

uint8_t hexToAscii(uint8_t c){
    if (c < 10){
        return c + '0';
    } else {
        return c + 'A' - 10;
    }
}

uint8_t uint8_to_hex(uint32_t n) {
    con_putS("0x");
    con_putC(hexToAscii((n >> 4)&0xf));
    con_putC(hexToAscii((n >> 0)&0xf));
    con_putC('\n');
}

void wait_UART_ready(void) {
    while ( !(UART_LSR & UART_LSR_DR ));
    return;
}


void askHalt(void) {
    
    //putS("Send '!' to halt\n");

    uint8_t c = 0;
    while(1){
        wait_UART_ready();
        c = UART_RX;
        if ( c == '!') {
            HALT_CPU;
        }
        UART_TX(c);
        uint8_to_hex(c);
    }
}

int main( void )
{   
    askHalt();
    putS("This was printed from within a c program!\n");

    for (int i = 0; i < 16; i++){
        con_putC('\n');
        ((uint8_t *)NTP_BASE)[i] = 0x08;
        ((uint16_t*)NTP_BASE)[i] = 0x1616;
        ((uint32_t*)NTP_BASE)[i] = 0x32323232;
        ((uint64_t*)NTP_BASE)[i] = 0x6464646464646464;
        break;
    }

    askHalt();

    HALT_CPU;
    return 0;
}


