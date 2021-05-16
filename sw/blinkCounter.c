
#include <stdint.h>
#include <stddef.h>

#define HALT_ADDR    0x80001009
#define UART_BASE    0x80002000

#define REG_BRDL (4*0x00) /* Baud rate divisor (LSB)        */
#define REG_IER (4*0x01)  /* Interrupt enable reg.          */
#define REG_FCR (4*0x02)  /* FIFO control reg.              */
#define REG_LCR (4*0x03)  /* Line control reg.              */
#define REG_LSR (4*0x05)  /* Line status reg.               */
#define LCR_CS8 0x03   /* 8 bits data size */
#define LCR_1_STB 0x00 /* 1 stop bit */
#define LCR_PDIS 0x00  /* parity disable */

#define LSR_THRE 0x20
#define FCR_FIFO 0x01    /* enable XMIT and RCVR FIFO */
#define FCR_RCVRCLR 0x02 /* clear RCVR FIFO */
#define FCR_XMITCLR 0x04 /* clear XMIT FIFO */
#define FCR_MODE0 0x00 /* set receiver in mode 0 */
#define FCR_MODE1 0x08 /* set receiver in mode 1 */
#define FCR_FIFO_8 0x80  /* 8 bytes in RCVR FIFO */



#define GPIO_BASE (0x80001010)
#define LED (*(volatile uint16_t*)(GPIO_BASE))
#define SWITCHES (*(volatile uint16_t*)(GPIO_BASE+2))

void delay(uint32_t count){
	for (int i = 0; i < count; i++){

	}
	return;
}



void main(){

	uint64_t counter = 0;
	uint16_t slider = 0;
	uint8_t lastChar = 0;

	

	while (1) {

		if (!(SWITCHES & (1<<1))){
			slider = slider << 1;
			if (slider & 0x8000) {
				slider &= ~1;
			} else {
				slider |= 1;
			}
			LED = slider;
		} else {
			counter++;
			LED = counter;
		}

		delay(SWITCHES & 0xFF00);

		if ((*(volatile uint8_t*)(UART_BASE+REG_LSR)) & 1) {
			//if data ready to read
			lastChar = (*(volatile uint8_t*)(UART_BASE));
			(*(volatile uint8_t*)(UART_BASE)) = lastChar;
		}
	}
}