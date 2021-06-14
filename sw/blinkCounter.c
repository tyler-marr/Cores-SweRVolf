
#include <stdint.h>
#include <stddef.h>

#include "swervolf.h"


void delay(uint32_t count){
	for (int i = 0; i < count; i++){
		//spin
	}
	return;
}

#define CLK_HZ (25000000)
#define BAUDRATE (115200)
void uart_init(){

	UART_con->line_con =  0x80; 			//enable DLAB
	uint16_t div_latch = ((CLK_HZ / 16) / BAUDRATE);
	UART_con->divisor_latch_LS_byte = div_latch & 0xFF;
	UART_con->divisor_latch_MS_byte = div_latch >> 8;
	UART_con->line_con = 3;					//word len (8bits)

	UART_con->FIFO_con = 0x87;				//enable and clear FIFO
	UART_con->int_enable = 0;				//disable all interrupts

}

void main(){

	uart_init();

	uint64_t counter = 0;
	uint16_t slider = 0;
	uint8_t lastChar = 0;

	uint32_t rawSSD = 0;

	while (1) {

		if (!BIT_TST(syscon->switches,0)){
			SSD_con->hexdecode_en = 1;

			slider = slider << 1;

			if (slider == 0){
				rawSSD = ((rawSSD << (4)) + (7 + (rawSSD & 0xF)));
				SSD_con->single_reg = rawSSD;
			}

			if (BIT_TST(slider,15)) {
				BIT_CLR(slider, 0);	//clear first bit 
			} else {
				BIT_SET(slider, 0);	//set first bit 
			}

			syscon->LEDs = slider;
			
		} else {

			SSD_con->hexdecode_en = 0;
			syscon->LEDs = counter;
			SSD_con->single_reg = counter;
		}
		counter++;

		delay(syscon->switches & 0xFF00);

		if (BIT_TST(UART_con->line_status,0)) {
			//if data ready to read
			lastChar = UART_con->rx;
			UART_con->tx = lastChar;
		}
	}
}