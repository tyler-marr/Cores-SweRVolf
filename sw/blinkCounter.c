
#include <stdint.h>
#include <stddef.h>

#include "swervolf.h"


void delay(uint32_t count){
	for (int i = 0; i < count; i++){
		//spin
	}
	return;
}



void main(){

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