
#ifndef SWERVOLF_H
#define SWERVOLF_H

#include <stdint.h>

#define BIT_SET(reg,bit) ((reg) = (reg) |  (1<<(bit)))
#define BIT_CLR(reg,bit) ((reg) = (reg) & ~(1<<(bit)))
#define BIT_TGL(reg,bit) ((reg) = (reg) ^  (1<<(bit)))
#define BIT_TST(reg,bit) (0 != (  (reg) &  (1<<(bit))))


typedef struct  {
	//version information
	uint8_t version_patch;
	uint8_t version_minor;
	uint8_t version_major;
	uint8_t version_misc;	//Bit 7 is set when SweRVolf was built from
							//modified sources
							//Bit 6:0 revision since last patch version
	uint32_t version_sha;

	// No effect on hardware
	uint8_t sim_print;		//Outputs a character in simulation.
	uint8_t sim_exit;		//Exits simulation.

	uint8_t init_status;	//Bit 0 = RAM initialization complete. 
							//Bit 1 = RAM initialization reported errors

	uint8_t sw_irq;			//Software-controlled external interrupts
	uint32_t nmi_vec;		//Interrupt vector for NMI 
	//should this be a (*nmi_vec)(void) ?
	
	union {
		uint64_t gpio;		//64 bits of GPIO
		struct {
			uint16_t LEDs;
			uint16_t switches;
			uint32_t unused;
		};
	};
	uint8_t POST_GPIO_GAP[8];

	uint64_t mtime;			//mtime from RISC-V privilege spec
	uint64_t mtimecmp;		//mtimecmp from RISC-V privilege spec

	uint32_t irq_timer_cnt;	//IRQ timer counter
	uint8_t irq_timer_ctrl;	//IRQ timer control

	uint8_t POST_IRQ_GAP[8];

	uint32_t clk_freq_hz;	//Clock frequency of main clock in Hz
} __attribute__((packed)) syscon_t; 

typedef struct {
	//DLAB = 1 specfic registers are only avaliable when (line_con&(1<<8) == 1)
	//DLAB should be 0 when not interacting with those registers
	union {
		struct {
			union { //is both recieve and transmit depending on read/write
				const uint8_t rx;	//don't write this
				uint8_t tx;			//don't read from this
			};
			uint8_t int_enable;
		};
		uint16_t divisor_latch;		//DLAB = 1 specfic 
	};
	union {
		const uint8_t int_status;
		uint8_t FIFO_con;
	};
	uint8_t line_con;
	uint8_t modm_con;
	union {
		const uint8_t line_status;
		uint8_t prescaler_div;		//DLAB = 1 specfic 
	};
	const uint8_t modm_status;
	uint8_t scratch_pad;			//extra byte 
} __attribute__((packed)) UART_con_t; 


typedef struct  {
	uint8_t hexdecode_en;
	uint8_t unused[3];
	union {
		uint8_t ssd_values[4];
		uint32_t single_reg;
	};
} __attribute__((packed)) SSD_con_t; 




volatile syscon_t* const syscon = (void*)(0x80001000);
volatile UART_con_t* const UART_con = (void*)(0x80002000);
volatile SSD_con_t* const SSD_con = (void*)(0x80003000);


#endif /* SWERVOLF_H */