/*
* Hello world for SweRVolf
*
* Prints a fancy string and exits afterwards
*
*/
#define CONSOLE_ADDR 0x80001008
#define HALT_ADDR    0x80001009
#define NTP_BASE     0x80003000

    /*
    a0 = Console address
    a1 = String address
    t0 = Character to write
    */

__asm (".global _start");
__asm ("_start:");

__asm ("lui sp, 0x8000");
__asm ("addi sp, sp, -1");      //set sp to 0x07FF FFFF

__asm ("j main");

int main( void )
{
    char *str = " This was printed from within a c programe!\n";
    char *s = str;
    while (*s++) {
        *(volatile char*)(CONSOLE_ADDR) = *s;
    }

    for (int i = 0; i < 16; i++){
        *(volatile unsigned char*)((NTP_BASE+i)) = (unsigned char)0x7F;
    }

    //write to HALT_ADDR will halt the SIM
    *(volatile char*)(HALT_ADDR) = 0;
    return 0;
}


