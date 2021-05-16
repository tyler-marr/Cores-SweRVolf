
#simple entry point to your program

#make file links the object file from this first so the first instruction
#is a valid entry point into your program

# Also initilzes the stackpointer to 0x07FF FFFF (bottom of RAM)


.global _start


_start:
	lui sp, 0x8000
	addi sp, sp, -1      #set sp to 0x07FF FFFF (bottom of RAM)
	j main
	