


.global _start
_start:
	lui sp, 0
	addi sp, zero, -16      #set sp to 0x07FF FFFF (bottom of RAM)
	j main