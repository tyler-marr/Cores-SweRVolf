


.global _start
_start:
# 0:   aaaa02b7                lui     t0,0xaaaa0
# 4:   7c029073                csrw    0x7c0,t0 
# 8:   00000067                jr      zero

	lui sp, 0
	addi sp, zero, -16      #set sp to 0x07FF FFFF (bottom of RAM)
	j main