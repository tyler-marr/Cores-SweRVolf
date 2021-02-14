TOOLCHAIN_PREFIX ?= riscv64-unknown-elf-
RESET_VECTOR ?= 0
FLASH_ADDR ?= 0x0
MARCH=rv32i
MABI=ilp32

bootloader.elf: boot_main.S spi_uimage_loader.S hexloader.S
	$(TOOLCHAIN_PREFIX)gcc -nostartfiles -march=$(MARCH)m -mabi=$(MABI) -Tlink.ld -o$@ $^
ethernet.elf: start.o ethernet_example.o uart_support.o
	$(TOOLCHAIN_PREFIX)gcc -nostartfiles -nostdlib -march=$(MARCH) -mabi=$(MABI) -Tlink.ld -o$@ $^
%.elf: %.S
	$(TOOLCHAIN_PREFIX)gcc -nostartfiles -nostdlib -march=$(MARCH) -mabi=$(MABI) -Tlink.ld -o$@ $<
%.elf: start.o %.o 
	$(TOOLCHAIN_PREFIX)gcc -nostartfiles -nostdlib -march=$(MARCH) -mabi=$(MABI) -Tlink.ld -o$@ $^
%.o: %.S
	$(TOOLCHAIN_PREFIX)gcc -nostartfiles -nostdlib -march=$(MARCH) -mabi=$(MABI) -Tlink.ld -c -o$@ $<
%.o: %.c
	$(TOOLCHAIN_PREFIX)gcc -nostartfiles -nostdlib -march=$(MARCH) -mabi=$(MABI) -Tlink.ld -c -g -o$@ $<
%.dis: %.elf %.vh
	$(TOOLCHAIN_PREFIX)objdump -D $< > $@
%.vh: %.bin
	python3 makehex.py $< > $@
%.bin: %.elf
	$(TOOLCHAIN_PREFIX)objcopy -O binary $< $@
%.ihex: %.elf
	$(TOOLCHAIN_PREFIX)objcopy -O ihex $< $@
%.ub: %.bin
	mkimage \
	-A riscv \
	-C none \
	-T standalone \
	-a 0x0 \
	-e $(RESET_VECTOR) \
	-n '$@' \
	-d $< \
	$@





%.ubvh: %.ub
	objcopy --change-addresses $(FLASH_ADDR) -I binary -O verilog $< $@

clean:
	rm -f *.elf *.bin *.vh *.o


.PRECIOUS:	%.vh


