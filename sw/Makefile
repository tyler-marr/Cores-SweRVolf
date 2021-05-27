TOOLCHAIN_PREFIX ?= riscv64-unknown-elf-
RESET_VECTOR ?= 0
FLASH_ADDR ?= 0x0

bootloader.elf: boot_main.S spi_uimage_loader.S hexloader.S
	$(TOOLCHAIN_PREFIX)gcc -nostartfiles -march=rv32im -mabi=ilp32 -Tlink.ld -o$@ $^
%.elf: %.S
	$(TOOLCHAIN_PREFIX)gcc -nostartfiles -march=rv32im -mabi=ilp32 -Tlink.ld -o$@ $<
# start.o must be linked first such that the first instruction
# is a valid entry point 
%.elf: start.o %.o 
	$(TOOLCHAIN_PREFIX)gcc -DDEBUG -g -nostartfiles -march=rv32im -mabi=ilp32 -Tlink.ld -fPIC -o$@ $^
%.o: %.S
	$(TOOLCHAIN_PREFIX)gcc -DDEBUG -g -nostartfiles -nostdlib -march=rv32imc -mabi=ilp32 -Tlink.ld -c -fPIC -o$@ $<
%.o: %.c
	$(TOOLCHAIN_PREFIX)gcc -DDEBUG -g -nostartfiles -nostdlib -march=rv32imc -mabi=ilp32 -Tlink.ld -c -fPIC -o$@ $<
%.dis: %.elf %.vh
	$(TOOLCHAIN_PREFIX)objdump -S $< > $@
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
