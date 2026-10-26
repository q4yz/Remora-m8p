CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS  = -mcpu=cortex-m0plus -mthumb -Os -Wall -Wextra -ffreestanding -nostdlib -nostartfiles

m8p_hello.bin: m8p_hello.elf
	$(OBJCOPY) -O binary $< $@

m8p_hello.elf: m8p_hello.c m8p_hello.ld
	$(CC) $(CFLAGS) -T m8p_hello.ld -o $@ m8p_hello.c -lgcc

clean:
	rm -f m8p_hello.elf m8p_hello.bin
