BINARY = fancyblink.elf
LDSCRIPT = stm32vl-discovery.ld
CC=arm-none-eabi-gcc
CFLAGS=-O2 -mthumb -mcpu=cortex-m3 -DSTM32F1 -mfix-cortex-m3-ldrd  -ffunction-sections -g -I/home/graham/work/projects/secure/stm32-discovery/libopencm3/include
LDFLAGS= --static -nostartfiles -T $(LDSCRIPT) -Wl,--gc-sections  -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd -L/home/graham/work/projects/secure/stm32-discovery/libopencm3/lib
LIBS=-lopencm3_stm32f1 -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group


all: $(BINARY)

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

%.elf: %.o
	$(CC)  $(LDFLAGS) $< -o $@ $(LIBS)

