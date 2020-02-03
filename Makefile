BINS = emu-rv32i

CROSS_COMPILE = riscv32-unknown-elf-
RV32I_CFLAGS = -march=rv32i -mabi=ilp32 -O3 -nostdlib

CFLAGS = -O3 -Wall
LDFLAGS =

all: $(BINS)
	
emu-rv32i: main.c emu-rv32i.c
	$(CC) $(CFLAGS) -o $@ $^

test1.bin: test1.elf
	$(CROSS_COMPILE)objcopy -O binary $< $@

test1.elf: test1.c
	$(CROSS_COMPILE)gcc $(RV32I_CFLAGS) -o $@ $<

check: $(BINS)
	./emu-rv32i

clean:
	$(RM) $(BINS) test1.bin test1.elf
