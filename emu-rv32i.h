/* emulate RAM */
#define RAM_SIZE 0x10000
uint8_t ram[RAM_SIZE];

/* CPU state */
uint32_t pc;
uint32_t reg[32];
uint32_t ram_start;

void riscv_cpu_interp_x32(int n_cycles);
