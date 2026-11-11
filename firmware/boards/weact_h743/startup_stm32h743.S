/* Reset entry for the STM32H743 (board design 3.2).
   Right after power-on the device is in Run* mode: no RAM may be written until the supply is
   configured and PWR_CSR1.ACTVOSRDY is set (RM0433 6.4.1; STM32CubeH7 V1.13.0 calls
   ExitRun0Mode before copying .data). So this code touches registers only -- no stack, no
   RAM -- until ACTVOSRDY, then copies .data, clears .bss, enables the FPU and enters C. */
    .syntax unified
    .cpu cortex-m7
    .thumb

    .equ PWR_CSR1,        0x58024804
    .equ PWR_CR3,         0x5802480C
    .equ PWR_CR3_LDOEN,   (1 << 1)
    .equ PWR_CSR1_ACTVOS, (1 << 13)
    .equ SCB_CPACR,       0xE000ED88
    .equ ACTVOS_SPINS,    2000000      /* about 30 ms at the 64 MHz reset clock (HSI) */

    .section .text.Reset_Handler, "ax", %progbits
    .global Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    ldr   r0, =_estack
    mov   sp, r0                        /* set, but not used before ACTVOSRDY */

    /* LDO supply (the H743 has no SMPS); written once after reset. */
    ldr   r0, =PWR_CR3
    ldr   r1, [r0]
    orr   r1, r1, #PWR_CR3_LDOEN
    str   r1, [r0]

    ldr   r0, =PWR_CSR1
    ldr   r2, =ACTVOS_SPINS
1:  ldr   r1, [r0]
    tst   r1, #PWR_CSR1_ACTVOS
    bne   2f
    subs  r2, r2, #1
    bne   1b
    b     .                             /* never ready: stop; the DRV8244 pulls keep outputs off */

2:  /* .data from flash, .bss to zero (word copies). */
    ldr   r0, =_sdata
    ldr   r1, =_edata
    ldr   r2, =_sidata
3:  cmp   r0, r1
    bhs   4f
    ldr   r3, [r2], #4
    str   r3, [r0], #4
    b     3b
4:  ldr   r0, =_sbss
    ldr   r1, =_ebss
    movs  r3, #0
5:  cmp   r0, r1
    bhs   6f
    str   r3, [r0], #4
    b     5b

6:  /* FPU: full access to CP10/CP11 before any C code may use it. */
    ldr   r0, =SCB_CPACR
    ldr   r1, [r0]
    orr   r1, r1, #(0xF << 20)
    str   r1, [r0]
    dsb
    isb

    bl    board_main
    b     .
    .size Reset_Handler, . - Reset_Handler
