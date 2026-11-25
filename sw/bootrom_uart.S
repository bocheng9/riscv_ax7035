    #========================================================================
    # bootrom_uart.S — BootROM loader: receive a program over UART into DDR
    #
    # Protocol (all little endian):
    #   1. host sends 4 bytes  -> payload length in bytes, excluding padding
    #   2. host sends N bytes  -> the program image
    #   3. loader rounds up to a word boundary, then jumps to DDR_BASE
    #
    # The receiver is a byte interface, so each byte is placed at the right
    # lane with sb; no alignment assumption is made about the incoming stream.
    #
    # Memory map
    #   0x0000_0000  this loader (BootROM)
    #   0x1110_0000  UART: +0 data, +5 line status
    #   0x8000_0000  DDR, where the image is placed
    #========================================================================
    .section .text
    .globl _start

    .equ UART_BASE,  0x11100000
    .equ UART_DATA,  0x11100000
    .equ UART_LSR,   0x11100014     # bit 0 = data ready
    .equ DDR_BASE,   0x80000000

_start:
    #------------------------------------------------------------------
    # Nothing to set up before the length arrives: the receiver is a
    # plain polled byte interface.
    #------------------------------------------------------------------
    li      s0, UART_LSR

    #--------------------------------------------------------------
    # Read the 4-byte length into t2.
    #--------------------------------------------------------------
    li      t2, 0
    li      t3, 4

len_loop:
    beqz    t3, len_done
    lw      t4, 0(s0)               # line status
    andi    t4, t4, 1
    beqz    t4, len_loop            # no byte yet

    li      t5, UART_DATA
    lw      t4, 0(t5)               # data byte
    andi    t4, t4, 0xFF

    # Little endian: the first byte received is the least significant, so the
    # shift is (4 - remaining) * 8 rather than (remaining - 1) * 8.
    li      t6, 4
    sub     t6, t6, t3
    slli    t6, t6, 3
    sll     t4, t4, t6
    or      t2, t2, t4

    addi    t3, t3, -1
    j       len_loop

len_done:
    #------------------------------------------------------------------
    # Copy t2 bytes from UART to DDR_BASE.
    #   s1 = destination pointer
    #   a0 = bytes still to receive
    #------------------------------------------------------------------
    li      s1, DDR_BASE
    mv      a0, t2

    li      s7, 0               # words stored
    li      s8, 0               # mismatches
    li      s9, 0               # first bad address
    li      s10, 0              # value read there
    li      s11, 0              # value expected there

copy_loop:
    li      a6, 4              # bytes still to gather for this word
    li      a7, 0              # word being assembled

gather_word:
    beqz    a6, word_ready

wait_byte:
    lw      t4, 0(s0)
    andi    t4, t4, 1
    beqz    t4, wait_byte

    li      t5, UART_DATA
    lw      t4, 0(t5)
    andi    t4, t4, 0xFF

    # Little endian: first byte of the image is the low byte of the word.
    li      t6, 4
    sub     t6, t6, a6
    slli    t6, t6, 3
    sll     t4, t4, t6
    or      a7, a7, t4

    addi    a6, a6, -1
    j       gather_word

word_ready:
    # Two words at a time so every store address is 8-byte aligned. This keeps
    # the access inside the low half of a 64-bit beat every time.
    sw      a7, 0(s1)
    mv      s6, a7              # remember what this word should read as
    lw      a5, 0(s1)           # read it straight back
    addi    s7, s7, 1           # words stored
    beq     a5, s6, first_ok
    addi    s8, s8, 1           # mismatches
    bnez    s9, first_ok        # only the first bad address is kept
    mv      s9, s1
    mv      s10, a5
    mv      s11, s6
first_ok:
    li      a6, 4
    li      a7, 0

gather_word2:
    beqz    a6, word2_ready

wait_byte2:
    lw      t4, 0(s0)
    andi    t4, t4, 1
    beqz    t4, wait_byte2

    li      t5, UART_DATA
    lw      t4, 0(t5)
    andi    t4, t4, 0xFF

    li      t6, 4
    sub     t6, t6, a6
    slli    t6, t6, 3
    sll     t4, t4, t6
    or      a7, a7, t4

    addi    a6, a6, -1
    j       gather_word2

word2_ready:
    sw      a7, 4(s1)
    mv      s6, a7
    addi    t0, s1, 4
    lw      a5, 0(t0)
    addi    s7, s7, 1
    beq     a5, s6, second_ok
    addi    s8, s8, 1
    bnez    s9, second_ok
    mv      s9, t0
    mv      s10, a5
    mv      s11, s6
second_ok:
    addi    s1, s1, 8
    addi    a0, a0, -8
    bgtz    a0, copy_loop

copy_done:
    #------------------------------------------------------------------
    # Report the per-word verification. Every transmit waits for the
    # console to be idle first, and nothing here calls a subroutine: a
    # jal inside a subroutine would overwrite ra.
    #------------------------------------------------------------------
    li      a2, UART_DATA
    li      a4, 0xAA
    mv      t3, a4
rw1:
    lw      a4, 0(s0)
    andi    a4, a4, 0x20
    beqz    a4, rw1
    sw      t3, 0(a2)
    li      a4, 0x55
    mv      t3, a4
rw2:
    lw      a4, 0(s0)
    andi    a4, a4, 0x20
    beqz    a4, rw2
    sw      t3, 0(a2)
    mv      a1, s7
    li      a3, 4
field_loop_s7:
    beqz    a3, field_done_s7
    andi    a4, a1, 0xFF
    mv      t3, a4
rw3:
    lw      a4, 0(s0)
    andi    a4, a4, 0x20
    beqz    a4, rw3
    sw      t3, 0(a2)
    srli    a1, a1, 8
    addi    a3, a3, -1
    j       field_loop_s7
field_done_s7:
    mv      a1, s8
    li      a3, 4
field_loop_s8:
    beqz    a3, field_done_s8
    andi    a4, a1, 0xFF
    mv      t3, a4
rw4:
    lw      a4, 0(s0)
    andi    a4, a4, 0x20
    beqz    a4, rw4
    sw      t3, 0(a2)
    srli    a1, a1, 8
    addi    a3, a3, -1
    j       field_loop_s8
field_done_s8:
    mv      a1, s9
    li      a3, 4
field_loop_s9:
    beqz    a3, field_done_s9
    andi    a4, a1, 0xFF
    mv      t3, a4
rw5:
    lw      a4, 0(s0)
    andi    a4, a4, 0x20
    beqz    a4, rw5
    sw      t3, 0(a2)
    srli    a1, a1, 8
    addi    a3, a3, -1
    j       field_loop_s9
field_done_s9:
    mv      a1, s10
    li      a3, 4
field_loop_s10:
    beqz    a3, field_done_s10
    andi    a4, a1, 0xFF
    mv      t3, a4
rw6:
    lw      a4, 0(s0)
    andi    a4, a4, 0x20
    beqz    a4, rw6
    sw      t3, 0(a2)
    srli    a1, a1, 8
    addi    a3, a3, -1
    j       field_loop_s10
field_done_s10:
    mv      a1, s11
    li      a3, 4
field_loop_s11:
    beqz    a3, field_done_s11
    andi    a4, a1, 0xFF
    mv      t3, a4
rw7:
    lw      a4, 0(s0)
    andi    a4, a4, 0x20
    beqz    a4, rw7
    sw      t3, 0(a2)
    srli    a1, a1, 8
    addi    a3, a3, -1
    j       field_loop_s11
field_done_s11:
    li      a4, 0x5A
    mv      t3, a4
rw8:
    lw      a4, 0(s0)
    andi    a4, a4, 0x20
    beqz    a4, rw8
    sw      t3, 0(a2)
    j       readback_done
readback_done:
    #------------------------------------------------------------------
    # Tell the debug logic the program is about to run. Before this point the
    # register file holds unknown values, so anything watching gp would record
    # noise; this write is the clean starting edge.
    #------------------------------------------------------------------
    li      t0, UART_BASE
    li      t1, 1
    sw      t1, 0x1C(t0)

    #------------------------------------------------------------------
    # Jump into the image.
    #------------------------------------------------------------------
    li      t0, DDR_BASE
    jr      t0
