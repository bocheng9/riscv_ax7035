/*============================================================================
 * test_ddr_uart.S — 用例 1+2：DDR 自检 + UART MMIO 输出
 *
 * 内存映射：
 *   0x0000_0000 - 0x0FFF_FFFF  DDR3 256MB
 *   0x1110_0000                UART 16550
 *
 * UART 16550 寄存器（偏移）：
 *   0x00  THR (写) / RBR (读)   ; DLAB=1 时是 DLL
 *   0x04  IER                   ; DLAB=1 时是 DLM
 *   0x08  FCR (写) / IIR (读)
 *   0x0C  LCR
 *   0x14  LSR  (bit5=THRE 发送保持寄存器空, bit6=TEMT 发送器空)
 *
 * 测试流程：
 *   1. 初始化 UART
 *   2. 打印 "DDR"
 *   3. 写 DDR 0x1000 处 4 个字，读回比对
 *   4. 成功打印 " OK" + 换行，失败打印 " ERR"
 *   5. 死循环
 *============================================================================*/

#define UART_BASE   0x11100000
#define UART_THR    (UART_BASE + 0x00)
#define UART_LCR    (UART_BASE + 0x0C)
#define UART_LSR    (UART_BASE + 0x14)

    .section .text
    .globl _start
_start:
    /*=== 1. 初始化 UART（8N1，无分频设置）===*/
    li      t0, UART_LCR
    li      t1, 0x03            /* 8 bits, no parity, 1 stop */
    sb      t1, 0(t0)

    /*=== 2. 打印 "DDR" ===*/
    li      a0, 'D'
    jal     ra, uart_putc
    li      a0, 'D'
    jal     ra, uart_putc
    li      a0, 'R'
    jal     ra, uart_putc

    /*=== 3. DDR 自检：写 4 个字到 0x1000 ===*/
    li      s0, 0x1000          /* DDR 测试基址 */

    /* 写 pattern 0xDEADBEEF */
    li      t1, 0xDEADBEEF
    sw      t1, 0(s0)

    /* 写 pattern 0x12345678 */
    li      t1, 0x12345678
    sw      t1, 4(s0)

    /* 写 pattern 0xA5A5A5A5 */
    li      t1, 0xA5A5A5A5
    sw      t1, 8(s0)

    /* 写 pattern 0x00000000 */
    li      t1, 0x00000000
    sw      t1, 12(s0)

    /*=== 4. 读回比对 ===*/
    /* 检查 0x1000 == 0xDEADBEEF */
    li      s1, 0xDEADBEEF
    lw      t2, 0(s0)
    bne     t2, s1, test_fail

    /* 检查 0x1004 == 0x12345678 */
    li      s1, 0x12345678
    lw      t2, 4(s0)
    bne     t2, s1, test_fail

    /* 检查 0x1008 == 0xA5A5A5A5 */
    li      s1, 0xA5A5A5A5
    lw      t2, 8(s0)
    bne     t2, s1, test_fail

    /* 检查 0x100C == 0x00000000 */
    li      s1, 0x00000000
    lw      t2, 12(s0)
    bne     t2, s1, test_fail

    /*=== 5. 全部通过，打印 " OK" ===*/
    li      a0, ' '
    jal     ra, uart_putc
    li      a0, 'O'
    jal     ra, uart_putc
    li      a0, 'K'
    jal     ra, uart_putc
    li      a0, '\n'
    jal     ra, uart_putc
    j       done

test_fail:
    li      a0, ' '
    jal     ra, uart_putc
    li      a0, 'E'
    jal     ra, uart_putc
    li      a0, 'R'
    jal     ra, uart_putc
    li      a0, 'R'
    jal     ra, uart_putc
    li      a0, '\n'
    jal     ra, uart_putc

done:
    /* 死循环，同时点亮一个"完成"标志方便 ILA/LED 观察 */
    li      s2, 0xCAFEBABE
    li      t0, 0x2000
    sw      s2, 0(t0)
1:  j       1b

/*============================================================================
 * uart_putc — 发送一个字符（轮询 THRE）
 *   输入：a0 = 字符
 *   破坏：t0, t1
 *============================================================================*/
uart_putc:
    /* ★ 调试版：跳过 LSR 轮询，直接发送并软件延时 */
    li      t0, UART_THR
    sb      a0, 0(t0)
    li      t2, 2000
2:  addi    t2, t2, -1
    bnez    t2, 2b
    ret
