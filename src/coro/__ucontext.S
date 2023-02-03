.section .text

#define SZREG 8

#define oX0 (SZREG * 2)
#define oPC	(oX0 + SZREG * 32)
#define oSP	(oX0 + SZREG * 31)

    .global getcontext
    .global setcontext
    .global ctx_wrapper

    .extern resumecontext

getcontext:
    str xzr,      [x0, oX0 + 0 * SZREG]
    stp x18, x19, [x0, oX0 + 18 * SZREG]
    stp x20, x21, [x0, oX0 + 20 * SZREG]
    stp x22, x23, [x0, oX0 + 22 * SZREG]
    stp x24, x25, [x0, oX0 + 24 * SZREG]
    stp x26, x27, [x0, oX0 + 26 * SZREG]
    stp x28, x29, [x0, oX0 + 28 * SZREG]
    str x30,      [x0, oX0 + 30 * SZREG]

    str x30, [x0, oPC]

    mov x2, sp
    str x2, [x0, oSP]

    mov x0, 0
    RET

setcontext:
    ldp        x18, x19, [x0, oX0 + 18 * SZREG]
    ldp        x20, x21, [x0, oX0 + 20 * SZREG]
    ldp        x22, x23, [x0, oX0 + 22 * SZREG]
    ldp        x24, x25, [x0, oX0 + 24 * SZREG]
    ldp        x26, x27, [x0, oX0 + 26 * SZREG]
    ldp        x28, x29, [x0, oX0 + 28 * SZREG]
    ldr     x30,      [x0, oX0 + 30 * SZREG]
    ldr     x2, [x0, oSP]
    mov        sp, x2

    ldr     x16, [x0, oPC]

    ldp        x2, x3, [x0, oX0 + 2 * SZREG]
    ldp        x4, x5, [x0, oX0 + 4 * SZREG]
    ldp        x6, x7, [x0, oX0 + 6 * SZREG]
    ldp        x0, x1, [x0, oX0 + 0 * SZREG]

    br        x16

ctx_wrapper:
    mov x0, x21
    b   resumecontext
