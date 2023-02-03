#include <stddef.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>

#include "ucontext.h"

void ctx_wrapper();

void makecontext(ucontext_t* ucp, void (*func)(void), int argc, ...)
{
    va_list ap;
    unsigned long* stack_top;
    int i;

    if (ucp == NULL) {
        return;
    } else if ((ucp->uc_stack.ss_sp == NULL) ||
               (ucp->uc_stack.ss_size < MINSIGSTKSZ)) {
        _UC_MACHINE_SET_STACK(ucp, 0);
        return;
    }

    stack_top = (unsigned long*)((uintptr_t)ucp->uc_stack.ss_sp +
                                 ucp->uc_stack.ss_size);
    stack_top = (unsigned long*)((uintptr_t)stack_top & ~0xf);

    if (argc > 8) stack_top -= argc - 8;

    _UC_MACHINE_SET_STACK(ucp, (unsigned long)stack_top);
    _UC_MACHINE_SET_PC(ucp, (unsigned long)func);
    _UC_MACHINE_SET_LR(ucp, (unsigned long)ctx_wrapper);
    _UC_MACHINE_SET_FP(ucp, 0);
    _UC_MACHINE_SET_R21(ucp, (unsigned long)ucp);

    va_start(ap, argc);

    for (i = 0; i < argc && i < 8; i++) {
        ucp->uc_mcontext.regs[i] = va_arg(ap, uintptr_t);
    }

    for (; i < argc; i++) {
        *stack_top++ = va_arg(ap, uintptr_t);
    }

    va_end(ap);

    if (stack_top == ucp->uc_stack.ss_sp) {
        _UC_MACHINE_SET_STACK(ucp, 0);
    }
}

int swapcontext(ucontext_t* oucp, const ucontext_t* ucp)
{
    int r;

    if ((oucp == NULL) || (ucp == NULL)) {
        return -1;
    }

    if (_UC_MACHINE_STACK(ucp) == 0) {
        return -1;
    }

    oucp->uc_flags &= ~_UC_SWAPPED;
    r = getcontext(oucp);
    if ((r == 0) && !(oucp->uc_flags & _UC_SWAPPED)) {
        oucp->uc_flags |= _UC_SWAPPED;
        r = setcontext(ucp);
    }

    return r;
}

void resumecontext(ucontext_t* ucp)
{
    if (ucp->uc_link == NULL) exit(0);

    (void)setcontext((const ucontext_t*)ucp->uc_link);

    exit(1);
}
