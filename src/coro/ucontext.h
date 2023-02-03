#ifndef _UCONTEXT_H_
#define _UCONTEXT_H_

#define MINSIGSTKSZ 2048

typedef struct sigaltstack {
    void* ss_sp;    /* Stack base or pointer.  */
    int ss_flags;   /* Flags.  */
    size_t ss_size; /* Stack size.  */
} stack_t;

typedef struct {
    unsigned long regs[31];
    unsigned long sp;
    unsigned long pc;
} mcontext_t;

typedef struct __ucontext ucontext_t;

struct __ucontext {
    unsigned long uc_flags;
    ucontext_t* uc_link;
    mcontext_t uc_mcontext;
    stack_t uc_stack;
};

/* uc_flags */
#define _UC_SWAPPED 0x1

#define _UC_MACHINE_STACK(ucp)         ((ucp)->uc_mcontext.sp)
#define _UC_MACHINE_SET_STACK(ucp, sp) _UC_MACHINE_STACK(ucp) = sp

#define _UC_MACHINE_PC(ucp)         ((ucp)->uc_mcontext.pc)
#define _UC_MACHINE_SET_PC(ucp, pc) _UC_MACHINE_PC(ucp) = pc

#define _UC_MACHINE_LR(ucp)         ((ucp)->uc_mcontext.regs[30])
#define _UC_MACHINE_SET_LR(ucp, lr) _UC_MACHINE_LR(ucp) = lr

#define _UC_MACHINE_FP(ucp)         ((ucp)->uc_mcontext.regs[29])
#define _UC_MACHINE_SET_FP(ucp, fp) _UC_MACHINE_FP(ucp) = fp

#define _UC_MACHINE_R21(ucp)          ((ucp)->uc_mcontext.regs[21])
#define _UC_MACHINE_SET_R21(ucp, r21) _UC_MACHINE_R21(ucp) = r21

int getcontext(ucontext_t* ucp);
int setcontext(const ucontext_t* ucp);
void makecontext(ucontext_t* ucp, void (*func)(void), int argc, ...);
int swapcontext(ucontext_t* oucp, const ucontext_t* ucp);

#endif
