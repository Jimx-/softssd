#ifndef _TLS_H_
#define _TLS_H_

#include <config.h>
#include <thread.h>

#include <stddef.h>

#ifndef __tls_offset

extern size_t __tls_offset[NR_WORKER_THREADS];
#define tls_offset(tid) __tls_offset[tid]

#endif /* __tls_offset */

#define TLS_BASE_SECTION ".tlsdata"

#define get_tls_var_ptr(tid, name)                \
    ({                                            \
        unsigned long __ptr;                      \
        __ptr = (unsigned long)(&(name));         \
        (typeof(name)*)(__ptr + tls_offset(tid)); \
    })

#define get_tls_var(tid, name)  (*get_tls_var_ptr(tid, name))
#define get_local_var_ptr(name) get_tls_var_ptr((worker_self()->tid), name)
#define get_local_var(name)     get_tls_var((worker_self()->tid), name)

#define DECLARE_TLS(type, name) \
    extern __attribute__((section(TLS_BASE_SECTION))) __typeof__(type) name

#define DEFINE_TLS(type, name) \
    __attribute__((section(TLS_BASE_SECTION))) __typeof__(type) name

#endif /* _TLS_H_ */
