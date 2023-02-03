#ifndef _PROTO_H_
#define _PROTO_H_

#include <stddef.h>
#include "xscugic.h"

#include <types.h>
#include <list.h>

struct iov_iter;
struct flash_transaction;
struct flash_address;

/* alloc.c */
void mem_init(int mem_zone, unsigned long mem_start, size_t free_mem_size);
void* alloc_mem(size_t memsize, size_t alignment, int flags);
void* alloc_pages(size_t nr_pages, int flags);
void free_mem(void* base, unsigned long len);

/* amu.c */
void amu_init(size_t capacity, int reset);
int amu_submit_transaction(struct flash_transaction* txn);
void amu_shutdown(void);
int amu_dispatch(struct list_head* txn_list);

/* block_mananger.c */
void bm_init(int wipe, int full_scan);
void bm_shutdown(void);
void bm_alloc_page(unsigned int nsid, struct flash_address* addr, int for_gc,
                   int for_mapping);
void bm_invalidate_page(struct flash_address* addr);

/* data_cache.c */
void dc_init(size_t capacity);
int dc_process_request(struct user_request* req);
void dc_flush_ns(unsigned int nsid);
void dc_flush_all(void);
void dc_report_stats(void);

/* ftl.c */
void ftl_init(void);
int ftl_process_request(struct user_request* req);
void ftl_shutdown(int abrupt);
void ftl_report_stats(void);

/* intr.c */
int intr_setup(void);
void intr_enable(void);
void intr_disable(void);
int intr_setup_irq(u16 intr_id, int trigger_type, Xil_InterruptHandler handler,
                   void* cb_data);
int intr_enable_irq(u16 intr_id);
int intr_disable_irq(u16 intr_id);
void intr_disconnect_irq(u16 intr_id);
unsigned long intr_save(void);
void intr_restore(unsigned long flags);

/* ipi.c */
int ipi_setup(void);
void ipi_clear_status(void);

/* main.c */
void panic(const char* fmt, ...);
int submit_flash_transaction(struct flash_transaction* txn);
int ecc_calculate(const u8* data, size_t data_length, u8* code,
                  size_t code_length, size_t offset);
int ecc_correct(u8* data, size_t data_length, const u8* code,
                size_t code_length, uint64_t err_bitmap);

/* nvme.c */
void nvme_init(void);
void nvme_worker_main(void);
int nvme_dma_read(struct user_request* req, struct iov_iter* iter,
                  size_t count);
int nvme_dma_write(struct user_request* req, struct iov_iter* iter,
                   size_t count);

/* pcie.c */
int pcie_setup(void);
void pcie_stop(void);
int pcie_dma_read_iter(unsigned long addr, struct iov_iter* iter, size_t count);
int pcie_dma_write_iter(unsigned long addr, struct iov_iter* iter,
                        size_t count);
int pcie_dma_read(unsigned long addr, u8* buffer, size_t count);
int pcie_dma_write(unsigned long addr, const u8* buffer, size_t count);
int pcie_send_completion(unsigned long addr, u16 requester_id, u8 tag,
                         const u8* buffer, size_t count);
int pcie_send_msi(u16 vector);

/* slab.c */
void slabs_init();
void* slaballoc(size_t bytes);
void slabfree(void* mem, size_t bytes);
#define SLABALLOC(p)               \
    do {                           \
        p = slaballoc(sizeof(*p)); \
    } while (0)
#define SLABFREE(p)              \
    do {                         \
        slabfree(p, sizeof(*p)); \
        p = NULL;                \
    } while (0)

/* tls.c */
void tls_init(void);

/* timer.c */
int timer_setup(void);
u32 timer_get_cycles(void);
u32 timer_ms_to_cycles(u32 ms);
u32 timer_cycles_to_us(u32 cycles);

/* uart.c */
int uart_setup(void);
void uart_set_recv_data_handler(void (*handler)(const u8*, size_t));

/* dbgcon.c */
void dbgcon_setup(void);

#endif
