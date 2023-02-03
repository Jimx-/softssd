#include "xparameters.h"
#include "xil_exception.h"
#include "xdebug.h"
#include "xil_io.h"
#include "xscugic.h"
#include "xil_mmu.h"
#include "xaxidma.h"

#include <errno.h>
#include <inttypes.h>

#include <config.h>
#include <const.h>
#include <proto.h>
#include <thread.h>
#include <iov_iter.h>
#include <list.h>
#include <utils.h>
#include <tls.h>

#include "hostif.h"
#include "pcie_soft_intf.h"

#define MAX_PKT_LEN 0x2000

/* DMA device definitions */
#define CC_DMA_DEV_ID XPAR_AXIDMA_0_DEVICE_ID
#define RQ_DMA_DEV_ID XPAR_AXIDMA_1_DEVICE_ID

static XAxiDma cc_axi_dma;
static XAxiDma rq_axi_dma;

#define PCIE_SOFT_INTF_BASE_ADDR XPAR_PCIE_INTF_SOFT_0_BASEADDR

static struct pcie_soft_intf pcie_soft_intf;

#define CC_RX_INTR_ID XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR
#define CC_TX_INTR_ID XPAR_FABRIC_AXI_DMA_0_MM2S_INTROUT_INTR
#define RQ_RX_INTR_ID XPAR_FABRIC_PCIE_INTF_SOFT_0_RC_INTROUT_INTR
#define RQ_TX_INTR_ID XPAR_FABRIC_AXI_DMA_1_MM2S_INTROUT_INTR

#define RESET_TIMEOUT_COUNTER 10000

#define TLP_REQ_MEMORY_READ  0
#define TLP_REQ_MEMORY_WRITE 1

#define RQ_TLP_HEADER_SIZE 32

static const size_t max_dma_read_payload_size =
    4096; /* 1024 DWs, 11 bits dword count in TLP */
static size_t max_dma_write_payload_size;

struct completer_request {
    unsigned long addr;
    u8 at;
    u16 dword_count;
    u8 req_type;
    u16 requester_id;
    u8 tag;
    u8 target_func;
    u8 bar_id;
};

struct completer_completion {
    u8 low_addr;
    u8 at;
    u16 byte_count;
    u16 dword_count;
    u8 completion_status;
    u16 requester_id;
    u8 tag;
};

struct requester_request {
    unsigned long addr;
    u8 at;
    size_t byte_count;
    u8 req_type;
    u8 tag;
    u16 completer_id;
};

struct requester_completion {
    u16 low_addr;
    u8 err_code;
    u16 byte_count;
    u16 dword_count;
    u8 request_completed;
    u8 completion_status;
    u16 requester_id;
    u16 completor_id;
    u8 tag;
};

struct rq_queue_entry {
    struct worker_thread* thread;
    void* rx_buf;
    u16 rx_count;
};

/* A single data buffer presented to the requester interface for DMA read/write.
 */
struct rq_buffer_desc {
    u8 tlp_header[RQ_TLP_HEADER_SIZE];
    u8* data;
    size_t length;
    u8 tag;
    u8 __padding[15];
};

static u8* cc_tx_buffer;

#define TAG_MAX 64
static struct rq_queue_entry rq_queue[TAG_MAX];
static size_t rqes_in_used;
static mutex_t rq_queue_mutex;
static cond_t rq_queue_cond;

#define RQ_BUFFER_MAX 16
static DEFINE_TLS(struct rq_buffer_desc, rq_buffers[RQ_BUFFER_MAX])
    __attribute__((aligned(64)));

static u8* bdring_buffer;
static u8* cc_rx_bdring;
static u8* cc_tx_bdring;
static u8* rq_rx_bdring;
static u8* rq_tx_bdring;

static struct list_head cc_rx_buffers;
static struct list_head rq_rx_buffers;
static size_t cc_rx_buffer_count;
static size_t rq_rx_buffer_count;
static size_t max_rx_buffers;

static size_t rq_tx_free_slots;
static mutex_t rq_tx_slot_mutex;
static cond_t rq_tx_slot_cond;

static int setup_cc_rx(void);
static int setup_cc_tx(void);
static int setup_rq_rx(void);
static int setup_rq_tx(void);
static void cc_rx_intr_handler(void* callback);
static void rq_tx_intr_handler(void* callback);
static void rq_rx_intr_handler(void* callback);

int pcie_setup(void)
{
    int Status;
    XAxiDma_Config* Config;

    bdring_buffer = alloc_mem(1 << 20, 1 << 20, ZONE_PS_DDR_LOW);
    Xil_SetTlbAttributes((UINTPTR)bdring_buffer, 0x701); /* Uncachable */
    cc_rx_bdring = bdring_buffer + (0 << 18);            /* 256 kB */
    cc_tx_bdring = bdring_buffer + (1 << 18);
    rq_rx_bdring = bdring_buffer + (2 << 18);
    rq_tx_bdring = bdring_buffer + (3 << 18);

    INIT_LIST_HEAD(&cc_rx_buffers);
    INIT_LIST_HEAD(&rq_rx_buffers);

    if (mutex_init(&rq_queue_mutex, NULL) != 0) {
        panic("failed to initialize RQ queue mutex");
    }
    if (cond_init(&rq_queue_cond, NULL) != 0) {
        panic("failed to initialize RQ queue condvar");
    }

    psif_init(&pcie_soft_intf, (void*)PCIE_SOFT_INTF_BASE_ADDR);

    cc_tx_buffer = alloc_pages(1, ZONE_PS_DDR_LOW);

    Config = XAxiDma_LookupConfig(CC_DMA_DEV_ID);
    if (!Config) {
        xil_printf("No config found for %d\r\n", CC_DMA_DEV_ID);
        return XST_FAILURE;
    }

    /* Initialize CC DMA engine */
    Status = XAxiDma_CfgInitialize(&cc_axi_dma, Config);
    if (Status != XST_SUCCESS) {
        xil_printf("Initialization failed %d\r\n", Status);
        return XST_FAILURE;
    }

    Config = XAxiDma_LookupConfig(RQ_DMA_DEV_ID);
    if (!Config) {
        xil_printf("No config found for %d\r\n", RQ_DMA_DEV_ID);
        return XST_FAILURE;
    }

    /* Initialize RQ DMA engine */
    Status = XAxiDma_CfgInitialize(&rq_axi_dma, Config);
    if (Status != XST_SUCCESS) {
        xil_printf("Initialization failed %d\r\n", Status);
        return XST_FAILURE;
    }

    if (!XAxiDma_HasSg(&cc_axi_dma)) {
        xil_printf("Device configured as simple mode \r\n");
        return XST_FAILURE;
    }

    if (!XAxiDma_HasSg(&rq_axi_dma)) {
        xil_printf("Device configured as simple mode \r\n");
        return XST_FAILURE;
    }

    Status = setup_cc_rx();
    if (Status != XST_SUCCESS) {
        xil_printf("Failed to setup CC RX buffer descriptor ring\r\n");
        return XST_FAILURE;
    }

    Status = setup_cc_tx();
    if (Status != XST_SUCCESS) {
        xil_printf("Failed to setup CC TX buffer descriptor ring\r\n");
        return XST_FAILURE;
    }

    Status = setup_rq_rx();
    if (Status != XST_SUCCESS) {
        xil_printf("Failed to setup RQ RX buffer descriptor ring\r\n");
        return XST_FAILURE;
    }

    Status = setup_rq_tx();
    if (Status != XST_SUCCESS) {
        xil_printf("Failed to setup RQ TX buffer descriptor ring\r\n");
        return XST_FAILURE;
    }

    Status = intr_setup_irq(CC_RX_INTR_ID, 0x3,
                            (Xil_InterruptHandler)cc_rx_intr_handler,
                            XAxiDma_GetRxRing(&cc_axi_dma));
    if (Status != XST_SUCCESS) {
        xil_printf("Failed to setup CC RX IRQ\r\n");
        return XST_FAILURE;
    }

    Status = intr_setup_irq(RQ_TX_INTR_ID, 0x3,
                            (Xil_InterruptHandler)rq_tx_intr_handler,
                            XAxiDma_GetTxRing(&rq_axi_dma));
    if (Status != XST_SUCCESS) {
        xil_printf("Failed to setup RQ TX IRQ\r\n");
        return XST_FAILURE;
    }

    Status = intr_setup_irq(RQ_RX_INTR_ID, 0x1,
                            (Xil_InterruptHandler)rq_rx_intr_handler,
                            &pcie_soft_intf);
    if (Status != XST_SUCCESS) {
        xil_printf("Failed to setup RQ RX IRQ\r\n");
        return XST_FAILURE;
    }

    intr_enable_irq(CC_RX_INTR_ID);
    intr_enable_irq(RQ_TX_INTR_ID);
    intr_enable_irq(RQ_RX_INTR_ID);

    return XST_SUCCESS;
}

void pcie_stop(void)
{
    intr_disconnect_irq(CC_TX_INTR_ID);
    intr_disconnect_irq(CC_RX_INTR_ID);
    intr_disconnect_irq(RQ_TX_INTR_ID);
    intr_disconnect_irq(RQ_RX_INTR_ID);
}

int pcie_send_msi(u16 vector) { return psif_send_msi(&pcie_soft_intf, vector); }

/* This function and complete_rq() below should be called with interrupt
 * DISABLED because the interrupt handler touches the RQ table too. _force_
 * specifies whether to wait when no RQ table entry is available or just to
 * return an invalid tag. */
static int setup_rq(void* buf, size_t rx_count, int force)
{
    int i;

    Xil_AssertNonvoid(rqes_in_used <= TAG_MAX);

    if (force) {
        mutex_lock(&rq_queue_mutex);

        while (rqes_in_used == TAG_MAX) {
            cond_wait(&rq_queue_cond, &rq_queue_mutex);
        }

        mutex_unlock(&rq_queue_mutex);
    }

    for (i = 0; i < TAG_MAX; i++) {
        struct rq_queue_entry* rqe = &rq_queue[i];

        if (rqe->thread == NULL) {
            rqe->thread = worker_self();
            rqe->rx_buf = buf;
            rqe->rx_count = rx_count;

            Xil_DCacheFlushRange((UINTPTR)buf, rx_count);
            psif_rc_setup_buffer(&pcie_soft_intf, i, buf, rx_count);

            rqes_in_used++;
            return i;
        }
    }

    return -1;
}

static void complete_rq(int tag)
{
    struct rq_queue_entry* rqe = &rq_queue[tag];

    Xil_AssertVoid(rqe->thread);
    Xil_AssertVoid(rqes_in_used > 0);

    rqes_in_used--;
    rqe->thread = NULL;
    rqe->rx_buf = NULL;
    rqe->rx_count = 0;

    if (rqes_in_used == TAG_MAX - 1) cond_broadcast(&rq_queue_cond);
}

static void complete_all_rqs(const struct rq_buffer_desc* rqs, size_t nr_rqs)
{
    int i;
    struct worker_thread* self = worker_self();

    if (!nr_rqs) return;

    intr_disable();

    for (i = 0; i < nr_rqs; i++) {
        u8 tag = rqs[i].tag;
        struct rq_queue_entry* rqe = &rq_queue[tag];

        Xil_AssertVoid(rqe->thread == self);

        complete_rq(tag);
    }

    intr_enable();
}

static int send_packet(XAxiDma* axi_dma_inst, const u8* buffer, size_t size)
{
    XAxiDma_BdRing* tx_ring;
    XAxiDma_Bd* bd_ptr;
    int status;

    tx_ring = XAxiDma_GetTxRing(axi_dma_inst);

    status = XAxiDma_BdRingAlloc(tx_ring, 1, &bd_ptr);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    status = XAxiDma_BdSetBufAddr(bd_ptr, (UINTPTR)buffer);
    if (status != XST_SUCCESS) {
        xil_printf("Tx set buffer addr %p on BD %p failed %d\r\n", buffer,
                   bd_ptr, status);

        return XST_FAILURE;
    }

    status = XAxiDma_BdSetLength(bd_ptr, size, tx_ring->MaxTransferLen);
    if (status != XST_SUCCESS) {
        xil_printf("Tx set length %d on BD %p failed %d\r\n", MAX_PKT_LEN,
                   bd_ptr, status);

        return XST_FAILURE;
    }

    XAxiDma_BdSetCtrl(bd_ptr,
                      XAXIDMA_BD_CTRL_TXEOF_MASK | XAXIDMA_BD_CTRL_TXSOF_MASK);

    XAxiDma_BdSetId(bd_ptr, (UINTPTR)buffer);

    status = XAxiDma_BdRingToHw(tx_ring, 1, bd_ptr);
    if (status != XST_SUCCESS) {
        xil_printf("to hw failed %d\r\n", status);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static int send_cc_packet_sync(const u8* pkt_buf, size_t pkt_size)
{
    XAxiDma_BdRing* tx_ring;
    XAxiDma_Bd* bd_ptr;
    int count;
    int status;

    Xil_DCacheFlushRange((UINTPTR)pkt_buf, pkt_size);

    status = send_packet(&cc_axi_dma, pkt_buf, pkt_size);
    if (status != XST_SUCCESS) return EIO;

    tx_ring = XAxiDma_GetTxRing(&cc_axi_dma);
    while ((count = XAxiDma_BdRingFromHw(tx_ring, XAXIDMA_ALL_BDS, &bd_ptr)) ==
           0)
        /* NOP */;

    status = XAxiDma_BdRingFree(tx_ring, count, bd_ptr);
    if (status != XST_SUCCESS) return EIO;

    return 0;
}

static int send_rq_packets_sync(const struct rq_buffer_desc* bufs, size_t count)
{
    struct worker_thread* self = worker_self();
    XAxiDma_BdRing* tx_ring;
    XAxiDma_Bd *bd_ptr, *first_bd;
    int i;
    size_t bd_count;
    int status, r;

    bd_count = 0;
    for (i = 0; i < count; i++) {
        bd_count += 1;
        /* Need an extra BD for the data buffer. */
        if (bufs[i].data) bd_count += 1;
    }

    /* Secure enough buffer slots for submission. */
    mutex_lock(&rq_tx_slot_mutex);

    while (bd_count > rq_tx_free_slots) {
        cond_wait(&rq_tx_slot_cond, &rq_tx_slot_mutex);
    }

    Xil_AssertNonvoid(rq_tx_free_slots >= bd_count);
    rq_tx_free_slots -= bd_count;

    mutex_unlock(&rq_tx_slot_mutex);

    tx_ring = XAxiDma_GetTxRing(&rq_axi_dma);

    status = XAxiDma_BdRingAlloc(tx_ring, bd_count, &bd_ptr);
    if (status != XST_SUCCESS) {
        xil_printf("Bd ring alloc %d failed\r\n", bd_count);

        rq_tx_free_slots += bd_count;
        cond_broadcast(&rq_tx_slot_cond);

        return ENOSPC;
    }

    first_bd = bd_ptr;

    r = EINVAL;
    for (i = 0; i < count; i++) {
        const struct rq_buffer_desc* buf = &bufs[i];

        Xil_DCacheFlushRange((UINTPTR)buf->tlp_header, RQ_TLP_HEADER_SIZE);

        /* Set up TLP header. */
        status = XAxiDma_BdSetBufAddr(bd_ptr, (UINTPTR)buf->tlp_header);
        if (status != XST_SUCCESS) {
            xil_printf("Tx set header addr %p on BD %p failed %d\r\n",
                       buf->tlp_header, bd_ptr, status);

            goto cleanup;
        }

        status = XAxiDma_BdSetLength(bd_ptr, RQ_TLP_HEADER_SIZE,
                                     tx_ring->MaxTransferLen);
        if (status != XST_SUCCESS) {
            xil_printf("Tx set length %d on BD %p failed %d\r\n",
                       RQ_TLP_HEADER_SIZE, bd_ptr, status);

            goto cleanup;
        }

        XAxiDma_BdSetCtrl(bd_ptr, (buf->data ? 0 : XAXIDMA_BD_CTRL_TXEOF_MASK) |
                                      XAXIDMA_BD_CTRL_TXSOF_MASK);
        XAxiDma_BdSetId(bd_ptr, (UINTPTR)self);

        if (buf->data) {
            /* Set up data part if necessary. */
            Xil_DCacheFlushRange((UINTPTR)buf->data, buf->length);

            bd_ptr = (XAxiDma_Bd*)XAxiDma_BdRingNext(tx_ring, bd_ptr);

            status = XAxiDma_BdSetBufAddr(bd_ptr, (UINTPTR)buf->data);
            if (status != XST_SUCCESS) {
                xil_printf(
                    "Tx set buffer addr %p length %d on BD %p failed %d\r\n",
                    buf->data, buf->length, bd_ptr, status);

                goto cleanup;
            }

            status = XAxiDma_BdSetLength(bd_ptr, buf->length,
                                         tx_ring->MaxTransferLen);
            if (status != XST_SUCCESS) {
                xil_printf("Tx set length %d on BD %p failed %d\r\n",
                           buf->length, bd_ptr, status);

                goto cleanup;
            }

            XAxiDma_BdSetCtrl(bd_ptr, XAXIDMA_BD_CTRL_TXEOF_MASK);
            XAxiDma_BdSetId(bd_ptr, (UINTPTR)self);
        }

        bd_ptr = (XAxiDma_Bd*)XAxiDma_BdRingNext(tx_ring, bd_ptr);
    }

    r = EIO;
    status = XAxiDma_BdRingToHw(tx_ring, bd_count, first_bd);
    if (status != XST_SUCCESS) {
        xil_printf("to hw failed %d\r\n", status);
        goto cleanup;
    }

    self->rq_error = FALSE;
    self->pending_rqs = bd_count;

    r = worker_wait_timeout(WT_BLOCKED_ON_PCIE_RQ, 3000);

    if (self->rq_error) return EIO;

    return r;

cleanup:
    XAxiDma_BdRingFree(tx_ring, bd_count, first_bd);

    rq_tx_free_slots += bd_count;
    cond_broadcast(&rq_tx_slot_cond);

    return r;
}

static size_t pack_requester_request(const struct requester_request* req,
                                     const u8* buffer, size_t count,
                                     struct rq_buffer_desc* rq_buf)
{
    u32* dws = (u32*)rq_buf->tlp_header;
    size_t out_size;

    memset(rq_buf, 0, sizeof(*rq_buf));

    dws[0] = (req->addr & 0xfffffffc) | (req->at & 0x3);
    dws[1] = (req->addr >> 32) & 0xffffffff;
    dws[2] = (u32)req->byte_count;
    dws[3] = req->tag;
    dws[3] |= req->completer_id << 8;
    dws[3] |= (req->req_type & 0xf) << 24;

    out_size = 32;

    if (buffer) {
        if (count <= 16) {
            /* Piggybacking */
            *(u64*)&rq_buf->tlp_header[16] = *(u64*)buffer;
            *(u64*)&rq_buf->tlp_header[24] = *(u64*)&buffer[8];
        } else {
            rq_buf->data = (u8*)buffer;
            rq_buf->length = count;
            out_size += count;
        }
    }

    rq_buf->tag = req->tag;

    return out_size;
}

static size_t unpack_completer_request(const u8* buffer,
                                       struct completer_request* cr)
{
    const volatile u32* dws = (const volatile u32*)buffer;
    u8 bar_aperture;
    unsigned long mask;

    cr->bar_id = (dws[3] >> 16) & 0x7;
    bar_aperture = (dws[3] >> 19) & 0x3f;
    cr->addr = (((unsigned long)dws[1] << 32UL) | dws[0]) & ~0x3UL;
    mask = (1ULL << bar_aperture) - 1;
    cr->addr = cr->addr & mask;
    cr->at = dws[0] & 0x3;
    cr->requester_id = (dws[2] >> 16) & 0xffff;
    cr->dword_count = dws[2] & 0x7ff;
    cr->req_type = (dws[2] >> 11) & 0xf;
    cr->tag = dws[3] & 0x3f; /* 6-bit tags */
    cr->target_func = (dws[3] >> 8) & 0xff;

    return 16;
}

__attribute__((unused)) static void
dump_completer_request(const struct completer_request* cr)
{
    xil_printf("CompleterRequest {\n");
    xil_printf("  Addr: 0x%08x%08x\n", (u32)(cr->addr >> 32),
               (u32)(cr->addr & 0xffffffff));
    xil_printf("  Dword count: %d, request type: %d, requester ID: 0x%04x\n",
               cr->dword_count, cr->req_type, cr->requester_id);
    xil_printf("  Tag: %d, target function: %d, BAR%d\n", cr->tag,
               cr->target_func, cr->bar_id);
    xil_printf("}\n");
}

__attribute__((unused)) static size_t
unpack_requester_completion(const u8* buffer,
                            struct requester_completion* out_desc)
{
    const u32* dws = (const u32*)buffer;

    out_desc->low_addr = dws[0] & 0xfff;
    out_desc->err_code = (dws[0] >> 12) & 0xf;
    out_desc->byte_count = (dws[0] >> 16) & 0x1fff;
    out_desc->request_completed = (dws[0] >> 30) & 0x1;
    out_desc->dword_count = dws[1] & 0x7ff;
    out_desc->completion_status = (dws[1] >> 11) & 0x7;
    out_desc->requester_id = (dws[1] >> 16) & 0xffff;
    out_desc->tag = dws[2] & 0xff;
    out_desc->completor_id = (dws[2] >> 8) & 0xffff;

    return 12;
}

__attribute__((unused)) static void
dump_requester_completion(struct requester_completion* rc)
{
    xil_printf("RequesterCompletion {\n");
    xil_printf("  Low addr: 0x%06x, error code: %d, byte count: %d %s\n",
               rc->low_addr, rc->err_code, rc->byte_count,
               rc->request_completed ? "(Completed)" : "");
    xil_printf(
        "  Dword count: %d, completion status: %d, requester ID: 0x%04x\n",
        rc->dword_count, rc->completion_status, rc->requester_id);
    xil_printf("  Tag: %d, completer ID: 0x%04x\n", rc->tag, rc->completor_id);
    xil_printf("}\n");
}

static int pcie_do_dma_iter(unsigned long addr, int do_write,
                            struct iov_iter* iter, size_t count)
{
    struct worker_thread* self = worker_self();
    struct requester_request req = {.at = 0, .completer_id = 0};
    size_t max_payload_size;
    struct rq_buffer_desc* rq_bufs = get_local_var(rq_buffers);
    size_t nbufs = 0;
    int tag = 0;
    int r;

    max_payload_size =
        do_write ? max_dma_write_payload_size : max_dma_read_payload_size;

    while (count > 0) {
        struct rq_buffer_desc* rq_buf;
        void* buf = NULL;
        size_t chunk = count;

        if (chunk > max_payload_size) chunk = max_payload_size;

        iov_iter_get_bufaddr(iter, &buf, &chunk);

        if (!do_write) {
            intr_disable();
            tag = setup_rq(buf, chunk, nbufs == 0);
            intr_enable();
        }

        if (nbufs >= RQ_BUFFER_MAX || (!do_write && tag < 0)) {
            /* Flush RQ buffers when full or we cannot allocate a tag from the
             * RQ table. */

            /* Disable interrupt from here so that we will not be woken up by
             * the interrupt handler before we start to wait. */
            intr_disable();

            if (!do_write) {
                self->pending_rcs = nbufs;
                self->rc_error = FALSE;
            }

            r = send_rq_packets_sync(rq_bufs, nbufs);
            if (r) {
                intr_enable();
                if (!do_write) complete_all_rqs(rq_bufs, nbufs);
                return r;
            }

            if (!do_write) {
                r = 0;

                if (self->pending_rcs) {
                    r = worker_wait_timeout(WT_BLOCKED_ON_PCIE_CPL, 3000);
                }

                intr_enable();
                complete_all_rqs(rq_bufs, nbufs);

                if (self->rc_error) r = r ?: EIO;
                if (r) return r;
            } else {
                intr_enable();
            }

            nbufs = 0;

            if (!do_write && tag < 0) {
                /* Try to get another tag. */
                intr_disable();
                tag = setup_rq(buf, chunk, TRUE);
                intr_enable();
                Xil_AssertNonvoid(tag >= 0);
            }
        }

        rq_buf = &rq_bufs[nbufs++];

        req.tag = tag;
        req.req_type = do_write ? TLP_REQ_MEMORY_WRITE : TLP_REQ_MEMORY_READ;
        req.addr = addr;
        req.byte_count = chunk;

        pack_requester_request(&req, do_write ? buf : NULL, chunk, rq_buf);

        count -= chunk;
        addr += chunk;
    }

    if (nbufs > 0) {
        intr_disable();

        if (!do_write) {
            self->pending_rcs = nbufs;
            self->rc_error = FALSE;
        }

        r = send_rq_packets_sync(rq_bufs, nbufs);
        if (r) {
            intr_enable();
            if (!do_write) complete_all_rqs(rq_bufs, nbufs);
            return r;
        }

        if (!do_write) {
            r = 0;

            if (self->pending_rcs) {
                r = worker_wait_timeout(WT_BLOCKED_ON_PCIE_CPL, 3000);
            }
            intr_enable();
            complete_all_rqs(rq_bufs, nbufs);

            if (self->rc_error) r = r ?: EIO;
            if (r) return r;
        } else {
            intr_enable();
        }
    }

    return 0;
}

int pcie_dma_read_iter(unsigned long addr, struct iov_iter* iter, size_t count)
{
    return pcie_do_dma_iter(addr, 0, iter, count);
}

int pcie_dma_write_iter(unsigned long addr, struct iov_iter* iter, size_t count)
{
    return pcie_do_dma_iter(addr, 1, iter, count);
}

int pcie_dma_read(unsigned long addr, u8* buffer, size_t count)
{
    struct iovec iov = {
        .iov_base = buffer,
        .iov_len = count,
    };
    struct iov_iter iter;

    iov_iter_init(&iter, &iov, 1, count);

    return pcie_dma_read_iter(addr, &iter, count);
}

int pcie_dma_write(unsigned long addr, const u8* buffer, size_t count)
{
    struct iovec iov = {
        .iov_base = (void*)buffer,
        .iov_len = count,
    };
    struct iov_iter iter;

    iov_iter_init(&iter, &iov, 1, count);

    return pcie_dma_write_iter(addr, &iter, count);
}

static size_t pack_completer_completion(const struct completer_completion* cc,
                                        const u8* buffer, size_t count,
                                        u8* out_buffer)
{
    u32* dws = (u32*)out_buffer;

    dws[0] = cc->low_addr & 0x7f;
    dws[0] |= (cc->at & 0x3) << 8;
    dws[0] |= (cc->byte_count & 0x1fff) << 16;
    dws[1] = cc->dword_count & 0x7ff;
    dws[1] |= (cc->completion_status & 0x7) << 11;
    dws[1] |= cc->requester_id << 16;
    dws[2] = cc->tag;

    memcpy(&out_buffer[12], buffer, count);

    return 12 + count;
}

int pcie_send_completion(unsigned long addr, u16 requester_id, u8 tag,
                         const u8* buffer, size_t count)
{
    struct completer_completion cc = {
        .low_addr = (u8)(addr & 0xff),
        .at = 0,
        .byte_count = count,
        .dword_count = count >> 2,
        .completion_status = 0,
        .requester_id = requester_id,
        .tag = tag,
    };
    u8* tlp_buf = cc_tx_buffer;
    size_t tlp_size;
    int r;

    tlp_size = pack_completer_completion(&cc, buffer, count, tlp_buf);

    r = send_cc_packet_sync(tlp_buf, tlp_size);
    if (r) return r;

    return 0;
}

static int fill_rx_buffers(XAxiDma_BdRing* rx_ring, int is_cc)
{
    struct list_head* list;
    XAxiDma_Bd* bd_ptr;
    XAxiDma_Bd* bd_cur_ptr;
    u8* buffer;
    size_t* buffer_count;
    size_t count;
    int status;

    list = is_cc ? &cc_rx_buffers : &rq_rx_buffers;
    buffer_count = is_cc ? &cc_rx_buffer_count : &rq_rx_buffer_count;
    count = *buffer_count;

    status = XAxiDma_BdRingAlloc(rx_ring, count, &bd_ptr);
    if (status != XST_SUCCESS) {
        xil_printf("Rx bd alloc failed with %d\r\n", status);
        return XST_FAILURE;
    }

    bd_cur_ptr = bd_ptr;

    while (*buffer_count > 0) {
        buffer = (u8*)list->next;
        list_del((struct list_head*)buffer);

        Xil_DCacheFlushRange((UINTPTR)buffer, MAX_PKT_LEN);

        status = XAxiDma_BdSetBufAddr(bd_cur_ptr, (UINTPTR)buffer);
        if (status != XST_SUCCESS) {
            xil_printf("Rx set buffer addr %p on BD %p failed %d\r\n", buffer,
                       bd_cur_ptr, status);

            return XST_FAILURE;
        }

        status = XAxiDma_BdSetLength(bd_cur_ptr, MAX_PKT_LEN,
                                     rx_ring->MaxTransferLen);
        if (status != XST_SUCCESS) {
            xil_printf("Rx set length %d on BD %p failed %d\r\n", MAX_PKT_LEN,
                       bd_cur_ptr, status);

            return XST_FAILURE;
        }

        XAxiDma_BdSetCtrl(bd_cur_ptr, 0);
        XAxiDma_BdSetId(bd_cur_ptr, buffer);

        bd_cur_ptr = (XAxiDma_Bd*)XAxiDma_BdRingNext(rx_ring, bd_cur_ptr);
        (*buffer_count)--;
    }

    status = XAxiDma_BdRingToHw(rx_ring, count, bd_ptr);
    if (status != XST_SUCCESS) {
        xil_printf("Rx ToHw failed with %d\r\n", status);
        return XST_FAILURE;
    }

    return 0;
}

static int rx_setup(XAxiDma* axi_dma_inst, int is_cc)
{
    XAxiDma_BdRing* rx_ring;
    u8* bdr_buffer;
    XAxiDma_Bd bd_template;
    void* rx_buffer;
    int bd_count, free_bd_count;
    struct list_head* list;
    size_t* buffer_count;
    int i, status;

    rx_ring = XAxiDma_GetRxRing(axi_dma_inst);
    XAxiDma_BdRingIntDisable(rx_ring, XAXIDMA_IRQ_ALL_MASK);

    bdr_buffer = is_cc ? cc_rx_bdring : rq_rx_bdring;
    list = is_cc ? &cc_rx_buffers : &rq_rx_buffers;
    buffer_count = is_cc ? &cc_rx_buffer_count : &rq_rx_buffer_count;

    bd_count = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT,
                                     0x1000); /* 64 slots */

    status =
        XAxiDma_BdRingCreate(rx_ring, (UINTPTR)bdr_buffer, (UINTPTR)bdr_buffer,
                             XAXIDMA_BD_MINIMUM_ALIGNMENT, bd_count);
    if (status != XST_SUCCESS) {
        xil_printf("Rx bd create failed with %d\r\n", status);
        return XST_FAILURE;
    }

    XAxiDma_BdClear(&bd_template);
    status = XAxiDma_BdRingClone(rx_ring, &bd_template);
    if (status != XST_SUCCESS) {
        xil_printf("Rx bd clone failed with %d\r\n", status);
        return XST_FAILURE;
    }

    free_bd_count = XAxiDma_BdRingGetFreeCnt(rx_ring);

    rx_buffer =
        alloc_mem(free_bd_count * MAX_PKT_LEN, 1 << 20, ZONE_PS_DDR_LOW);

    for (i = 0; i < free_bd_count; i++) {
        list_add_tail((struct list_head*)rx_buffer, list);
        rx_buffer += MAX_PKT_LEN;
    }
    *buffer_count = free_bd_count;

    if (free_bd_count > max_rx_buffers) max_rx_buffers = free_bd_count;

    status = fill_rx_buffers(rx_ring, is_cc);
    if (status != XST_SUCCESS) {
        xil_printf("Rx fill buffers failed with %d\r\n", status);
        return XST_FAILURE;
    }

    status = XAxiDma_BdRingSetCoalesce(rx_ring, 1, 0);
    if (status != XST_SUCCESS) {
        xil_printf("Rx set coalesce failed with %d\r\n", status);
        return XST_FAILURE;
    }

    XAxiDma_BdRingIntEnable(rx_ring, XAXIDMA_IRQ_ALL_MASK);

    status = XAxiDma_BdRingStart(rx_ring);
    if (status != XST_SUCCESS) {
        xil_printf("Rx start BD ring failed with %d\r\n", status);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static int setup_cc_rx(void) { return rx_setup(&cc_axi_dma, 1); }

static int setup_rq_rx(void) { return XST_SUCCESS; }

static int tx_setup(XAxiDma* axi_dma_inst, int is_cc)
{
    XAxiDma_BdRing* tx_ring;
    XAxiDma_Bd bd_template;
    u8* bdr_buffer;
    int bd_count;
    int status;

    bdr_buffer = is_cc ? cc_tx_bdring : rq_tx_bdring;

    tx_ring = XAxiDma_GetTxRing(axi_dma_inst);

    XAxiDma_BdRingIntDisable(tx_ring, XAXIDMA_IRQ_ALL_MASK);

    bd_count = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT,
                                     (1 << 18)); /* 4096 slots */

    if (!is_cc) rq_tx_free_slots = bd_count;

    status =
        XAxiDma_BdRingCreate(tx_ring, (UINTPTR)bdr_buffer, (UINTPTR)bdr_buffer,
                             XAXIDMA_BD_MINIMUM_ALIGNMENT, bd_count);
    if (status != XST_SUCCESS) {
        xil_printf("Tx bd create failed with %d\r\n", status);
        return XST_FAILURE;
    }

    XAxiDma_BdClear(&bd_template);
    status = XAxiDma_BdRingClone(tx_ring, &bd_template);
    if (status != XST_SUCCESS) {
        xil_printf("Tx bd clone failed with %d\r\n", status);
        return XST_FAILURE;
    }

    if (!is_cc) XAxiDma_BdRingIntEnable(tx_ring, XAXIDMA_IRQ_ALL_MASK);

    status = XAxiDma_BdRingStart(tx_ring);
    if (status != XST_SUCCESS) {
        xil_printf("Tx start BD ring failed with %d\r\n", status);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static int setup_cc_tx(void) { return tx_setup(&cc_axi_dma, 1); }

static int setup_rq_tx(void)
{
    XAxiDma_BdRing* tx_ring;
    u32 max_transfer_len;

    if (mutex_init(&rq_tx_slot_mutex, NULL) != 0) {
        panic("failed to initialize RQ TX slot mutex");
    }
    if (cond_init(&rq_tx_slot_cond, NULL) != 0) {
        panic("failed to initialize RQ TX slot condvar");
    }

    tx_ring = XAxiDma_GetTxRing(&rq_axi_dma);
    max_transfer_len = tx_ring->MaxTransferLen;

    max_transfer_len = 1 << (31 - clz(max_transfer_len));
    max_dma_write_payload_size = max_transfer_len;

    return tx_setup(&rq_axi_dma, 0);
}

static void handle_cc_rx(const u8* buffer)
{
    struct completer_request cr;
    int data_offset;

    data_offset = unpack_completer_request(buffer, &cr);

    /* dump_completer_request(&cr); */

    if (cr.req_type == TLP_REQ_MEMORY_READ) {
        nvme_handle_reg_read(cr.addr, cr.requester_id, cr.tag,
                             cr.dword_count << 2);
    } else if (cr.req_type == TLP_REQ_MEMORY_WRITE) {
        nvme_handle_reg_write(cr.addr, &buffer[data_offset],
                              cr.dword_count << 2);
    }
}

static void handle_rq_rx(int tag, int error)
{
    struct rq_queue_entry* rqe;
    size_t nbufs;

    rqe = &rq_queue[tag];
    if (rqe->thread == NULL) {
        xil_printf("Spurious RQ RX interrupt, tag %d\n", tag);
        return;
    }

    Xil_DCacheInvalidateRange((UINTPTR)rqe->rx_buf, rqe->rx_count);

    rqe->thread->rc_error = error;

    nbufs = rqe->thread->pending_rcs;
    if (nbufs == 0) {
        xil_printf("Worker blocked with nbufs == 0\n");
    }
    nbufs--;
    rqe->thread->pending_rcs = nbufs;

    if (!nbufs) {
        worker_wake(rqe->thread, WT_BLOCKED_ON_PCIE_CPL);
        sevl();
    }
}

static void rx_callback(XAxiDma_BdRing* rx_ring, int is_cc)
{
    int bd_count;
    XAxiDma_Bd* bd_ptr;
    XAxiDma_Bd* bd_cur_ptr;
    u32 bd_status;
    void* buffer;
    struct list_head* list;
    size_t* buffer_count;
    int i, status;

    list = is_cc ? &cc_rx_buffers : &rq_rx_buffers;
    buffer_count = is_cc ? &cc_rx_buffer_count : &rq_rx_buffer_count;

    bd_count = XAxiDma_BdRingFromHw(rx_ring, XAXIDMA_ALL_BDS, &bd_ptr);

    bd_cur_ptr = bd_ptr;

    for (i = 0; i < bd_count; i++) {
        bd_status = XAxiDma_BdGetSts(bd_cur_ptr);
        buffer = (void*)(UINTPTR)XAxiDma_BdGetBufAddr(bd_cur_ptr);

        if ((bd_status & XAXIDMA_BD_STS_ALL_ERR_MASK) ||
            (!(bd_status & XAXIDMA_BD_STS_COMPLETE_MASK))) {
            xil_printf("RX callback error\r\n");
        } else {
            Xil_DCacheInvalidateRange((UINTPTR)buffer, MAX_PKT_LEN);

            if (is_cc) handle_cc_rx(buffer);
        }

        list_add_tail((struct list_head*)buffer, list);
        (*buffer_count)++;

        bd_cur_ptr = (XAxiDma_Bd*)XAxiDma_BdRingNext(rx_ring, bd_cur_ptr);
    }

    status = XAxiDma_BdRingFree(rx_ring, bd_count, bd_ptr);
    if (status != XST_SUCCESS) {
        xil_printf("RX bd ring free failed\r\n");
    }

    if (*buffer_count > (max_rx_buffers >> 1)) {
        fill_rx_buffers(rx_ring, is_cc);
    }
}

static int rq_tx_callback(XAxiDma_BdRing* tx_ring)
{
    int bd_count;
    XAxiDma_Bd *bd_ptr, *bd_cur_ptr;
    struct worker_thread* worker;
    u32 bd_status;
    int i, status;

    bd_count = XAxiDma_BdRingFromHw(tx_ring, XAXIDMA_ALL_BDS, &bd_ptr);

    bd_cur_ptr = bd_ptr;

    for (i = 0; i < bd_count; i++) {
        bd_status = XAxiDma_BdGetSts(bd_cur_ptr);
        worker = (struct worker_thread*)(UINTPTR)XAxiDma_BdGetId(bd_cur_ptr);

        if ((bd_status & XAXIDMA_BD_STS_ALL_ERR_MASK) ||
            (!(bd_status & XAXIDMA_BD_STS_COMPLETE_MASK))) {
            xil_printf("RQ TX error worker %p\n", worker);
            worker->rq_error = TRUE;
        }

        if (worker->pending_rqs <= 0)
            xil_printf("Spurious RQ TX completion\n");
        else {
            if (--worker->pending_rqs == 0) {
                worker_wake(worker, WT_BLOCKED_ON_PCIE_RQ);
                sevl();
            }
        }

        bd_cur_ptr = (XAxiDma_Bd*)XAxiDma_BdRingNext(tx_ring, bd_cur_ptr);
    }

    if (bd_count > 0) {
        status = XAxiDma_BdRingFree(tx_ring, bd_count, bd_ptr);
        if (status != XST_SUCCESS) {
            xil_printf("TX bd ring free failed\r\n");
        }
    }

    return bd_count;
}

static void cc_rx_intr_handler(void* callback)
{
    u32 IrqStatus;
    int TimeOut;
    XAxiDma_BdRing* rx_ring = (XAxiDma_BdRing*)callback;

    /* xil_printf("CcRxIntrHandler\r\n"); */

    IrqStatus = XAxiDma_BdRingGetIrq(rx_ring);
    XAxiDma_BdRingAckIrq(rx_ring, IrqStatus);

    if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
        return;
    }

    if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {
        XAxiDma_BdRingDumpRegs(rx_ring);

        XAxiDma_Reset(&cc_axi_dma);

        TimeOut = RESET_TIMEOUT_COUNTER;
        while (TimeOut) {
            if (XAxiDma_ResetIsDone(&cc_axi_dma)) {
                break;
            }

            TimeOut -= 1;
        }

        return;
    }

    if (IrqStatus & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK)) {
        rx_callback(rx_ring, 1);
    }
}

static void rq_tx_intr_handler(void* callback)
{

    u32 IrqStatus;
    int TimeOut;
    XAxiDma_BdRing* tx_ring = (XAxiDma_BdRing*)callback;
    int found;

    IrqStatus = XAxiDma_BdRingGetIrq(tx_ring);
    XAxiDma_BdRingAckIrq(tx_ring, IrqStatus);

    if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
        return;
    }

    if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {
        XAxiDma_BdRingDumpRegs(tx_ring);

        XAxiDma_Reset(&rq_axi_dma);
        TimeOut = RESET_TIMEOUT_COUNTER;
        while (TimeOut) {
            if (XAxiDma_ResetIsDone(&rq_axi_dma)) {
                break;
            }

            TimeOut -= 1;
        }

        return;
    }

    if (IrqStatus & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK)) {
        found = rq_tx_callback(tx_ring);

        if (found) {
            rq_tx_free_slots += found;
            cond_broadcast(&rq_tx_slot_cond);
        }
    }
}

static void rq_rx_intr_handler(void* callback)
{
    struct pcie_soft_intf* psif = (struct pcie_soft_intf*)callback;
    u64 ioc_bitmap, err_bitmap;
    int i;

    ioc_bitmap = psif_get_ioc_bitmap(psif);
    err_bitmap = psif_get_err_bitmap(psif);

    psif_rc_ack_intr(psif, ioc_bitmap | err_bitmap);

    for (i = 0; i < 64; i++) {
        if ((ioc_bitmap & (1ULL << i)) || (err_bitmap & (1ULL << i)))
            handle_rq_rx(i, !!(err_bitmap & (1ULL << i)));
    }
}
