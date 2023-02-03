#ifndef _HOSTIF_HOSTIF_H_
#define _HOSTIF_HOSTIF_H_

/* nvme.c */
void nvme_handle_reg_read(unsigned long addr, u16 requester_id, u8 tag,
                          size_t len);
void nvme_handle_reg_write(unsigned long addr, const u8* buf, size_t len);

/* nvme_identify.c */
void nvme_identify_namespace(u32 nsid, u8* data);
void nvme_identify_controller(u8* data);
void nvme_identify_ns_active_list(u8* data);
void nvme_identify_cs_controller(u8 csi, u8* data);

#endif
