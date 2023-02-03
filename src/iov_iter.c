#include <iov_iter.h>
#include <string.h>
#include <proto.h>
#include <utils.h>

void iov_iter_init(struct iov_iter* iter, const struct iovec* iov,
                   size_t nr_segs, size_t count)
{
    iter->iov = iov;
    iter->count = count;
    iter->nr_segs = nr_segs;
    iter->iov_offset = 0;
}

size_t iov_iter_copy_from(struct iov_iter* iter, void* buf, size_t bytes)
{
    size_t copied = 0;
    size_t chunk;

    while (copied < bytes && iter->nr_segs > 0) {
        chunk = min(iter->iov->iov_len - iter->iov_offset, bytes - copied);

        if (chunk > 0) {
            memcpy(buf, iter->iov->iov_base + iter->iov_offset, chunk);
        }

        copied += chunk;
        iter->iov_offset += chunk;
        buf += chunk;

        if (iter->iov_offset == iter->iov->iov_len) {
            iter->iov++;
            iter->nr_segs--;
            iter->iov_offset = 0;
        }
    }

    return copied;
}

size_t iov_iter_copy_to(struct iov_iter* iter, const void* buf, size_t bytes)
{
    size_t copied = 0;
    size_t chunk;

    while (copied < bytes && iter->nr_segs > 0) {
        chunk = min(iter->iov->iov_len - iter->iov_offset, bytes - copied);

        if (chunk > 0) {
            memcpy(iter->iov->iov_base + iter->iov_offset, buf, chunk);
        }

        copied += chunk;
        iter->iov_offset += chunk;
        buf += chunk;

        if (iter->iov_offset == iter->iov->iov_len) {
            iter->iov++;
            iter->nr_segs--;
            iter->iov_offset = 0;
        }
    }

    return copied;
}

int iov_iter_get_bufaddr(struct iov_iter* iter, void** buf, size_t* bytes)
{
    if (!iter->nr_segs) return FALSE;

    *buf = iter->iov->iov_base + iter->iov_offset;
    *bytes = min(*bytes, iter->iov->iov_len - iter->iov_offset);
    iter->iov_offset += *bytes;

    if (iter->iov_offset == iter->iov->iov_len) {
        iter->iov++;
        iter->nr_segs--;
        iter->iov_offset = 0;
    }

    return TRUE;
}
