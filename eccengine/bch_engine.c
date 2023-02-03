#include "bch_engine.h"

#include <stdlib.h>
#include <string.h>
#include <errno.h>

void bch_engine_cleanup(struct bch_engine* engine)
{
    bch_free(engine->bch);
    free(engine->err_loc);
    free(engine->ecc_mask);
}

int bch_engine_init(struct bch_engine* engine, unsigned int ecc_size,
                    unsigned int ecc_bytes)
{
    unsigned int m, t, i;
    unsigned char* erased_page;
    int r;

    memset(engine, 0, sizeof(*engine));

    m = fls(1 + (8 * ecc_size));
    t = (ecc_bytes * 8) / m;

    engine->step_size = ecc_size;
    engine->code_size = ecc_bytes;

    engine->bch = bch_init(m, t, 0, 0);
    if (!engine->bch) return EINVAL;

    engine->ecc_mask = malloc(ecc_bytes);
    engine->err_loc = calloc(t, sizeof(*engine->err_loc));
    if (!engine->ecc_mask || !engine->err_loc) {
        r = ENOMEM;
        goto cleanup;
    }

    memset(engine->ecc_mask, 0, ecc_bytes);

    erased_page = malloc(ecc_size);
    if (!erased_page) {
        r = ENOMEM;
        goto cleanup;
    }

    memset(erased_page, 0xff, ecc_size);
    bch_encode(engine->bch, erased_page, ecc_size, engine->ecc_mask);
    free(erased_page);

    for (i = 0; i < ecc_bytes; i++)
        engine->ecc_mask[i] ^= 0xff;

    return 0;

cleanup:
    return r;
}

int bch_engine_calculate(struct bch_engine* engine, const unsigned char* buf,
                         unsigned char* code)
{
    unsigned int i;

    memset(code, 0, engine->code_size);
    bch_encode(engine->bch, buf, engine->step_size, code);

    for (i = 0; i < engine->code_size; i++)
        code[i] ^= engine->ecc_mask[i];

    return 0;
}

int bch_engine_correct(struct bch_engine* engine, unsigned char* buf,
                       const unsigned char* read_ecc,
                       const unsigned char* calc_ecc)
{
    unsigned int step_size = engine->step_size;
    unsigned int* errloc = engine->err_loc;
    int i, count;

    count = bch_decode(engine->bch, NULL, step_size, read_ecc, calc_ecc, NULL,
                       errloc);
    if (count > 0) {
        for (i = 0; i < count; i++) {
            if (errloc[i] < (step_size * 8))
                buf[errloc[i] >> 3] ^= (1 << (errloc[i] & 7));
        }
    } else if (count < 0) {
        count = -EBADMSG;
    }

    return count;
}
