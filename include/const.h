#ifndef _CONST_H_
#define _CONST_H_

/* Memory zones */
#define MEMZONE_PS_DDR_HIGH 0
#define MEMZONE_PS_DDR_LOW  1
#define MEMZONE_PL_DDR      2
#define MEMZONE_MAX         3

/* Zone flags */
#define ZONE_PS_DDR_LOW  (1 << MEMZONE_PS_DDR_LOW)
#define ZONE_PS_DDR_HIGH (1 << MEMZONE_PS_DDR_HIGH)
#define ZONE_PL_DDR      (1 << MEMZONE_PL_DDR)
#define ZONE_DMA         (ZONE_PS_DDR_LOW | ZONE_PL_DDR)
#define ZONE_PS_DDR      (ZONE_PS_DDR_LOW | ZONE_PS_DDR_HIGH)
#define ZONE_ALL         (ZONE_PS_DDR_LOW | ZONE_PS_DDR_HIGH | ZONE_PL_DDR)

/* IO request types */
#define IOREQ_READ  1
#define IOREQ_WRITE 2
#define IOREQ_FLUSH 3

#endif
