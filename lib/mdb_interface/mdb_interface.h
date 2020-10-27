#include <stdint.h>
#include "freertos/task.h"
#include "driver/gpio.h"

#ifndef MDB_INTERFACE
#define MDB_INTERFACE

#define MDB_VMC_TXD   18
#define MDB_VMC_RXD   19
#define MDB_PERI_TXD  21
#define MDB_PERI_RXD  22
#define MBD_OUTPUT_PIN_SEL  ((1ULL<<MDB_VMC_TXD) | (1ULL<<MDB_PERI_TXD))
#define MBD_INPUT_PIN_SEL  ((1ULL<<MDB_VMC_RXD) | (1ULL<<MDB_PERI_RXD))

#define MDB_VMC_TXD_SET  1
#define MDB_VMC_TXD_CLR  0
#define MDB_PERI_TXD_SET  0
#define MDB_PERI_TXD_CLR  1 

#define MDB_TXD_DELAY_TICKS  1

#define MDB_MODE_SET  1
#define MDB_MODE_CLR  0
#define MDB_INTERFACE_VMC   0
#define MDB_INTERFACE_PERI  1

#define MDB_EVT_HIGH  1
#define MDB_EVT_LOW   0

#define MDB_SYMBOL_TIME_US  104  // 104.166 uS @ 9600 baud

typedef struct {
    uint64_t evt_time;
    uint32_t gpio;
    uint_fast8_t mdb_evt;
} rx_events;

void mdb_init(void);
void mdb_send(uint32_t interface, uint8_t mode, uint8_t data);
void mdb_read(uint32_t interface, uint8_t* mode, uint8_t* data);
int32_t mdb_data_avail(uint32_t interface);

#endif /* MDB_INTERFACE */
