/*
 * tc_uart.c - The board's UARTs. See tc_uart.h. Board only (ADS build).
 *
 * The iLLD ASC driver moves bytes between the ASCLIN hardware FIFOs and a
 * software FIFO in its interrupts; uart_read / uart_write only touch the
 * software FIFOs, so they never wait.
 */
#include "tc_uart.h"

#include "Ifx_Types.h"
#include "Asclin/Asc/IfxAsclin_Asc.h"
#include "IfxCpu_Irq.h"

#include "hal.h"
#include "hal_pins.h"

/* software FIFO sizes per port; the gateway's TX must be HAL_GW_TX_RING (link.c
 * computes the backlog as HAL_GW_TX_RING - hal_gw_tx_free()) */
#define FIFO_BYTES(n) ((n) + sizeof(Ifx_Fifo) + 8u)
static uint8 s_link_rx[FIFO_BYTES(2048u)], s_link_tx[FIFO_BYTES(4096u)];
static uint8 s_gw_rx[FIFO_BYTES(2048u)],   s_gw_tx[FIFO_BYTES(HAL_GW_TX_RING)];
static uint8 s_gps_rx[FIFO_BYTES(1024u)],  s_gps_tx[FIFO_BYTES(256u)];
static uint8 s_br_rx[FIFO_BYTES(1024u)],   s_br_tx[FIFO_BYTES(64u)];
static uint8 s_rl_rx[FIFO_BYTES(128u)],    s_rl_tx[FIFO_BYTES(64u)];     /* Modbus: frames of <= 16 bytes */

typedef struct {
    IfxAsclin_Asc asc;
    uart_stat_t st;
    float baud;
} uart_t;

static uart_t s_u[UART_COUNT];
static void (*volatile s_txdone_fn[UART_COUNT])(void);   /* RS485: release the driver at the frame end */

typedef struct {
    Ifx_ASCLIN *mod;
    IfxAsclin_Tx_Out *tx;
    IfxAsclin_Rx_In *rx;
    uint16 prio_tx, prio_rx, prio_er;
    uint8 *rxbuf, *txbuf;
    uint16 rx_size, tx_size;
} uart_hw_t;

static const uart_hw_t k_hw[UART_COUNT] = {
    { &LINK_ASC,   &LINK_TX,   &LINK_RX,   PRIO_LINK_TX, PRIO_LINK_RX, PRIO_LINK_ER, s_link_rx, s_link_tx, 2048u, 4096u },
    { &GW_ASC,     &GW_TX,     &GW_RX,     PRIO_GW_TX,   PRIO_GW_RX,   PRIO_GW_ER,   s_gw_rx,   s_gw_tx,   2048u, HAL_GW_TX_RING },
    { &GPS_ASC,    &GPS_TX,    &GPS_RX,    PRIO_GPS_TX,  PRIO_GPS_RX,  PRIO_GPS_ER,  s_gps_rx,  s_gps_tx,  1024u, 256u },
    { &BRIDGE_ASC, &BRIDGE_TX, &BRIDGE_RX, PRIO_BR_TX,   PRIO_BR_RX,   PRIO_BR_ER,   s_br_rx,   s_br_tx,   1024u, 64u },
    { &RELAY_ASC,  &RELAY_TX,  &RELAY_RX,  PRIO_RL_TX,   PRIO_RL_RX,   PRIO_RL_ER,   s_rl_rx,   s_rl_tx,   128u,  64u },
};

/* The error interrupt also carries the TC (transmission completed) flag (ASCLIN EX
 * service request; iLLD Lin: exTransmissionCompleted). Only armed (TCE) on a port
 * with uart_on_tx_done: when the last frame is out and nothing is left in the
 * software / hardware FIFO, the hook runs once and TCE goes off again. */
static boolean tx_done_check(uart_id_t id);

/* --- interrupts (CPU0) ------------------------------------------------------------ */
#define UART_ISRS(n, id, ptx, prx, per)                                                   \
    IFX_INTERRUPT(uart##n##_tx_isr, 0, ptx) { IfxAsclin_Asc_isrTransmit(&s_u[id].asc); }   \
    IFX_INTERRUPT(uart##n##_rx_isr, 0, prx) { IfxAsclin_Asc_isrReceive(&s_u[id].asc); }    \
    IFX_INTERRUPT(uart##n##_er_isr, 0, per) {                                             \
        boolean tc = tx_done_check(id);                                                   \
        IfxAsclin_Asc_isrError(&s_u[id].asc);                                             \
        if (!tc) s_u[id].st.errors++;                                                     \
    }

UART_ISRS(0, UART_LINK,   PRIO_LINK_TX, PRIO_LINK_RX, PRIO_LINK_ER)
UART_ISRS(1, UART_GW,     PRIO_GW_TX,   PRIO_GW_RX,   PRIO_GW_ER)
UART_ISRS(2, UART_GPS,    PRIO_GPS_TX,  PRIO_GPS_RX,  PRIO_GPS_ER)
UART_ISRS(3, UART_BRIDGE, PRIO_BR_TX,   PRIO_BR_RX,   PRIO_BR_ER)
UART_ISRS(4, UART_RELAY,  PRIO_RL_TX,   PRIO_RL_RX,   PRIO_RL_ER)

/* true = this interrupt was the TC flag (not an error) */
static boolean tx_done_check(uart_id_t id)
{
    uart_t *u = &s_u[id];
    if (s_txdone_fn[id] == NULL || !IfxAsclin_getTransmissionCompletedFlagStatus(u->asc.asclin)) return FALSE;
    IfxAsclin_clearTransmissionCompletedFlag(u->asc.asclin);
    if (IfxAsclin_Asc_getWriteCount(&u->asc) >= (sint32)k_hw[id].tx_size &&
        IfxAsclin_getTxFifoFillLevel(u->asc.asclin) == 0u) {
        IfxAsclin_enableTransmissionCompletedFlag(u->asc.asclin, FALSE);
        s_txdone_fn[id]();                              /* the last stop bit is out: release the bus now */
    }
    return TRUE;
}

void uart_on_tx_done(uart_id_t id, void (*fn)(void))
{
    s_txdone_fn[id] = fn;
}

void uart_init(uart_id_t id, float baud)
{
    uart_t *u = &s_u[id];
    const uart_hw_t *h = &k_hw[id];
    IfxAsclin_Asc_Config c;
    IfxAsclin_Asc_initModuleConfig(&c, h->mod);
    c.baudrate.baudrate = baud;
    c.interrupt.txPriority = h->prio_tx;
    c.interrupt.rxPriority = h->prio_rx;
    c.interrupt.erPriority = h->prio_er;
    c.interrupt.typeOfService = IfxSrc_Tos_cpu0;
    IfxAsclin_Asc_Pins pins = {
        NULL_PTR, IfxPort_InputMode_pullUp,             /* no CTS */
        h->rx, IfxPort_InputMode_pullUp,                /* RX (idle high) */
        NULL_PTR, IfxPort_OutputMode_pushPull,          /* no RTS */
        h->tx, IfxPort_OutputMode_pushPull,             /* TX */
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    c.pins = &pins;
    c.rxBuffer = h->rxbuf;
    c.rxBufferSize = h->rx_size;
    c.txBuffer = h->txbuf;
    c.txBufferSize = h->tx_size;
    IfxAsclin_Asc_initModule(&u->asc, &c);
    u->baud = baud;
    u->st.up = TRUE;
}

void uart_set_baud(uart_id_t id, float baud)
{
    if (s_u[id].st.up && s_u[id].baud == baud) return;
    if (s_u[id].st.up) IfxAsclin_Asc_disableModule(&s_u[id].asc);
    uart_init(id, baud);
}

uint32_t uart_read(uart_id_t id, uint8_t *buf, uint32_t max)
{
    uart_t *u = &s_u[id];
    if (!u->st.up || max == 0u) return 0u;
    sint32 avail = IfxAsclin_Asc_getReadCount(&u->asc);
    if (avail <= 0) return 0u;
    Ifx_SizeT n = (Ifx_SizeT)((uint32_t)avail < max ? (uint32_t)avail : max);
    (void)IfxAsclin_Asc_read(&u->asc, buf, &n, 0);                     /* no waiting */
    if (n > 0) {
        u->st.rx_bytes += (uint32_t)n;
        u->st.last_rx_ms = hal_millis() ? hal_millis() : 1u;
    }
    return (uint32_t)n;
}

uint32_t uart_tx_free(uart_id_t id)
{
    uart_t *u = &s_u[id];
    if (!u->st.up) return 0u;
    sint32 f = IfxAsclin_Asc_getWriteCount(&u->asc);
    return f > 0 ? (uint32_t)f : 0u;
}

bool uart_write(uart_id_t id, const uint8_t *buf, uint32_t len)
{
    uart_t *u = &s_u[id];
    if (!u->st.up) return false;
    if (uart_tx_free(id) < len) {                       /* whole frames only: a cut frame is garbage */
        u->st.tx_dropped += len;
        return false;
    }
    Ifx_SizeT n = (Ifx_SizeT)len;
    IfxAsclin_clearTransmissionCompletedFlag(u->asc.asclin);   /* uart_tx_done: this frame's end */
    if (s_txdone_fn[id] != NULL) IfxAsclin_enableTransmissionCompletedFlag(u->asc.asclin, TRUE);  /* RS485 */
    (void)IfxAsclin_Asc_write(&u->asc, buf, &n, 0);
    u->st.tx_bytes += (uint32_t)n;
    return n == (Ifx_SizeT)len;
}

bool uart_tx_done(uart_id_t id)
{
    uart_t *u = &s_u[id];
    if (!u->st.up) return true;
    /* software FIFO empty, hardware TX FIFO empty, the shift register done (TC flag:
     * cleared by uart_write before every frame) */
    return IfxAsclin_Asc_getWriteCount(&u->asc) >= (sint32)k_hw[id].tx_size &&
           IfxAsclin_getTxFifoFillLevel(u->asc.asclin) == 0u &&
           IfxAsclin_getTransmissionCompletedFlagStatus(u->asc.asclin);
}

const uart_stat_t *uart_stat(uart_id_t id)
{
    return &s_u[id].st;
}
