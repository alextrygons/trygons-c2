/*
 * tc_spi.c - SPI masters (iLLD QSPI SpiMaster, interrupt driven). See tc_spi.h.
 */
#include "tc_spi.h"

#include <string.h>

#include "Ifx_Types.h"
#include "Qspi/SpiMaster/IfxQspi_SpiMaster.h"
#include "IfxCpu_Irq.h"

#include "hal.h"
#include "hal_pins.h"

#define CHUNK 64u                       /* bytes per exchange (the driver's 8-bit FIFO jobs) */

typedef struct {
    IfxQspi_SpiMaster         master;
    IfxQspi_SpiMaster_Channel ch;
    float    hz;
    bool     up;
    uint32_t timeouts;
} spi_t;

static spi_t s_spi[SPI_COUNT];

/* interrupts: QSPI1 on CPU0, QSPI2 on CPU2 (vector table of the core that uses it) */
IFX_INTERRUPT(spi_bno_tx_isr, 0, PRIO_BNO_TX) { IfxQspi_SpiMaster_isrTransmit(&s_spi[SPI_BNO].master); }
IFX_INTERRUPT(spi_bno_rx_isr, 0, PRIO_BNO_RX) { IfxQspi_SpiMaster_isrReceive(&s_spi[SPI_BNO].master); }
IFX_INTERRUPT(spi_bno_er_isr, 0, PRIO_BNO_ER) { IfxQspi_SpiMaster_isrError(&s_spi[SPI_BNO].master); }
IFX_INTERRUPT(spi_sd_tx_isr, 2, PRIO_SD_TX) { IfxQspi_SpiMaster_isrTransmit(&s_spi[SPI_SD].master); }
IFX_INTERRUPT(spi_sd_rx_isr, 2, PRIO_SD_RX) { IfxQspi_SpiMaster_isrReceive(&s_spi[SPI_SD].master); }
IFX_INTERRUPT(spi_sd_er_isr, 2, PRIO_SD_ER) { IfxQspi_SpiMaster_isrError(&s_spi[SPI_SD].master); }

static void channel_init(spi_t *p, spi_bus_t bus, float hz)
{
    IfxQspi_SpiMaster_ChannelConfig cc;
    IfxQspi_SpiMaster_initChannelConfig(&cc, &p->master);
    cc.ch.baudrate = hz;
    cc.ch.mode.dataWidth = 8;
    cc.ch.mode.dataHeading = TRUE;                      /* MSB first */
    /* AURIX ECON.CPH is the inverse of the usual SPI CPHA: the iLLD default (idle low +
     * "shift on leading edge" = CPH 1) is SPI mode 0. If a device doesn't answer, check
     * these two on the scope first (FIRST_TEST.md, bench test). */
    if (bus == SPI_BNO) {                               /* SPI mode 3 (CPOL 1, CPHA 1) */
        cc.ch.mode.clockPolarity = IfxQspi_ClockPolarity_idleHigh;
        cc.ch.mode.shiftClock = IfxQspi_ShiftClock_shiftTransmitDataOnTrailingEdge;
    } else {                                            /* SPI mode 0 (CPOL 0, CPHA 0) */
        cc.ch.mode.clockPolarity = IfxQspi_ClockPolarity_idleLow;
        cc.ch.mode.shiftClock = IfxQspi_ShiftClock_shiftTransmitDataOnLeadingEdge;
    }
    cc.ch.channelId = (bus == SPI_BNO) ? IfxQspi_ChannelId_0 : IfxQspi_ChannelId_4;
    cc.sls.output.pin = NULL_PTR;                       /* chip select: a GPIO of the device driver */
    cc.dummyTxValue = 0xFFu;
    (void)IfxQspi_SpiMaster_initChannel(&p->ch, &cc);
    p->hz = hz;
}

void spi_init(spi_bus_t bus, float hz)
{
    spi_t *p = &s_spi[bus];
    IfxQspi_SpiMaster_Config c;
    if (bus == SPI_BNO) {
        static const IfxQspi_SpiMaster_Pins pins = {
            &BNO_SCLK, IfxPort_OutputMode_pushPull, &BNO_MTSR, IfxPort_OutputMode_pushPull,
            &BNO_MRST, IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed3 };
        IfxQspi_SpiMaster_initModuleConfig(&c, &BNO_QSPI);
        c.pins = &pins;
        c.txPriority = PRIO_BNO_TX;
        c.rxPriority = PRIO_BNO_RX;
        c.erPriority = PRIO_BNO_ER;
        c.isrProvider = IfxSrc_Tos_cpu0;
    } else {
        static const IfxQspi_SpiMaster_Pins pins = {
            &SD_SCLK, IfxPort_OutputMode_pushPull, &SD_MTSR, IfxPort_OutputMode_pushPull,
            &SD_MRST, IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed3 };
        IfxQspi_SpiMaster_initModuleConfig(&c, &SD_QSPI);
        c.pins = &pins;
        c.txPriority = PRIO_SD_TX;
        c.rxPriority = PRIO_SD_RX;
        c.erPriority = PRIO_SD_ER;
        c.isrProvider = IfxSrc_Tos_cpu2;
    }
    c.maximumBaudrate = 25000000.0f;
    IfxQspi_SpiMaster_initModule(&p->master, &c);
    channel_init(p, bus, hz);
    p->up = TRUE;
}

void spi_set_speed(spi_bus_t bus, float hz)
{
    spi_t *p = &s_spi[bus];
    if (!p->up) { spi_init(bus, hz); return; }
    if (p->hz != hz) (void)IfxQspi_SpiMaster_setChannelBaudrate(&p->ch, hz);
    p->hz = hz;
}

bool spi_xfer(spi_bus_t bus, const uint8_t *tx, uint8_t *rx, uint32_t n, uint32_t timeout_ms)
{
    spi_t *p = &s_spi[bus];
    static uint8 ones[CHUNK], sink[SPI_COUNT][CHUNK];
    if (!p->up) return false;
    if (ones[0] != 0xFFu) memset(ones, 0xFF, sizeof ones);
    uint32_t t0 = hal_millis();
    while (n > 0u) {
        uint32_t k = n < CHUNK ? n : CHUNK;
        const uint8 *src = tx ? tx : ones;
        uint8 *dst = rx ? rx : sink[bus];
        if (IfxQspi_SpiMaster_exchange(&p->ch, src, dst, (Ifx_SizeT)k) != IfxQspi_Status_ok) {
            p->timeouts++;
            return false;
        }
        while (IfxQspi_SpiMaster_getStatus(&p->ch) == IfxQspi_Status_busy) {
            if ((uint32_t)(hal_millis() - t0) > timeout_ms) {
                p->timeouts++;
                p->up = FALSE;                          /* stuck: initialise again before the next use */
                return false;
            }
        }
        if (tx) tx += k;
        if (rx) rx += k;
        n -= k;
    }
    return true;
}

uint32_t spi_timeouts(spi_bus_t bus)
{
    return s_spi[bus].timeouts;
}
