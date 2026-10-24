/*
 * tc_spi.h - SPI masters (iLLD QSPI SpiMaster). Chip selects are GPIOs,
 * driven by the device drivers.
 *   SPI_BNO: QSPI1, BNO085, mode 3, used by CPU0 (control loop)
 *   SPI_SD : QSPI2, microSD card, mode 0, used by CPU2 (logger) only
 * A bus is only ever used by ONE core.
 */
#ifndef TC_SPI_H
#define TC_SPI_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { SPI_BNO = 0, SPI_SD, SPI_COUNT } spi_bus_t;

void spi_init(spi_bus_t bus, float hz);
void spi_set_speed(spi_bus_t bus, float hz);
/* Full duplex transfer of n bytes (tx NULL: sends 0xFF; rx NULL: discards).
 * false = timeout (the bus is then re-initialised by the next spi_init). */
bool spi_xfer(spi_bus_t bus, const uint8_t *tx, uint8_t *rx, uint32_t n, uint32_t timeout_ms);
uint32_t spi_timeouts(spi_bus_t bus);

#endif /* TC_SPI_H */
