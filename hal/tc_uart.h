/*
 * tc_uart.h - The board's UARTs (iLLD ASC driver: interrupt driven, software
 * FIFOs). One instance per port; all on CPU0 (the control loop uses them).
 */
#ifndef TC_UART_H
#define TC_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { UART_LINK = 0, UART_GW, UART_GPS, UART_BRIDGE, UART_COUNT } uart_id_t;

typedef struct {
    uint32_t rx_bytes, tx_bytes;    /* counted by the HAL */
    uint32_t tx_dropped;            /* bytes that didn't fit the software FIFO */
    uint32_t errors;                /* parity / frame / FIFO overflow (ISR) */
    uint32_t last_rx_ms;            /* 0 = never: "no data" detection */
    bool     up;                    /* initialised */
} uart_stat_t;

void     uart_init(uart_id_t id, float baud);
void     uart_set_baud(uart_id_t id, float baud);       /* re-initialise at another baud rate */
uint32_t uart_read(uart_id_t id, uint8_t *buf, uint32_t max);
bool     uart_write(uart_id_t id, const uint8_t *buf, uint32_t len);   /* whole block or nothing */
uint32_t uart_tx_free(uart_id_t id);
const uart_stat_t *uart_stat(uart_id_t id);

#endif /* TC_UART_H */
