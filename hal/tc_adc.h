/*
 * tc_adc.h - Analog inputs (iLLD EVADC, queue 0 in refill mode: the converter
 * measures the channels over and over by itself; reading = the newest
 * result, never waits). Used by CPU0. Channels (hal_pins.h): the steering
 * motor current sense and the supply voltage divider, both in EVADC group 8.
 */
#ifndef TC_ADC_H
#define TC_ADC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { ADC_MOTOR_CS = 0, ADC_SUPPLY, ADC_COUNT } adc_in_t;

void adc_init(void);
/* newest 12-bit result (0..4095 = 0..3.3 V); false = no conversion yet / not running */
bool adc_read(adc_in_t in, uint16_t *raw);
float adc_volts(uint16_t raw);

#endif /* TC_ADC_H */
