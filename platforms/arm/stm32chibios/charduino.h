#ifndef __INC_CH_ARDUINO_H
#define __INC_CH_ARDUINO_H

#include <stdbool.h>
#include <math.h>
#include "ch.h"

typedef bool boolean;

#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1

static inline unsigned long micros(void) {
    return RTC2US(F_CPU, chSysGetRealtimeCounterX());
}

static inline unsigned long millis(void) {
    return micros() / 1000L;
}

static inline void pinMode(uint8_t pin, uint8_t mode) {
    (void)pin;
    (void)mode;
}

static inline void digitalWrite(uint8_t pin, uint8_t value) {
    (void)pin;
    (void)value;
}

static inline void delay(unsigned long ms) {
    chThdSleepMilliseconds(ms);
}

#endif
