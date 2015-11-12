#ifndef __INC_LED_SYSDEFS_ARM_SAM_H
#define __INC_LED_SYSDEFS_ARM_SAM_H

#include "stdio.h"

#define FASTLED_ARM

// Those pins are defined just to silence warnings
#define SPI_DATA 0
#define SPI_CLOCK 0
#define HAS_HARDWARE_PIN_SUPPORT

#ifndef INTERRUPT_THRESHOLD
#define INTERRUPT_THRESHOLD 1
#endif

// Default to allowing interrupts
#ifndef FASTLED_ALLOW_INTERRUPTS
#define FASTLED_ALLOW_INTERRUPTS 1
#endif

#if FASTLED_ALLOW_INTERRUPTS == 1
#define FASTLED_ACCURATE_CLOCK
#endif

// reusing/abusing cli/sei defs for due
#define cli()  __disable_irq(); __disable_fault_irq();
#define sei() __enable_irq(); __enable_fault_irq();

// pgmspace definitions
#define PROGMEM
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_dword_near(addr) pgm_read_dword(addr)

// Default to NOT using PROGMEM here
#ifndef FASTLED_USE_PROGMEM
#define FASTLED_USE_PROGMEM 0
#endif

// data type defs
typedef volatile       uint8_t RoReg; /**< Read only 8-bit register (volatile const unsigned int) */
typedef volatile       uint8_t RwReg; /**< Read-Write 8-bit register (volatile unsigned int) */
typedef uint8_t        byte;

#define FASTLED_NO_PINMAP

#endif
