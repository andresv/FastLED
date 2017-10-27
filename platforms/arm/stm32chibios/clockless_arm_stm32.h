#ifndef __INC_CLOCKLESS_ARM_STM32_H
#define __INC_CLOCKLESS_ARM_STM32_H

extern "C" {
    #include "ch.h"
    #include "hal.h"
    #include "string.h"
}

FASTLED_NAMESPACE_BEGIN
#define FASTLED_HAS_CLOCKLESS 1

// ChibiOS SPI config
static const SPIConfig spicfg = {
    NULL,
    GPIOB,
    0,
    SPI_CR1_BR_1 | SPI_CR1_BR_0,
    0
};

#define FASTLED_MAX_LED_NUMBER 100

// 100 LEDS
// each LED has 3 bytes for RGB value
// 1 SPI byte for representing 1 bit (check m_one and m_zero)
// 8 bits in a byte
// 2 bytes is for 0x00 that are sent before and after actual LED data to keep MOSI line low
static uint8_t m_dma_buffer[FASTLED_MAX_LED_NUMBER * 3 * 1 * 8 + 2];
static uint8_t m_one = 0xFC;
static uint8_t m_zero = 0xC0;

template <int DATA_ENABLE_PIN, int T1, int T2, int T3, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 50>
class ClocklessController : public CLEDController {
    CMinWait<WAIT_TIME> mWait;

public:
    virtual void init() {
        // This works only if SYSCLK frequency is 168 MHz:
        // STM32_PLLM_VALUE 8
        // STM32_PLLN_VALUE 336
        // STM32_PLLP_VALUE 2
        // STM32_PLLQ_VALUE 7
        //
        // SPI_CR1_BR_1 set
        // In this case 1 bit in SPI MOSI is 190 ns long.
        //
        // This LED driver needs following timing according to this datasheet https://www.adafruit.com/datasheets/WS2812B.pdf:
        // 1 bit: 0.8 us HIGH and 0.45 us LOW = 1.25 us
        // 0 bit: 0.4 us HIGH and 0.85 us LOW = 1.25 us
        // However it seems that this timing must not be very strict and following also works (measured with Logic):
        // 1 bit: 0.96 us HIGH and 0.56 us LOW = 1.52 us
        // 0 bit: 0.36 us HIGH and 1.16 us LOW = 1.52 us
        // Therefore we can represent this timing using 1 SPI byte. Look m_one and m_zero.
        // It means 1 byte of data is sent out to MOSI to just represent one bit of RGB data for the LED driver.
        // Total of 24 bytes of data is sent to MOSI pin to set RGB value for one LED (1SPI_BYTE * 8BITS_IN_BYTE * 3BYTES_FOR_RGB)

        // first byte is filled with 0x00 that is sent always before actually updating LED data
        // this byte fixes a little glitch in timing that only happens with the first time
        // MOSI goes high before sending actual data
        m_dma_buffer[0] = 0x00;

        spiStart(&SPID1, &spicfg);
    }

    virtual uint16_t getMaxRefreshRate() const {
        return 400;
    }

    virtual void clearLeds(int nLeds) {
        showColor(CRGB(0, 0, 0), nLeds, 0);
    }

protected:

    // set all the leds on the controller to a given color
    virtual void showColor(const struct CRGB & rgbdata, int nLeds, CRGB scale) {
        PixelController<RGB_ORDER> pixels(rgbdata, nLeds, scale, getDither());

        mWait.wait();
        showRGBInternal(pixels);
        mWait.mark();
    }

    virtual void show(const struct CRGB *rgbdata, int nLeds, CRGB scale) {
        PixelController<RGB_ORDER> pixels(rgbdata, nLeds, scale, getDither());

        mWait.wait();
        showRGBInternal(pixels);
        mWait.mark();
    }

    #ifdef SUPPORT_ARGB
    virtual void show(const struct CARGB *rgbdata, int nLeds, CRGB scale) {
        PixelController<RGB_ORDER> pixels(rgbdata, nLeds, scale, getDither());
        mWait.wait();
        showRGBInternal(pixels);
        mWait.mark();
    }
    #endif

    __attribute__ ((always_inline)) inline static uint32_t prepareDMABuffer(uint32_t offset, uint8_t b)  {
        for (int i=0; i<8; i++) {
            if (b & 0x80) {
                *(m_dma_buffer + 1 + offset) = m_one;
            }
            else {
                *(m_dma_buffer + 1 + offset) = m_zero;
            }

            b = b << 1;
            offset += 1;
        }

        return offset;
    }

    // This method is made static to force making register Y available to use for data on AVR - if the method is non-static, then
    // gcc will use register Y for the this pointer.
    static void showRGBInternal(PixelController<RGB_ORDER> & pixels) {

        uint32_t buffer_len = 0;
        memset(m_dma_buffer, 0x00, sizeof(m_dma_buffer));

        // Setup the pixel controller and load/scale the first byte
        pixels.preStepFirstByteDithering();
        register uint8_t b = pixels.loadAndScale0();

        // prepare DMA buffer for SPI transfer
        while (pixels.has(1)) {
            pixels.stepDithering();

            // Write first byte, read next byte
            buffer_len = prepareDMABuffer(buffer_len, b);
            b = pixels.loadAndScale1();

            // Write second byte, read 3rd byte
            buffer_len = prepareDMABuffer(buffer_len, b);
            b = pixels.loadAndScale2();

            // Write third byte, read 1st byte of next pixel
            buffer_len = prepareDMABuffer(buffer_len, b);

            b = pixels.advanceAndLoadAndScale0();

        };

        // set last byte to 0x00, otherwise MISO does not go low
        m_dma_buffer[buffer_len + 1] = 0x00;

        // write to SPI using DMA
        // +2 is first 0x00 byte and last 0x00 byte
        spiSend(&SPID1, buffer_len + 2, m_dma_buffer);
    }
};

FASTLED_NAMESPACE_END

#endif
