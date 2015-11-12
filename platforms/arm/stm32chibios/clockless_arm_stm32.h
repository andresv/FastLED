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
    GPIOC,
    4,
    0
};


#define FASTLED_MAX_LED_NUMBER 100

// 100 LEDS
// each LED has 3 bytes for RGB value
// 3 SPI bytes for representing 1 bit (check m_one and m_zero buffers)
// 8 bits in a byte
static uint8_t m_dma_buffer[FASTLED_MAX_LED_NUMBER*3*3*8];
static uint8_t m_one[3];
static uint8_t m_zero[3];

template <int DATA_PIN, int T1, int T2, int T3, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 50>
class ClocklessController : public CLEDController {
    CMinWait<WAIT_TIME> mWait;

public:
    virtual void init() {

        // This works only if SYSCLK frequency is 153.5 MHz:
        // STM32_PLLM_VALUE 8
        // STM32_PLLN_VALUE 307
        // STM32_PLLP_VALUE 2
        // STM32_PLLQ_VALUE 7
        //
        // In this case 1 bit in SPI MISO is ca 52 ns long.
        // This LED driver needs following timing:
        // 1 bit: 0.8 us HIGH and 0.45 us LOW = 1.25 us
        // 0 bit: 0.4 us HIGH and 0.85 us LOW = 1.25 us
        // 1250 ns / 24 bits (which is 3 bytes) = 52.08 ns
        // For example making 800 ns HIGH signal we set 14 bits high, which gives 781.25 ns (LED driver needs around +- 150 ns precision about timing)
        // m_one and m_zero represent 1 bit and 0 bit timing SPI bytes
        // It means 3 bytes of data is sent out to MOSI to just represent one bit of RGB data for the LED driver.
        // Total of 72 bytes of data is sent to MOSI pin to set RGB value for 1 LED (3SPI_BYTES * 8BITS_IN_BYTE * 3BYTES_FOR_RGB)

        spiStart(&SPID2, &spicfg);
        m_one[0] = 0xFF;
        m_one[1] = 0xFE;
        m_one[2] = 0x00;

        m_zero[0] = 0xFF;
        m_zero[1] = 0x00;
        m_zero[2] = 0x00;
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
            if (b & 0x01) {
                memcpy(m_dma_buffer + offset, m_one, 3);
            }
            else {
                memcpy(m_dma_buffer + offset, m_zero, 3);
            }

            b = b >> 1;
            offset += 3;
        }

        return offset;
    }

    // This method is made static to force making register Y available to use for data on AVR - if the method is non-static, then
    // gcc will use register Y for the this pointer.
    static void showRGBInternal(PixelController<RGB_ORDER> & pixels) {

        uint32_t buffer_len = 0;
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

        // write to SPI using DMA
        spiSend(&SPID2, buffer_len, m_dma_buffer);
    }
};

FASTLED_NAMESPACE_END

#endif
