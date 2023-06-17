#pragma once

/* Delay to stabilize the oscillator. */
#if !defined(PICO_XOSC_STARTUP_DELAY_MULTIPLIER)
# define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#endif

/* QSPI flash access protocol selection. */
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
//#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 1

/* Flash access clock divider. */
#if !defined(PICO_FLASH_SPI_CLKDIV)
# define PICO_FLASH_SPI_CLKDIV 2
#endif

/* Total flash size in bytes. */
#if !defined(PICO_FLASH_SIZE_BYTES)
# define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

/* Buttons */
#define NUM_SWITCHES 1
#define SWITCH_PULL_UP 0
#define SWITCH_DEBOUNCE_US 0
#define SW1_PIN 11

/* IR */
#define IR_TX_PIN 9
#define IR_TX_SLICE 4
#define IR_TX_CHAN PWM_CHAN_B
#define IR_EN_PIN 10
#define IR_RX_PIN 26
#define IR_OSC_PIN 17
#define IR_OSC_SLICE 0
#define IR_OSC_CHAN PWM_CHAN_B

/* Buzzer */
#define BZ_PIN 27
#define BZ_SLICE 5
#define BZ_CHAN PWM_CHAN_B

/* Microphone */
#define MIC_DATA_PIN 0
#define MIC_CLK_PIN 1

/* WS2812B */
#define LED_PIN 7
#define LED_RED_MUL 1.0f
#define LED_GREEN_MUL 0.6f
#define LED_BLUE_MUL 2.0f

/* System clock frequency. */
#define SYS_CLOCK_HZ (125 * 1000 * 1000)
