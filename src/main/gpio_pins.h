// gpio_pins.h
#pragma once

// LED pins
#define PIN_LED_RED         46
#define PIN_LED_GREEN       0
#define PIN_LED_BLUE        45

// ADS1256 - ADC PINS
#define ADS1256_1_CS_PIN             GPIO_NUM_7
#define ADS1256_1_DRDY_PIN           GPIO_NUM_6
#define ADS1256_1_RESET_PIN          GPIO_NUM_8
#define ADS1256_2_CS_PIN             GPIO_NUM_10
#define ADS1256_2_DRDY_PIN           GPIO_NUM_9
#define ADS1256_2_RESET_PIN          GPIO_NUM_17

// #define ADS1256_1_CS_PIN             7
// #define ADS1256_1_DRDY_PIN           6
// #define ADS1256_1_RESET_PIN          8
// #define ADS1256_2_CS_PIN             10
// #define ADS1256_2_DRDY_PIN           9
// #define ADS1256_2_RESET_PIN          17

// SPI Configuration
#define ADS1256_SPI_SPEED     2400000  // 2.4MHz
#define ADS1256_HOST          SPI2_HOST
#define NUM_ADCS              2
#define CHANNELS_PER_ADC      8
#define TOTAL_CHANNELS        (NUM_ADCS * CHANNELS_PER_ADC)
#define ADS1256_MOSI_PIN    GPIO_NUM_38
#define ADS1256_MISO_PIN    GPIO_NUM_47
#define ADS1256_CLK_PIN     GPIO_NUM_48

// #define ADS1256_MOSI_PIN    38
// #define ADS1256_MISO_PIN    47
// #define ADS1256_CLK_PIN     48


// ADS1256 Commands
#define ADS1256_CMD_WAKEUP    0x00
#define ADS1256_CMD_RDATA     0x01
#define ADS1256_CMD_RDATAC    0x03
#define ADS1256_CMD_SDATAC    0x0F
#define ADS1256_CMD_RREG      0x10
#define ADS1256_CMD_WREG      0x50
#define ADS1256_CMD_SELFCAL   0xF0
#define ADS1256_CMD_SYNC      0xFC
#define ADS1256_CMD_RESET     0xFE

// Register addresses
#define ADS1256_REG_STATUS    0x00
#define ADS1256_REG_MUX       0x01
#define ADS1256_REG_ADCON     0x02
#define ADS1256_REG_DRATE     0x03

// MUX Pins
#define s0                    GPIO_NUM_3
#define s1                    GPIO_NUM_4
#define s2                    GPIO_NUM_1
#define s3                    GPIO_NUM_2
// #define MUX_EN_PIN            GPIO_NUM_13

// #define s0                    3
// #define s1                    4
// #define s2                    1
// #define s3                    2
#define MUX_EN_PIN            13
#define MUX_EN                GPIO_NUM_13

// I2C PINS
#define I2C_SDA                   11
#define I2C_SCL                   12

// BLE Declarations
#define fNIRS_SERVICE_UUID        "938548e6-c655-11ea-87d0-0242ac130003"
#define DATA_CHARACTERISTIC_UUID  "77539407-6493-4b89-985f-baaf4c0f8d86"
#define DATA_CHARACTERISTIC_UUID2 "513b630c-e5fd-45b5-a678-bb2835d6c1d2"
#define DATA_CHARACTERISTIC_UUID3 "613b680c-e5fd-95b5-a638-bb2233d6c1d7"
#define LED_CHARACTERISTIC_UUID   "19B10001-E8F2-537E-4F6C-D104768A1213"







