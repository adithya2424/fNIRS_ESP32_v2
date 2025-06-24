#include "Arduino.h"
#include "NimBLEDevice.h"
#include "gpio_pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "Adafruit_MCP4728.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "Wire.h"
#include "SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h"
#include "driver/spi_master.h"
#include "CD74HC4067.h"
#include "rom/ets_sys.h" 

// #define DEBUG_MODE
#define STORAGE_NAMESPACE "led_storage"
#define LED_INTENSITIES_KEY "led_intensities"

// Define battery thresholds
#define LOW_BATT_THRESHOLD 20
#define CRITICAL_BATT_THRESHOLD 5

SFE_MAX1704X lipo(MAX1704X_MAX17043);
CD74HC4067 my_mux(s0, s1, s2, s3);

static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t drdy_semaphore1; // NEW: Semaphore for ADC1 DRDY
static SemaphoreHandle_t drdy_semaphore2; // NEW: Semaphore for ADC2 DRDY

// --- Global Variables ---
static bool readDataBool = false;
const int NUM_SOURCES = 32 + 1; // regular power light source, low power light source, dark
#define NUM_LEDS (NUM_SOURCES-1)  // 32 LEDs based on your array
const int DETECTORS_PER_SOURCE = 16; // 16 detectors + round duration
const int DATA_SIZE_BYTES = 4; // 24 bits

// Each source cycle stores 16 detector values plus one timestamp
#define SLOTS_PER_SOURCE (DETECTORS_PER_SOURCE + 1)
const int PACKET_SIZE = NUM_SOURCES * SLOTS_PER_SOURCE; // * DATA_SIZE_BYTES; // 384 bytes for data + 6 bytes for header
int data_packet[PACKET_SIZE]; // First data packet
int dataPacketToSend[PACKET_SIZE]; // First data packet
int darkCurrentValues[SLOTS_PER_SOURCE];

static int ledIntensities[NUM_SOURCES-1] = {100, 100, 100, 100,
                          100, 100, 100, 100,
                          100, 100, 100, 100,
                          100, 100, 100, 100,
                          50, 50, 50, 50,
                          50, 50, 50, 50,
                          50, 50, 50, 50,
                          50, 50, 50, 50
                          };

Adafruit_MCP4728 mcp; // DAC object

// Storage queue for async operations
QueueHandle_t storageQueue;

// --- BLE UUIDs and Device Name ---
#define DEVICE_NAME               "BBOL NIRDuino (Nano ESP32)"
#define FNIRS_SERVICE_UUID        "938548e6-c655-11ea-87d0-0242ac130003"
#define DATA_CHARACTERISTIC_UUID  "77539407-6493-4b89-985f-baaf4c0f8d86"
#define DATA_CHARACTERISTIC_UUID2 "513b630c-e5fd-45b5-a678-bb2835d6c1d2"
#define LED_CHARACTERISTIC_UUID   "19B10001-E8F2-537E-4F6C-D104768A1213"

// --- Global BLE Pointers ---
NimBLEServer* pServer = NULL;
NimBLECharacteristic* pDataCharacteristic = NULL;
NimBLECharacteristic* pDataCharacteristic2 = NULL;
NimBLECharacteristic* pLEDCharacteristic = NULL;

static const char *TAG = "NIRduino";

// Structure to hold ADC configuration
// --- ADC Configuration Struct ---
typedef struct {
    spi_device_handle_t spi_handle;
    gpio_num_t cs_pin;
    gpio_num_t drdy_pin;
    gpio_num_t reset_pin;
    const char* name;
    SemaphoreHandle_t drdy_sem;
} ads1256_config_t;

static bool deviceConnected = false;

// --- NEW: ISR Handler for DRDY signal ---
static void IRAM_ATTR drdy_isr_handler(void* arg) {
    SemaphoreHandle_t drdy_sem = (SemaphoreHandle_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(drdy_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/* @brief Sets the RGB LED color.
 * Assumes a common-anode setup where LOW turns the LED ON.
*/
static void setLedColor(int red, int green, int blue) {
    digitalWrite(PIN_LED_RED,   red > 0 ? LOW : HIGH);
    digitalWrite(PIN_LED_GREEN, green > 0 ? LOW : HIGH);
    digitalWrite(PIN_LED_BLUE,  blue > 0 ? LOW : HIGH);
}

/* @brief Sets the RGB LED color.
 * Sets PWM values to control led color.
*/
static void setRGB(int red, int green, int blue) {
  analogWrite(redOut, red);
  analogWrite(greenOut, green);
  analogWrite(blueOut, blue);
}


static int getLEDIntensity(int sourceNumber){

    if ((sourceNumber % 2) == 0){ // red

      if (ledIntensities[sourceNumber] == 0){   
        // disable MUX, active low so needs to be set HIGH
        gpio_set_level(MUX_EN, 1);
        return 0;
      }
      else{ 
        // enable MUX, active low so needs to be set LOW
        gpio_set_level(MUX_EN, 0);
        // float voltage = ((ledIntensities[sourceNumber] + 155.0)/53.072)*(4095.0/5.0);
        float voltage = (ledIntensities[sourceNumber]/255.0)*4095.0;
        return (int) voltage;
      }
    }
    else{

      if (ledIntensities[sourceNumber] == 0){
        // disable MUX, active low so needs to be set HIGH
        gpio_set_level(MUX_EN, 1);
        return 0;
      }
      else{
        // enable MUX, active low so needs to be set LOW
        gpio_set_level(MUX_EN, 0);
        // float voltage = ((ledIntensities[sourceNumber] + 177.0)/79.367)*(4095.0/5.0);
        float voltage = (ledIntensities[sourceNumber]/255.0)*4095.0;
        return (int) voltage;
      }

    }
}

// --- Placeholder functions for your application logic ---
static void set_source_state(int sourceNumber) {
    // TODO: Implement your logic to turn on the correct LED/source
    // This will be called at the start of each measurement cycle.
    // For example: turn_on_led(sourceNumber);
     // Case for dark current measurement (all LEDs off)
    gpio_set_level(MUX_EN, 1);
    if (sourceNumber == 32) {
        // Disable MUX (active low, so set HIGH)
        gpio_set_level(MUX_EN, 1);

        // Take the mutex before using the I2C bus
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        
        // Set DAC output to 0 to turn off any active LED
        mcp.setChannelValue(MCP4728_CHANNEL_A, 0);

        // Give the mutex back immediately after the I2C command is sent
        xSemaphoreGive(i2c_mutex); 
        }
    }
    else {
        // Enable MUX for LED sources (active low, so set LOW)
        gpio_set_level(MUX_EN, 0);

        int muxPin = (sourceNumber < 16) ? sourceNumber : sourceNumber - 16;
        
        // Set MUX to the correct channel
        my_mux.channel(muxPin);

        // Take the mutex before using the I2C bus
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        
            mcp.setChannelValue(MCP4728_CHANNEL_A, getLEDIntensity(sourceNumber));

            // Give the mutex back immediately after the I2C command is sent
            xSemaphoreGive(i2c_mutex);

        }

        // Add a small, blocking delay to let the DAC voltage stabilize before ADC reading.
        // esp_rom_delay_us is used for short, precise delays inside a real-time task.
        esp_rom_delay_us(425);
    }
}

//------------------------------------------------------------------------ ADC CODE ------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--- using sempahore context code here---//
// --- Application Configuration ---
#define SAMPLING_FREQUENCY_HZ 251 // Set desired sampling frequency in Hz
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQUENCY_HZ)
static esp_timer_handle_t periodic_timer;

// --- Global Handles ---
static SemaphoreHandle_t adc_semaphore;
static SemaphoreHandle_t ble_semaphore; // Semaphore to signal BLE task
static ads1256_config_t adc1_config;
static ads1256_config_t adc2_config;

// --- Low-Level SPI Functions (Unchanged) ---
static void ADS1256_cmd(spi_device_handle_t spi, const uint8_t cmd) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void ADS1256_write_reg(spi_device_handle_t spi, const uint8_t reg, const uint8_t data) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 24;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = 0x50 | reg;
    t.tx_data[1] = 0x00;
    t.tx_data[2] = data;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void ADS1256_read_data(spi_device_handle_t spi, uint8_t* buffer) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 24;
    t.rx_buffer = buffer;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void ADS1256_wait_drdy(gpio_num_t drdy_pin) {
    while (gpio_get_level(drdy_pin) == 1) {}
}


// --- High-Level ADS1256 Functions (Unchanged) ---
static void ADS1256_init(ads1256_config_t* adc) {
    ESP_LOGI(TAG, "Initializing %s", adc->name);
    gpio_set_direction(adc->reset_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(adc->reset_pin, 1); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(adc->reset_pin, 0); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(adc->reset_pin, 1); vTaskDelay(pdMS_TO_TICKS(10));

    // Configure DRDY pin and attach ISR
    gpio_set_direction(adc->drdy_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(adc->drdy_pin, GPIO_FLOATING); // Use pullup for stability
    gpio_set_intr_type(adc->drdy_pin, GPIO_INTR_ANYEDGE); // Trigger on falling edge
    gpio_isr_handler_add(adc->drdy_pin, drdy_isr_handler, (void*)adc->drdy_sem);

    gpio_set_level(adc->cs_pin, 0);
    xSemaphoreTake(adc->drdy_sem, pdMS_TO_TICKS(100)); // Wait for DRDY using semaphore
    ADS1256_cmd(adc->spi_handle, 0xFE); // RESET
    vTaskDelay(pdMS_TO_TICKS(10));

    ADS1256_write_reg(adc->spi_handle, 0x00, 0x01); // STATUS
    ADS1256_write_reg(adc->spi_handle, 0x02, 0x00); // ADCON
    ADS1256_write_reg(adc->spi_handle, 0x03, 0xD0); // DRATE
    ADS1256_cmd(adc->spi_handle, 0xF0); // SELFCAL
    xSemaphoreTake(adc->drdy_sem, pdMS_TO_TICKS(100)); // Wait for calibration to finish
    gpio_set_level(adc->cs_pin, 1);
    ESP_LOGI(TAG, "%s initialized and calibrated.", adc->name);
}


static int32_t ADS1256_read_channel(ads1256_config_t* adc, uint8_t mux_setting) {
    int32_t adc_val = 0;
    uint8_t read_buffer[3];
    gpio_set_level(adc->cs_pin, 0);

    // Wait for the previous conversion to finish.
    if (xSemaphoreTake(adc->drdy_sem, pdMS_TO_TICKS(5)) != pdTRUE) {
        ESP_LOGE(adc->name, "DRDY timeout before MUX write!");
        gpio_set_level(adc->cs_pin, 1);
        return -1; // Indicate an error
    }

    ADS1256_write_reg(adc->spi_handle, 0x01, mux_setting);
    ADS1256_cmd(adc->spi_handle, 0xFC); // SYNC
    ADS1256_cmd(adc->spi_handle, 0x00); // WAKEUP

    // Wait for the new conversion to complete.
    if (xSemaphoreTake(adc->drdy_sem, pdMS_TO_TICKS(5)) != pdTRUE) {
        ESP_LOGE(adc->name, "DRDY timeout after WAKEUP!");
        gpio_set_level(adc->cs_pin, 1);
        return -1;
    }

    ADS1256_cmd(adc->spi_handle, 0x01); // RDATA
    ADS1256_read_data(adc->spi_handle, read_buffer);
    gpio_set_level(adc->cs_pin, 1);

    adc_val = ((int32_t)read_buffer[0] << 16) | ((int32_t)read_buffer[1] << 8) | (read_buffer[2]);
    if (adc_val & 0x800000) { adc_val |= 0xFF000000; }
    return adc_val;
}


// --- Real-Time Architecture ---

/**
 * @brief Timer ISR that gives the semaphore to unblock the ADC task.
 * @param arg Not used.
 */
static void IRAM_ATTR timer_callback(void* arg) {
    // Give the semaphore from the ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(adc_semaphore, &xHigherPriorityTaskWoken);
    // Yield if a higher priority task was unblocked
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Dedicated task for sampling ADCs when triggered by the timer.
 * @param arg Not used.
 */
static void adc_sampling_task(void* arg) {
    // Arrays to hold the raw ADC values for one complete cycle
    int32_t adc1_values[8];
    int32_t adc2_values[8];

    // The current light source to be activated. Static to persist across calls.
    static int sourceNumber = 0;
    
    // Timestamp of the last measurement cycle, in milliseconds.
    // Static to persist across calls.
    static uint32_t last_cycle_timestamp_ms = 0;

    // Multiplexer settings for the 8 channels of the ADS1256
    uint8_t mux[8] = {0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78};

    ESP_LOGI(TAG, "ADC sampling task started.");

    while (1) {
        // Wait for the semaphore from the timer ISR. This is CPU efficient.
        if (xSemaphoreTake(adc_semaphore, portMAX_DELAY) == pdTRUE) {
            uint64_t cycle_start_time = esp_timer_get_time();

            // 1. Activate the correct light source for this measurement cycle
            set_source_state(sourceNumber);
           
            // 2. Read all 16 channels (8 from each ADC) as quickly as possible
            for (int i = 0; i < 8; i++) {
                adc1_values[i] = ADS1256_read_channel(&adc1_config, mux[i]);
                adc2_values[i] = ADS1256_read_channel(&adc2_config, mux[i]);
            }
            
            // 3. Store the collected data, remapping channels to match Arduino implementation.
            // Arduino storage order: AIN1->AIN7 (Detectors 1-7), then AIN0 (Detector 8).
            // Current read order: AIN0->AIN7 stored in adcX_values[0] through adcX_values[7].
            int base_index = sourceNumber * SLOTS_PER_SOURCE;

            // --- Remap and store ADC1 data ---
            // Copy AIN1-AIN7 (from adc1_values[1]..[7]) to the first 7 slots.
            memcpy(&data_packet[base_index], &adc1_values[1], 7 * sizeof(int32_t));
            // Copy AIN0 (from adc1_values[0]) to the 8th slot.
            data_packet[base_index + 7] = adc1_values[0];

            // --- Remap and store ADC2 data ---
            // Copy AIN1-AIN7 (from adc2_values[1]..[7]) to the next 7 slots (detectors 9-15).
            memcpy(&data_packet[base_index + 8], &adc2_values[1], 7 * sizeof(int32_t));
            // Copy AIN0 (from adc2_values[0]) to the 16th slot (detector 16).
            data_packet[base_index + 15] = adc2_values[0];

            // 4. Calculate and store the time interval since the last measurement.
            // uint32_t current_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
            uint32_t current_time_ms = (uint32_t)(millis());
            
            if (last_cycle_timestamp_ms == 0) {
                last_cycle_timestamp_ms = current_time_ms;
            }

            uint32_t interval = current_time_ms - last_cycle_timestamp_ms;
            data_packet[base_index + DETECTORS_PER_SOURCE] = interval;
            
            last_cycle_timestamp_ms = current_time_ms;

            #ifdef DEBUG_MODE
            uint64_t cycle_duration_us = esp_timer_get_time() - cycle_start_time;
            // Print results for debugging purposes for the current source
            printf("Source: %-2d | ADC1: ", sourceNumber);
            for (int i = 0; i < 8; i++) { printf("%-9ld ", adc1_values[i]); }
            printf("\n            | ADC2: ");
            for (int i = 0; i < 8; i++) { printf("%-9ld ", adc2_values[i]); }
            printf("| Stored Interval: %-4lu ms | Cycle Time: %llu us\n\n", interval, cycle_duration_us);
            #endif
          
            // 5. Advance the source number. If a full round is complete,
            // copy the data to the send buffer and reset. This matches the Arduino logic.
            if (sourceNumber == 32) { // After processing source 32 (the 33rd cycle)
                memcpy(dataPacketToSend, data_packet, sizeof(data_packet));
                xSemaphoreGive(ble_semaphore);
                sourceNumber = 0; // Reset for the next full round
            } else {
                sourceNumber++; // Advance to the next source
            }
        }
    }
}

/**
 * @brief Encapsulates all ADC hardware and task initialization.
 */
void init_adc_system(void) {
    ESP_LOGI(TAG, "Initializing ADC system...");
    esp_err_t ret;
    
    // --- 1. Create Semaphore ---
    adc_semaphore = xSemaphoreCreateBinary();
    if (adc_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }
    drdy_semaphore1 = xSemaphoreCreateBinary(); // Create new semaphores
    drdy_semaphore2 = xSemaphoreCreateBinary();

    // Install GPIO ISR service, required for pin interrupts
    gpio_install_isr_service(0);

    // --- 2. Configure Hardware ---
    gpio_reset_pin(ADS1256_1_CS_PIN);
    gpio_set_direction(ADS1256_1_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ADS1256_1_CS_PIN, 1);
    gpio_reset_pin(ADS1256_2_CS_PIN);
    gpio_set_direction(ADS1256_2_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ADS1256_2_CS_PIN, 1);

    spi_bus_config_t buscfg = {
        .mosi_io_num = ADS1256_MOSI_PIN,
        .miso_io_num = ADS1256_MISO_PIN,
        .sclk_io_num = ADS1256_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    // Initialize without DMA as it was faster for this use case
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg = {
        .mode = 1,
        .clock_speed_hz = 1800 * 1000, // Matching the clock speed from your app_main
        .spics_io_num = -1,
        .queue_size = 7,
    };
    spi_device_handle_t shared_spi_handle;
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &shared_spi_handle);
    ESP_ERROR_CHECK(ret);

    // --- 3. Initialize ADC Configs ---
    adc1_config = (ads1256_config_t){ .spi_handle = shared_spi_handle, .cs_pin = ADS1256_1_CS_PIN, .drdy_pin = ADS1256_1_DRDY_PIN, .reset_pin = ADS1256_1_RESET_PIN, .name = "ADC1", .drdy_sem = drdy_semaphore1 };
    adc2_config = (ads1256_config_t){ .spi_handle = shared_spi_handle, .cs_pin = ADS1256_2_CS_PIN, .drdy_pin = ADS1256_2_DRDY_PIN, .reset_pin = ADS1256_2_RESET_PIN, .name = "ADC2", .drdy_sem = drdy_semaphore2 };
    
    ADS1256_init(&adc1_config);
    ADS1256_init(&adc2_config);
    
    // --- 4. Create and Start Periodic Timer ---
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "adc_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SAMPLING_PERIOD_US));
    
    // --- 5. Create Dedicated ADC Task on Core 1 ---
    xTaskCreatePinnedToCore(
        adc_sampling_task,   // Function to implement the task
        "adc_sampling_task", // Name of the task
        4096,                // Stack size in words
        NULL,                // Task input parameter
        5,                   // Priority of the task
        NULL,                // Task handle
        1                    // Core where the task should run
    );

    ESP_LOGI(TAG, "ADC Initialization complete");
}

/**
 * @brief Starts the periodic ADC sampling timer.
 */
void start_adc_sampling(void) {
    if (periodic_timer == NULL) {
        ESP_LOGE(TAG, "Timer not initialized. Call init_adc_system() first.");
        return;
    }
    esp_err_t err = esp_timer_start_periodic(periodic_timer, SAMPLING_PERIOD_US);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ADC sampling timer started for %d Hz.", SAMPLING_FREQUENCY_HZ);
    } else {
        ESP_LOGE(TAG, "Failed to start ADC timer: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Stops the periodic ADC sampling timer.
 */
void stop_adc_sampling(void) {
    if (periodic_timer == NULL) {
        ESP_LOGE(TAG, "Timer not initialized.");
        return;
    }
    esp_err_t err = esp_timer_stop(periodic_timer);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ADC sampling timer stopped.");
    } else {
        ESP_LOGE(TAG, "Failed to stop ADC timer: %s", esp_err_to_name(err));
    }
}

//adc new
static void sendBatteryLevel(){
  #ifdef DEBUG_MODE
  ESP_LOGI(TAG, "Sending battery level over BLE!");
  #endif

  int batteryLevel = 0;
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
  batteryLevel = lipo.getSOC();
  xSemaphoreGive(i2c_mutex);
  }
  #ifdef DEBUG_MODE
  ESP_LOGI(TAG, "Battery level: %d", batteryLevel);
  #endif 
  
  if (pDataCharacteristic != NULL) {
    // NimBLE uses notify() with data directly, no setValue() method
    bool success = pDataCharacteristic->notify((uint8_t*)&batteryLevel, sizeof(batteryLevel));
    if (!success) {
      ESP_LOGW(TAG, "Failed to send battery level notification");
    }
  }
}

/**
 * Check if LED intensities are stored in NVS
 * Returns true if data exists, false otherwise
 */
bool areLEDIntensitiesInNVS(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    size_t required_size = 0;
    
    ESP_LOGI(TAG, "Checking NVS for LED intensities...");
    
    // Open NVS handle
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return false;
    }
    
    // Check if the blob exists by getting its size
    err = nvs_get_blob(my_handle, LED_INTENSITIES_KEY, NULL, &required_size);
    nvs_close(my_handle);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "LED intensities not found in NVS");
        return false;
    } else if (err == ESP_OK && required_size == sizeof(ledIntensities)) {
        ESP_LOGI(TAG, "LED intensities found in NVS");
        return true;
    } else {
        ESP_LOGE(TAG, "Error (%s) or size mismatch checking NVS!", esp_err_to_name(err));
        return false;
    }
}

/**
 * Save LED intensities array to NVS
 * Returns ESP_OK on success, error code otherwise
 */
esp_err_t saveLEDIntensitiesToNVS(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    
    ESP_LOGI(TAG, "Saving LED intensities to NVS...");
    
    // Open NVS handle
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    
    // Write LED intensities as a blob
    err = nvs_set_blob(my_handle, LED_INTENSITIES_KEY, ledIntensities, sizeof(ledIntensities));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) writing LED intensities!", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }
    
    // Commit changes
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) committing LED intensities!", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LED intensities saved successfully");
    }
    
    nvs_close(my_handle);
    return err;
}

/**
 * Load LED intensities array from NVS
 * Returns ESP_OK on success, error code otherwise
 */
esp_err_t loadLEDIntensitiesFromNVS(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    size_t required_size = sizeof(ledIntensities);
    
    ESP_LOGI(TAG, "Loading LED intensities from NVS...");
    
    // Open NVS handle
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    
    // Read LED intensities blob
    err = nvs_get_blob(my_handle, LED_INTENSITIES_KEY, ledIntensities, &required_size);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "LED intensities not found in NVS!");
        } else {
            ESP_LOGE(TAG, "Error (%s) reading LED intensities!", esp_err_to_name(err));
        }
    } else if (required_size != sizeof(ledIntensities)) {
        ESP_LOGE(TAG, "Size mismatch: expected %d bytes, got %d bytes", 
                 sizeof(ledIntensities), required_size);
        err = ESP_ERR_INVALID_SIZE;
    } else {
        ESP_LOGI(TAG, "LED intensities loaded successfully");
        
        #ifdef DEBUG_MODE
        // Print loaded values for debugging
        ESP_LOGI(TAG, "Loaded LED intensities:");
        for (int i = 0; i < NUM_LEDS; i++) {
            ESP_LOGI(TAG, "LED[%d] = %d", i, ledIntensities[i]);
        }
        #endif
    }
    
    nvs_close(my_handle);
    return err;
}

/**
 * Initialize default LED intensities
 */
void initDefaultLEDIntensities(void)
{
    ESP_LOGI(TAG, "Initializing default LED intensities");
    int defaultIntensities[NUM_SOURCES-1] = {100, 100, 100, 100,
                              100, 100, 100, 100,
                              100, 100, 100, 100,
                              100, 100, 100, 100,
                              50, 50, 50, 50,
                              50, 50, 50, 50,
                              50, 50, 50, 50,
                              50, 50, 50, 50
                              };
    memcpy(ledIntensities, defaultIntensities, sizeof(ledIntensities));
}

/**
 * Initialize NVS and LED intensities
 * Call this during system initialization (in app_main)
 */
esp_err_t initLEDIntensitiesNVS(void)
{
    esp_err_t err;
    
    ESP_LOGI(TAG, "Initializing NVS and LED intensities...");
    
    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) initializing NVS!", esp_err_to_name(err));
        return err;
    }
    
    // Check if LED intensities exist in NVS and load them
    if (areLEDIntensitiesInNVS()) {
        err = loadLEDIntensitiesFromNVS();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load LED intensities from NVS, using defaults");
            initDefaultLEDIntensities();
        }
    } else {
        ESP_LOGI(TAG, "No LED intensities found in NVS, using default values");
        initDefaultLEDIntensities();
        // Save default values to NVS for next boot
        saveLEDIntensitiesToNVS();
    }
    
    return ESP_OK;
}

/**
 * Send current LED intensities over BLE
 */
static void sendLEDIntensities(void)
{
    ESP_LOGI(TAG, "Sending LED intensities over BLE");
}

/**
 * Storage task for async NVS operations
 * This runs at low priority to avoid blocking BLE operations
 */
void storage_task(void *pvParameters) {
    int command;
    ESP_LOGI(TAG, "Storage task started");
    
    for (;;) {
        // Wait for commands from the queue
        if (xQueueReceive(storageQueue, &command, portMAX_DELAY)) {
            #ifdef DEBUG_MODE
            ESP_LOGI(TAG, "Storage Task: Received command %d", command);
            #endif
            
            switch (command) {
                case 1: // Save LED intensities
                    {
                        esp_err_t err = saveLEDIntensitiesToNVS();
                        if (err == ESP_OK) {
                            ESP_LOGI(TAG, "Storage Task: LED intensities saved successfully");
                        } else {
                            ESP_LOGE(TAG, "Storage Task: Failed to save LED intensities");
                        }
                    }
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Storage Task: Unknown command %d", command);
                    break;
            }
        }
    }
}

// --- Callback Class Definitions ---
class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        ESP_LOGI(TAG, "Client Connected");
        // Log the negotiated MTU size. This is crucial for debugging data transmission.
        ESP_LOGI(TAG, "Negotiated MTU: %d bytes", connInfo.getMTU());
        deviceConnected = true;
        readDataBool = false;
        // Set to GREEN to indicate a successful connection.
        setLedColor(0, 255, 0); // Solid Green
    };

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        ESP_LOGI(TAG, "Client Disconnected - reason: %d", reason);
        deviceConnected = false;
        // Set to CYAN/TEAL to indicate ready to connect again.
        setLedColor(0, 255, 255); // Solid Cyan/Teal
        NimBLEDevice::startAdvertising();
    }
} myServerCallbacks;

class LEDCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() == 0) {
            ESP_LOGW(TAG, "LEDCallback: Received empty payload");
            return;
        }

        // The first byte is the command, the rest is the data payload
        uint8_t commandCode = rxValue[0];

        #ifdef DEBUG_MODE
        ESP_LOGI(TAG, "LEDCallback: Received command code %d with %d bytes of data", 
                 commandCode, rxValue.length() - 1);
        #endif

        // Update LED intensities if data is provided
        if (rxValue.length() > 1) {
            // Determine how many LED values we actually received
            size_t numLedsReceived = rxValue.length() - 1;
            // Determine the maximum number of LEDs we can update in our array
            size_t numLedsToUpdate = std::min(numLedsReceived, (size_t)NUM_LEDS);

            // CORRECT WAY: Iterate through the payload and assign values one by one.
            // This properly handles the conversion from an 8-bit byte to a 32-bit int.
            for (size_t i = 0; i < numLedsToUpdate; ++i) {
                // The LED intensity data starts at the second byte (index 1) of the payload
                ledIntensities[i] = (uint8_t)rxValue[i + 1];
            }
            
            #ifdef DEBUG_MODE
            ESP_LOGI(TAG, "LEDCallback: Updated %d LED intensity values.", numLedsToUpdate);
            // Optional: Print the first few values to confirm they are correct now
            ESP_LOGI(TAG, "New intensities: LED[0]=%d, LED[1]=%d, LED[2]=%d, LED[3]=%d", 
                     ledIntensities[0], ledIntensities[1], ledIntensities[2], ledIntensities[3]);
            #endif
        }

        // Handle commands (This part remains the same)
        switch (commandCode) {
            case 1: // START DATA ACQUISITION
                #ifdef DEBUG_MODE
                ESP_LOGI(TAG, "Command: START DATA ACQUISITION");
                #endif
                readDataBool = true; 
                // Queue async save operation to avoid blocking BLE
                {
                    int cmd = 1;
                    if (xQueueSend(storageQueue, &cmd, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Failed to queue save operation");
                    }
                }
                start_adc_sampling();
                sendBatteryLevel();
                break;

            case 3: // STOP DATA ACQUISITION
                ESP_LOGI(TAG, "Command: STOP DATA ACQUISITION");
                readDataBool = false;
                
                if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                    // Turn off DAC
                    mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
                    xSemaphoreGive(i2c_mutex);
                }
                
                // Queue async save operation
                {
                    int cmd = 1;
                    if (xQueueSend(storageQueue, &cmd, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Failed to queue save operation");
                    }
                }
                stop_adc_sampling();
                sendBatteryLevel();
                break;

            case 5: // SEND CURRENT LED SETTINGS
                ESP_LOGI(TAG, "Command: SEND CURRENT LED SETTINGS");
                readDataBool = false;
                sendLEDIntensities();
                break;

            default:
                ESP_LOGW(TAG, "LEDCallback: Unknown command code %d received", commandCode);
                break;
        }
    }
}ledCallbacks;


// --- Initialization Functions ---
static void initBLE() {
    ESP_LOGI(TAG, "Initializing BLE...");
    
    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setMTU(517); // Set max MTU size for large data packets

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&myServerCallbacks);

    NimBLEService *pfNIRSService = pServer->createService(FNIRS_SERVICE_UUID);

    pDataCharacteristic = pfNIRSService->createCharacteristic(
                                   DATA_CHARACTERISTIC_UUID,
                                   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                   NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    
    pDataCharacteristic2 = pfNIRSService->createCharacteristic(
                                   DATA_CHARACTERISTIC_UUID2,
                                   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                   NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);

    pLEDCharacteristic = pfNIRSService->createCharacteristic(
                                   LED_CHARACTERISTIC_UUID,
                                   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE); 
    
    pLEDCharacteristic->setCallbacks(&ledCallbacks);

    pfNIRSService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData advert;
    advert.setFlags(0x06);
    advert.setName(DEVICE_NAME);
    advert.addServiceUUID(FNIRS_SERVICE_UUID);
    
    pAdvertising->setAdvertisementData(advert);
    pAdvertising->enableScanResponse(false);
    pAdvertising->start();

    // Ready to conect - BLE Initialized
    setLedColor(0, 255, 255); // Solid Cyan/Teal
    #ifdef DEBUG_MODE
    ESP_LOGI(TAG, "BLE initialized and advertising started");
    #endif
}

static void gpio_init(void) {
   ESP_LOGI(TAG, "Initializing GPIO...");
   // Initialize LED pins
   pinMode(PIN_LED_BLUE, OUTPUT); 
   pinMode(PIN_LED_GREEN, OUTPUT);
   pinMode(PIN_LED_RED, OUTPUT);

   // --- MODIFY THIS SECTION ---
   // Start with YELLOW to indicate booting up.
   setLedColor(255, 255, 0); // Solid Yellow

   // Initialize the MUX
   pinMode(s0, OUTPUT);
   pinMode(s1, OUTPUT);
   pinMode(s2, OUTPUT);
   pinMode(s3, OUTPUT);
   pinMode(MUX_EN_PIN, OUTPUT);
   pinMode(ADS1256_1_CS_PIN, OUTPUT);
   pinMode(ADS1256_1_DRDY_PIN, INPUT);
   pinMode(ADS1256_1_RESET_PIN, OUTPUT);
   pinMode(ADS1256_2_CS_PIN, OUTPUT);
   pinMode(ADS1256_2_DRDY_PIN, INPUT);
   pinMode(ADS1256_2_RESET_PIN, OUTPUT);
}

static void initStorageTask(void) {
    ESP_LOGI(TAG, "Creating storage task and queue...");
    
    // Create queue for storage operations
    storageQueue = xQueueCreate(5, sizeof(int));
    if (storageQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create storage queue");
        return;
    }
    
    // Create storage task with low priority
    BaseType_t result = xTaskCreate(storage_task, "storage_task", 4096, NULL, 1, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create storage task");
    } else {
        #ifdef DEBUG_MODE
        ESP_LOGI(TAG, "Storage task created successfully");
        #endif
    }
}

static void initBatteryFuelGauge(){
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        // Set up the MAX17044 LiPo fuel gauge:
        if (lipo.begin() == false) // Connect to the MAX17044 using the default wire port
        {
            ESP_LOGI(TAG, "MAX17044 not detected. Please check wiring. Freezing.");
            while (1) {
            vTaskDelay(pdMS_TO_TICKS(10)); 
            }
        }

        // Quick start restarts the MAX17044 in hopes of getting a more accurate
        // guess for the SOC.
        lipo.quickStart();

        // We can set an interrupt to alert when the battery SoC gets too low.
        // We can alert at anywhere between 1% - 32%:
        lipo.setThreshold(20); // Set alert threshold to 20%.

        xSemaphoreGive(i2c_mutex);

    }
    #ifdef DEBUG_MODE
    ESP_LOGI(TAG, "MAX17044 detected. Success");
    #endif
}


static void initDACs(){
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    // Try to initialize!
    if (!mcp.begin(0x60)) {
    // if (!mcp.begin()) {
        ESP_LOGI(TAG, "Failed to find MCP4728 chip");
        while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); 
        }
    }
    xSemaphoreGive(i2c_mutex);
  }
  else{
    #ifdef DEBUG_MODE
    ESP_LOGI(TAG, "DAC init successful!");
    #endif
  }
}

// --- BLE BROADCAST TASK ---
/**
 * @brief Sends a data snippet over the specified BLE characteristic.
 */
static void sendDataViaBLE(NimBLECharacteristic* pCharacteristic, uint8_t* data, size_t size) {
   if (deviceConnected && pCharacteristic != nullptr) {
        // notify() returns true on success.
        if (pCharacteristic->notify(data, size)) {
            #ifdef DEBUG_MODE
            ESP_LOGI(TAG, "Notification sent successfully. Size: %d bytes", size);
            #endif
        } else {
            ESP_LOGE(TAG, "Failed to send notification. Size: %d bytes. Is client subscribed?", size);
            // This can happen if the client is not subscribed, or if the internal buffer is full.
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Increased delay slightly for better reliability
    }
}

/**
 * @brief A dedicated task to broadcast fNIRS data in chunks over BLE.
 * This task blocks on a semaphore until a full data packet is ready to be sent.
 */
void ble_broadcast_task(void *pvParameters) {
    ESP_LOGI(TAG, "BLE broadcast task started and waiting for data.");
    
    const int CHUNK_1_ELEMENTS = 17 * 7 + 1;
    const int CHUNK_2_ELEMENTS = 17 * 5 + 1;
    int32_t dataPacketSnippet[CHUNK_1_ELEMENTS]; // Sized to the largest chunk

    while(1) {
        // Wait indefinitely for the signal from the ADC task
        if(xSemaphoreTake(ble_semaphore, portMAX_DELAY) == pdTRUE) {
            
            // The semaphore was given, meaning data is ready.
            // Check if we are supposed to be sending data before proceeding.
            if (readDataBool && deviceConnected) {
                #ifdef DEBUG_MODE
                ESP_LOGI(TAG, "Data packet ready. Starting broadcast...");
                #endif
                
                // --- Send chunk 1 ---
                dataPacketSnippet[0] = 1;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[0], (CHUNK_1_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic, (uint8_t*)dataPacketSnippet, CHUNK_1_ELEMENTS * sizeof(int32_t));
                
                // --- Send chunk 2 ---
                dataPacketSnippet[0] = 2;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[7 * 17], (CHUNK_1_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic2, (uint8_t*)dataPacketSnippet, CHUNK_1_ELEMENTS * sizeof(int32_t));

                // --- Send chunk 3 ---
                dataPacketSnippet[0] = 3;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[14 * 17], (CHUNK_1_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic2, (uint8_t*)dataPacketSnippet, CHUNK_1_ELEMENTS * sizeof(int32_t));

                // --- Send chunk 4 ---
                dataPacketSnippet[0] = 4;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[21 * 17], (CHUNK_1_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic2, (uint8_t*)dataPacketSnippet, CHUNK_1_ELEMENTS * sizeof(int32_t));

                // --- Send chunk 5 (final, smaller chunk) ---
                dataPacketSnippet[0] = 5;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[28 * 17], (CHUNK_2_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic, (uint8_t*)dataPacketSnippet, CHUNK_2_ELEMENTS * sizeof(int32_t));
                #ifdef DEBUG_MODE
                ESP_LOGI(TAG, "Full data packet sent. Task will now wait for the next signal.");
                #endif
            } else {
                 #ifdef DEBUG_MODE
                 ESP_LOGI(TAG, "BLE semaphore received, but data acquisition is stopped or device disconnected. Ignoring.");
                 #endif
            }
        }
    }
}

/**
 * @brief A non-interfering, low-priority task to monitor battery status.
 *
 * This task runs periodically. It only takes control of the LED if the battery
 * is low or critical WHILE data acquisition is active. Otherwise, it does nothing,
 * allowing other parts of the program to control the LED.
 */
void led_battery_status_task(void *pvParameters) {
    #ifdef DEBUG_MODE
    ESP_LOGI(TAG, "LED Battery Status task started.");
    #endif

    // An initial delay to let the system stabilize
    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1) {
        // --- Loop periodically. A 5-second check is plenty. ---
        vTaskDelay(pdMS_TO_TICKS(5000));

        // --- Only run checks if data acquisition is active ---
        if (!readDataBool) {
            continue; // Do nothing if not acquiring data
        }

        int batteryLevel = 0;
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            batteryLevel = lipo.getSOC();
            xSemaphoreGive(i2c_mutex);
        } else {
            continue; // Couldn't get mutex, try again next cycle
        }

        // --- STATE 1: Critical Battery ---
        if (batteryLevel <= CRITICAL_BATT_THRESHOLD) {
            #ifdef DEBUG_MODE
            ESP_LOGE(TAG, "Battery critically low (%d%%)! Halting operations.", batteryLevel);
            #endif

            // Stop all data acquisition
            readDataBool = false;
            stop_adc_sampling();
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                mcp.setChannelValue(MCP4728_CHANNEL_A, 0); // Turn off DAC
                xSemaphoreGive(i2c_mutex);
            }

            // Enter a blocking, pulsing red light loop to indicate total failure.
            // The device will need to be restarted.
            while (1) {
                setLedColor(255, 0, 0); // Red ON
                vTaskDelay(pdMS_TO_TICKS(400));
                setLedColor(0, 0, 0); // LED OFF
                vTaskDelay(pdMS_TO_TICKS(400));
            }
        }
        // --- STATE 2: Low Battery ---
        else if (batteryLevel <= LOW_BATT_THRESHOLD) {
            #ifdef DEBUG_MODE
            ESP_LOGW(TAG, "Battery low (%d%%). Pulsing orange warning.", batteryLevel);
            #endif

            // This loop will "steal" control of the LED to pulse orange.
            // It will continue as long as the battery is low AND we're still acquiring data.
            while (readDataBool && batteryLevel <= LOW_BATT_THRESHOLD) {
                setRGB(255, 165, 0); // Orange ON
                vTaskDelay(pdMS_TO_TICKS(750));
                
                // Check if we should still be pulsing before turning the LED off
                if (!readDataBool) break;

                setLedColor(0, 0, 0); // LED OFF
                vTaskDelay(pdMS_TO_TICKS(750));

                // Re-check battery level to see if we should exit the loop
                if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    batteryLevel = lipo.getSOC();
                    xSemaphoreGive(i2c_mutex);
                }
            }
            
            // After the loop finishes (either by charging or stopping acquisition),
            // restore the LED to green if we are still operating.
            if(readDataBool){
                setLedColor(0, 255, 0);
            }
        }
    }
}

// --- Main Application ---
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting NIRduino application...");
    
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
    }

    // Initialize Arduino framework
    initArduino();

    // Create the semaphore for the BLE broadcast task
    ble_semaphore = xSemaphoreCreateBinary();
    if (ble_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create BLE semaphore");
        return;
    }

    // Initialize GPIO pins
    gpio_init();

    Wire.begin(I2C_SDA, I2C_SCL);

    initDACs();

    initBatteryFuelGauge();
    
    // Initialize NVS and load LED intensities
    esp_err_t err = initLEDIntensitiesNVS();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(err));
        // Continue anyway with default values
    }
    
    // Initialize storage task and queue
    initStorageTask();
    
    // Initialize BLE
    initBLE();

    init_adc_system();

    // --- Create and start the new LED battery status task ---
    xTaskCreate(
        led_battery_status_task,    // Function that implements the task.
        "led_battery_task",         // Text name for the task.
        2048,                       // Stack size in words.
        NULL,                       // Parameter passed into the task.
        2,                          // Low priority.
        NULL                        // Task handle.
    );

    // --- Create and start the new BLE broadcast task ---
    xTaskCreate(
        ble_broadcast_task,       // Function that implements the task.
        "ble_broadcast_task",     // Text name for the task.
        4096,                     // Stack size in words.
        NULL,                     // Parameter passed into the task.
        4,                        // Priority at which the task is created.
        NULL                      // Used to pass out the created task's handle.
    );
    
    #ifdef DEBUG_MODE
    ESP_LOGI(TAG, "NIRduino initialization complete");
    #endif
}