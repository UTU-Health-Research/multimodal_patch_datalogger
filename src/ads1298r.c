#include <string.h>
#include "ads1298r.h"
#include "config.h"
#include "data_types.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "i2c_sensors.h"

// ===== Pins from config.h =====
// ECG_MISO, ECG_MOSI, ECG_SCLK, ECG_CS, DRDY_PIN, START_PIN, PWDN_PIN, RESET_PIN

// ===== ADS1298R commands and registers =====
#define SDATAC_CMD   0x11
#define RDATAC_CMD   0x10
#define RDATA_CMD    0x12
#define WREG_CMD     0x40
#define RREG_CMD     0x20

#define ID_REG           0x00
#define CONFIG1_REG      0x01
#define CONFIG2_REG      0x02
#define CONFIG3_REG      0x03
#define LOFF_REG         0x04
#define CH1SET_REG       0x05
#define CH2SET_REG       0x06
#define CH3SET_REG       0x07
#define CH4SET_REG       0x08
#define CH5SET_REG       0x09
#define CH6SET_REG       0x0A
#define CH7SET_REG       0x0B
#define CH8SET_REG       0x0C
#define RLD_SENSP_REG    0x0D
#define RLD_SENSN_REG    0x0E
#define LOFF_SENSP_REG   0x0F
#define LOFF_SENSN_REG   0x10
#define LOFF_FLIP_REG    0x11
#define GPIO_REG         0x14
#define PACE_REG         0x15
#define RESP_REG         0x16
#define CONFIG4_REG      0x17
#define WCT1_REG         0x18
#define WCT2_REG         0x19

#define FRAME_SIZE       FRAME_SIZE_BYTES // 27 bytes: 3 status + 8*3

static const char *TAG = "ads1298r";

static spi_device_handle_t s_dev;
static QueueHandle_t s_outQ;
static SemaphoreHandle_t s_drdy_sem;

// ===== Helpers =====
static inline int32_t sext24(uint8_t b0, uint8_t b1, uint8_t b2) {
    int32_t v = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | b2;
    return (v & 0x00800000) ? (v | 0xFF000000) : v;
}

// Gains/reference setup
static const float CHANNEL_GAIN[ECG_CHANNELS] = {4.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0};
static const float BASE_FACTOR = 2400.0f / 8388607.0f; // 2.4V ref in mV / (2^23 -1)

static void parse_ecg_frame_to_sample(const uint8_t *frame, pkt_sample_t *out) {
    out->timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    for (int ch = 0; ch < ECG_CHANNELS; ch++) {
        int off = 3 + ch * 3;
        int32_t raw = sext24(frame[off], frame[off + 1], frame[off + 2]);
        out->ecg[ch] = (float)raw * (BASE_FACTOR / CHANNEL_GAIN[ch]);
    }
}

// Send a single-byte command (needs manually driven CS before and after function call)
static inline void ads_send_cmd(uint8_t cmd) {

    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_dev, &t));

}

// Write N registers starting at start_addr
static inline void ads_write_regs(uint8_t start_addr, const uint8_t *values, uint8_t num_regs) {
    // Stop conversions before register access
    gpio_set_level(ECG_CS, 0);
    esp_rom_delay_us(4);

    ads_send_cmd(SDATAC_CMD);
    gpio_set_level(ECG_CS, 1);
    esp_rom_delay_us(4);
    
    uint8_t hdr[2] = { (uint8_t)(WREG_CMD | start_addr), (uint8_t)(num_regs - 1) };

    gpio_set_level(ECG_CS, 0);
    esp_rom_delay_us(4);

    spi_transaction_t t1 = { .length = 16, .tx_buffer = hdr };
    spi_transaction_t t2 = { .length = 8 * num_regs, .tx_buffer = values };
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_dev, &t1));
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_dev, &t2));
    
    gpio_set_level(ECG_CS, 1);
    esp_rom_delay_us(4);
}

// Read one register (useful for ID)
static inline uint8_t ads_read_reg(uint8_t addr) {
    gpio_set_level(ECG_CS, 0);
    esp_rom_delay_us(4);

    ads_send_cmd(RREG_CMD | addr);
    ads_send_cmd(0x00);

    uint8_t rx = 0, dummy = 0x00;
    spi_transaction_t t2 = { .length = 8, .rxlength = 8, .tx_buffer = &dummy, .rx_buffer = &rx };
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_dev, &t2));
    gpio_set_level(ECG_CS, 1);
    esp_rom_delay_us(4);
    return rx;
}

static void init_ads1298r_registers(void) {

    // Send SDATAC 
    gpio_set_level(ECG_CS, 0);
    esp_rom_delay_us(4);
    ads_send_cmd(SDATAC_CMD);
    gpio_set_level(ECG_CS, 1);
    esp_rom_delay_us(4);

    // Optional: check ID
    uint8_t id = ads_read_reg(ID_REG);
    ESP_LOGI(TAG, "ID=0x%02X", id);

    // Register batches
    const uint8_t batch1[] = {
        0x86, // CONFIG1: 500 SPS
        0x02, // CONFIG2
        0xCD, // CONFIG3
        0x00, // LOFF
        0x10, // CH1SET 0100 0000 used to be 0x10 to be turned on for resp, now turned off with 0x91
        0x50, // CH2SET 0x50 for high gain lead 1
        0x50, // CH3SET 0x50 for high gain lead 2
        0x00, // CH4SET
        0x00, // CH5SET
        0x00, // CH6SET
        0x00, // CH7SET
        0x00, // CH8SET
        0x06, // RLD_SENSP RLDP derived from IN2P LA and IN3P LL
        0x02, // RLD_SENSN RLDN derived from IN2N RA 
        0x00, // LOFF_SENSP
        0x00, // LOFF_SENSN
    };
    ads_write_regs(CONFIG1_REG, batch1, sizeof(batch1));

    const uint8_t batch2[] = {
        0x0F, // GPIO
        0x00, // PACE
        0xF2, // RESP //0xF2 for respiration measurement turned ON. 0x20 for turned off
        0x20, // CONFIG4 0x20 for 32kHz resp modulation or 0x00 for 64kHz modulation
        0x0B, // WCT1
        0xD4, // WCT2
    };
    ads_write_regs(GPIO_REG, batch2, sizeof(batch2));

    // Start conversions in RDATAC
    gpio_set_level(START_PIN, 1);

    gpio_set_level(ECG_CS, 0);
    esp_rom_delay_us(4);
    ads_send_cmd(RDATAC_CMD);
    gpio_set_level(ECG_CS, 1);
    esp_rom_delay_us(4);

    //gpio_set_level(PWDN_PIN, 1);
}

// ===== DRDY ISR and acquisition task =====
static void IRAM_ATTR drdy_isr(void *arg) {
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_drdy_sem, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void ads_task(void *arg) {
    // DMA-capable RX ping-pong buffers
    uint8_t *rxA = heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_DMA);
    uint8_t *rxB = heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_DMA);
    static uint8_t zeros[FRAME_SIZE] = {0}; // clocks out RDATAC frame
    if (!rxA || !rxB) {
        ESP_LOGE(TAG, "DMA alloc failed");
        vTaskDelete(NULL);
        return;
    }
    bool useA = true;

    for (;;) {
        xSemaphoreTake(s_drdy_sem, portMAX_DELAY);

        uint8_t *rx = useA ? rxA : rxB;
        useA = !useA;

        gpio_set_level(ECG_CS, 0);
        esp_rom_delay_us(4);

        spi_transaction_t t = {
            .length = 8 * FRAME_SIZE,
            .tx_buffer = zeros,
            .rx_buffer = rx,
        };
        esp_err_t e = spi_device_polling_transmit(s_dev, &t);

        gpio_set_level(ECG_CS, 1);
        esp_rom_delay_us(4);

        if (e != ESP_OK) {
            ESP_LOGW(TAG, "SPI transmit: %s", esp_err_to_name(e));
            continue;
        }

        pkt_sample_t s;
        imu_sample_t imu1_data, imu2_data;
        float temp_data; 

        //esp_log_buffer_hex("ECG rx buffer", rx, 8*sizeof(rx)); //printing data from ADS1298R
        parse_ecg_frame_to_sample(rx, &s); 

         // Get latest IMU data
        sensors_get_latest_data(&imu1_data, &imu2_data, &temp_data);
        memcpy(&s.imu1, &imu1_data, sizeof(imu_sample_t));
        memcpy(&s.imu2, &imu2_data, sizeof(imu_sample_t));
        memcpy(&s.temp, &temp_data, sizeof(float));

        //esp_log_buffer_hex("ECG sample", &s.ch, 8*sizeof(&s.ch)); //printing data from sample
        (void)xQueueSend(s_outQ, &s, 0);
    }
}

void ads1298r_start(QueueHandle_t out_queue) {
    s_outQ = out_queue;

        // GPIO setup
    gpio_set_direction(GREEN_LED_ANODE, GPIO_MODE_OUTPUT);
    gpio_set_direction(RED_LED_ANODE, GPIO_MODE_OUTPUT);
    //gpio_set_level(GREEN_LED_ANODE, 1); // turn on green LED
    gpio_set_direction(ECG_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(ECG_CS, 1);
    gpio_set_direction(DRDY_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(START_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWDN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RESET_PIN, GPIO_MODE_OUTPUT);

    // Initialize control pins
    gpio_set_level(PWDN_PIN, 1);
    gpio_set_level(RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

        // Reset the chip
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));


    // SPI3 bus/device for ADS
    spi_bus_config_t buscfg = {
        .mosi_io_num = ECG_MOSI,
        .miso_io_num = ECG_MISO,
        .sclk_io_num = ECG_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = FRAME_SIZE,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // raise if stable
        .mode = 1,                          // CPHA=1
        .spics_io_num = -1,             // HW-controlled CS
        .queue_size = 2,
        .cs_ena_posttrans = 2,              // tiny CS hold time
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &s_dev));
    vTaskDelay(pdMS_TO_TICKS(100));

    // DRDY ISR
    s_drdy_sem = xSemaphoreCreateBinary();
    gpio_set_intr_type(DRDY_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(DRDY_PIN, drdy_isr, NULL);

        // Configure device
    init_ads1298r_registers();


    // Start acquisition task
    xTaskCreatePinnedToCore(ads_task, "ads_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 0);
}
