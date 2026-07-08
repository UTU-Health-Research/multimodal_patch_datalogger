// src/sd_logger.c
#include <string.h>
#include "esp_mac.h"
#include "sd_logger.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include <dirent.h>

static const char *TAG = "sd_logger";


static QueueHandle_t s_sd_queue = NULL;
static FILE *s_file = NULL;
static TaskHandle_t s_writer_task = NULL;
static char s_path[64];

static void sd_writer_task(void *arg) {
#if LOG_BINARY
    // no header
#else
    fputs("Timestamp,Ch1,Ch2,Ch3,Ch4,Ch5,Ch6,Ch7,Ch8\n", s_file);
#endif
    int64_t last_sync = esp_timer_get_time();


    for (;;) {
        batch_buf_t bb;
        if (xQueueReceive(s_sd_queue, &bb, portMAX_DELAY)) {
             // Print first few bytes of the batch buffer for inspection
            //ESP_LOG_BUFFER_HEX(TAG, bb.data, bb.len < 32 ? bb.len : 32);
            // if (!s_file) s_file = fopen("/sdcard/ecg_1.bin", "wb");
            size_t wr = fwrite(bb.data, 1, bb.len, s_file);

            if (wr != bb.len) {
                ESP_LOGE(TAG, "fwrite short: %u/%u", (unsigned)wr, (unsigned)bb.len);
            }
            
            free(bb.data); // allocated by pipeline
        }

        int64_t now = esp_timer_get_time();
        if (now - last_sync >= SYNC_INTERVAL_US) {

            fflush(s_file);
            
            fclose(s_file);

        ESP_LOGI(TAG, "SD sync at %lld us", now);
            last_sync = now;
            s_file = fopen(s_path,"ab");
            vTaskDelay(1);
        }
    }
}

static esp_err_t mount_sdcard(void) {
#if SD_USE_SDMMC
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = SDMMC_SLOT_WIDTH;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 32 * 1024
    };

    sdmmc_card_t *card;
    ESP_RETURN_ON_ERROR(esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card), TAG, "mount");
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
#else
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 20000; // tune

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS;
    slot_cfg.host_id = SPI2_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 32 * 1024,
    };

    sdmmc_card_t *card;
    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        // Try to clean up the SPI bus
        spi_bus_free(SPI2_HOST);
        return ret;
    }
    
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
#endif
}

static int get_next_file_index() {
    DIR *dir = opendir(SD_MOUNT_POINT);
    if (!dir) return 0;

    int max_index = 0;
    struct dirent *entry;
    while ((entry = readdir(dir))) {
        if (entry->d_type == DT_REG && 
            strncmp(entry->d_name, FILENAME_PREFIX, strlen(FILENAME_PREFIX)) == 0) {
            int num = atoi(entry->d_name + strlen(FILENAME_PREFIX));
            if (num > max_index) max_index = num;
        }
    }
    closedir(dir);
    return max_index + 1;
}

QueueHandle_t sd_logger_start() {

        // Try to mount SD card
    esp_err_t err = mount_sdcard();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed: %s. Continuing without SD logging.", esp_err_to_name(err));
        
        // Turn on the error LED
        gpio_set_level(RED_LED_ANODE, 1);
    
        
        return NULL;
    }

// FILE *test_file = fopen("/sdcard/testfile1.txt", "w");
// if (test_file) {
//     fputs("Hello, SD card!\n", test_file);
//     fclose(test_file);
//     ESP_LOGI(TAG, "Test file created successfully.");
// } else {
//     ESP_LOGE(TAG, "Failed to create test file.");
// }

    // Create a unique filename
     // Start SD logger (creates sd writer queue)
    snprintf(s_path, sizeof(s_path), "%s/" FILENAME_PREFIX "%04d" FILE_EXTENSION,
             SD_MOUNT_POINT, get_next_file_index()); //create unique filename based on last file index


#if LOG_BINARY
    s_file = fopen(s_path, "wb");
    taskYIELD();
    if(!s_file) s_file = fopen(s_path, "wb");
    taskYIELD();
#else
    s_file = fopen(path, "wb");
#endif
    if (!s_file) {
        ESP_LOGE(TAG, "open %s failed", s_path);
        return NULL;
    }
    else { 
        ESP_LOGI(TAG, "Opened %s for writing", s_path);
     }

    s_sd_queue = xQueueCreate(SD_QUEUE_LEN, sizeof(batch_buf_t));
    if (!s_sd_queue) {
        ESP_LOGE(TAG, "sd queue create failed");
        return NULL;
    }

    xTaskCreatePinnedToCore(sd_writer_task, "sd_writer", 4096, NULL, configMAX_PRIORITIES - 4, &s_writer_task, 1);
    return s_sd_queue;
}

void sd_logger_stop(void) {
    // For brevity: not implementing task teardown; typically signal task to exit.
    if (s_file) { fflush(s_file); fclose(s_file); s_file = NULL; }
    // Unmount if needed
}