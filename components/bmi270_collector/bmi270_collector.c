/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bmi270_collector.h"

#include <math.h>
#include <stdio.h>

#include "bmi270_api.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_board_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soccer_data_sync.h"

static const char *TAG = "bmi270_collector";

#define GPTIMER_RESOLUTION_HZ       1000000
#define ACC_COUNTS_PER_G            2048.0f
#define GYRO_LSB_PER_DPS            16.4f
#define I2C_MASTER_FREQ_HZ          1000000

#define COLLECT_TASK_STACK          4096
#define PRINT_TASK_STACK            3072
#define COLLECT_TASK_PRIO           (configMAX_PRIORITIES - 2)
#define PRINT_TASK_PRIO             (configMAX_PRIORITIES - 4)
#define SAMPLE_QUEUE_LEN            32
#define PRINT_QUEUE_LEN             16
#define CSV_LINE_MAX                96

#define I2C_MASTER_SDO_IO           GPIO_NUM_9

typedef struct {
    uint32_t tick_us;
} bmi270_sample_tick_t;

typedef struct {
    uint32_t timestamp_ms;
    float ax, ay, az;
    float gx, gy, gz;
    float pitch, roll;
} bmi270_csv_sample_t;

static bool s_active = false;
static i2c_bus_handle_t s_i2c_bus = NULL;
static bmi270_handle_t s_bmi_handle = NULL;
static struct bmi2_dev *s_bmi_dev = NULL;

static gptimer_handle_t s_gptimer = NULL;
static QueueHandle_t s_sample_queue = NULL;
static QueueHandle_t s_print_queue = NULL;
static TaskHandle_t s_collect_task = NULL;
static TaskHandle_t s_print_task = NULL;
static SemaphoreHandle_t s_i2c_mutex = NULL;
static bmi270_csv_line_sink_t s_csv_sink = NULL;
static void *s_csv_sink_user_data = NULL;

static volatile uint32_t s_isr_enqueued = 0;
static volatile uint32_t s_isr_dropped = 0;
static volatile uint32_t s_print_dropped = 0;

static bool init_bmi270(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_MASTER_SDO_IO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGE(TAG, "SDO gpio config failed");
        return false;
    }
    gpio_set_level(I2C_MASTER_SDO_IO, 0);

    i2c_master_bus_config_t *bm_i2c = NULL;
    if (esp_board_manager_get_periph_config("i2c_master", (void **)&bm_i2c) != ESP_OK || bm_i2c == NULL) {
        ESP_LOGE(TAG, "get i2c_master config failed");
        return false;
    }

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = bm_i2c->sda_io_num,
        .scl_io_num = bm_i2c->scl_io_num,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = I2C_MASTER_FREQ_HZ},
        .clk_flags = 0,
    };

    s_i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);
    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "i2c_bus_create failed");
        return false;
    }

    esp_err_t ret = bmi270_sensor_create(
        s_i2c_bus, &s_bmi_handle, bmi270_config_file,
        BMI2_GYRO_CROSS_SENS_ENABLE | BMI2_CRT_RTOSK_ENABLE);
    if (ret != ESP_OK || s_bmi_handle == NULL) {
        ESP_LOGE(TAG, "bmi270_sensor_create failed");
        return false;
    }

    s_bmi_dev = (struct bmi2_dev *)s_bmi_handle;

    struct bmi2_sens_config config[2] = {
        {.type = BMI2_ACCEL},
        {.type = BMI2_GYRO},
    };

    int8_t rslt = bmi2_get_sensor_config(config, 2, s_bmi_dev);
    if (rslt == BMI2_OK) {
        config[BMI2_ACCEL].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
        config[BMI2_ACCEL].cfg.acc.range = BMI2_ACC_RANGE_16G;
        config[BMI2_ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
        config[BMI2_ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        config[BMI2_GYRO].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
        config[BMI2_GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
        config[BMI2_GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
        config[BMI2_GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
        config[BMI2_GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        rslt = bmi270_set_sensor_config(config, 2, s_bmi_dev);
        if (rslt != BMI2_OK) {
            ESP_LOGE(TAG, "bmi270_set_sensor_config failed");
            return false;
        }
    }

    uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    rslt = bmi2_sensor_enable(sens_list, 2, s_bmi_dev);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "bmi2_sensor_enable failed");
        return false;
    }

    ESP_LOGI(TAG, "BMI270 ODR %d Hz, I2C %d kHz", BMI270_COLLECT_HZ, I2C_MASTER_FREQ_HZ / 1000);
    return true;
}

static bool IRAM_ATTR gptimer_alarm_isr(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    bmi270_sample_tick_t tick = {
        .tick_us = (uint32_t)esp_timer_get_time(),
    };

    BaseType_t wake = pdFALSE;
    if (xQueueSendFromISR(s_sample_queue, &tick, &wake) != pdTRUE) {
        s_isr_dropped++;
    } else {
        s_isr_enqueued++;
    }
    return wake;
}

static bool start_gptimer(void)
{
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = GPTIMER_RESOLUTION_HZ,
    };
    if (gptimer_new_timer(&timer_cfg, &s_gptimer) != ESP_OK) {
        ESP_LOGE(TAG, "gptimer_new_timer failed");
        return false;
    }

    gptimer_alarm_config_t alarm_cfg = {
        .reload_count = 0,
        .alarm_count = BMI270_COLLECT_PERIOD_US,
        .flags.auto_reload_on_alarm = true,
    };
    if (gptimer_set_alarm_action(s_gptimer, &alarm_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "gptimer_set_alarm_action failed");
        return false;
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_alarm_isr,
    };
    if (gptimer_register_event_callbacks(s_gptimer, &cbs, NULL) != ESP_OK ||
        gptimer_enable(s_gptimer) != ESP_OK ||
        gptimer_start(s_gptimer) != ESP_OK) {
        ESP_LOGE(TAG, "gptimer start failed");
        return false;
    }

    ESP_LOGI(TAG, "GPTimer %d Hz, period %d us", BMI270_COLLECT_HZ, BMI270_COLLECT_PERIOD_US);
    return true;
}

static void bmi270_print_task(void *arg)
{
    (void)arg;
    bmi270_csv_sample_t sample;
    char line[CSV_LINE_MAX];

    ESP_LOGI(TAG, "bmi270_print_task started");

    while (s_active) {
        if (xQueueReceive(s_print_queue, &sample, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_active) {
            break;
        }

        float acc_total = sqrtf(sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
        int len = snprintf(line, sizeof(line),
                           "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f\n",
                           (unsigned long)sample.timestamp_ms,
                           sample.ax, sample.ay, sample.az, acc_total,
                           sample.gx, sample.gy, sample.gz,
                           sample.pitch, sample.roll);
        if (len > 0) {
            bool delivered = false;
            if (s_csv_sink != NULL) {
                delivered = s_csv_sink(line, (size_t)len, s_csv_sink_user_data);
            }
            if (!delivered) {
                fwrite(line, 1, (size_t)len, stdout);
            }
        }
    }

    s_print_task = NULL;
    vTaskDelete(NULL);
}

static void bmi270_collect_task(void *arg)
{
    (void)arg;
    bmi270_sample_tick_t tick;
    uint32_t sample_count = 0;
    uint32_t last_log_ms = 0;

    ESP_LOGI(TAG, "bmi270_collect_task started (prio %d)", COLLECT_TASK_PRIO);

    while (s_active) {
        if (xQueueReceive(s_sample_queue, &tick, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_active) {
            break;
        }

        struct bmi2_sens_data raw = {0};

        if (xSemaphoreTake(s_i2c_mutex, portMAX_DELAY) == pdTRUE) {
            int8_t rslt = bmi2_get_sensor_data(&raw, s_bmi_dev);
            xSemaphoreGive(s_i2c_mutex);
            if (rslt != BMI2_OK) {
                continue;
            }
        } else {
            continue;
        }

        bmi270_csv_sample_t sample = {
            .timestamp_ms = tick.tick_us / 1000U,
            .ax = (float)raw.acc.x / ACC_COUNTS_PER_G,
            .ay = -(float)raw.acc.y / ACC_COUNTS_PER_G,
            .az = -(float)raw.acc.z / ACC_COUNTS_PER_G,
            .gx = (float)raw.gyr.x / GYRO_LSB_PER_DPS,
            .gy = -(float)raw.gyr.y / GYRO_LSB_PER_DPS,
            .gz = -(float)raw.gyr.z / GYRO_LSB_PER_DPS,
            .pitch = g_soccer_sensor_data.pitch,
            .roll = g_soccer_sensor_data.roll,
        };

        g_soccer_sensor_data.acc[0] = sample.ax;
        g_soccer_sensor_data.acc[1] = sample.ay;
        g_soccer_sensor_data.acc[2] = sample.az;
        g_soccer_sensor_data.gyro[0] = sample.gx;
        g_soccer_sensor_data.gyro[1] = sample.gy;
        g_soccer_sensor_data.gyro[2] = sample.gz;
        g_soccer_sensor_data.timestamp = sample.timestamp_ms;

        if (xQueueSend(s_print_queue, &sample, 0) != pdTRUE) {
            s_print_dropped++;
        }

        sample_count++;

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms - last_log_ms >= 5000) {
            ESP_LOGI(TAG, "samples=%lu isr_ok=%lu isr_drop=%lu print_drop=%lu",
                     (unsigned long)sample_count,
                     (unsigned long)s_isr_enqueued,
                     (unsigned long)s_isr_dropped,
                     (unsigned long)s_print_dropped);
            last_log_ms = now_ms;
        }
    }

    s_collect_task = NULL;
    vTaskDelete(NULL);
}

bool bmi270_collector_start(void)
{
    if (s_active) {
        return true;
    }

    s_isr_enqueued = 0;
    s_isr_dropped = 0;
    s_print_dropped = 0;

    if (!init_bmi270()) {
        bmi270_collector_stop();
        return false;
    }

    s_sample_queue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(bmi270_sample_tick_t));
    s_print_queue = xQueueCreate(PRINT_QUEUE_LEN, sizeof(bmi270_csv_sample_t));
    if (s_sample_queue == NULL || s_print_queue == NULL) {
        ESP_LOGE(TAG, "queue create failed");
        bmi270_collector_stop();
        return false;
    }

    s_i2c_mutex = xSemaphoreCreateMutex();
    if (s_i2c_mutex == NULL) {
        ESP_LOGE(TAG, "i2c mutex create failed");
        bmi270_collector_stop();
        return false;
    }

    s_active = true;

    if (xTaskCreate(bmi270_print_task, "bmi270_print", PRINT_TASK_STACK, NULL,
                    PRINT_TASK_PRIO, &s_print_task) != pdPASS) {
        ESP_LOGE(TAG, "print task create failed");
        bmi270_collector_stop();
        return false;
    }

    if (xTaskCreate(bmi270_collect_task, "bmi270_collect", COLLECT_TASK_STACK, NULL,
                    COLLECT_TASK_PRIO, &s_collect_task) != pdPASS) {
        ESP_LOGE(TAG, "collect task create failed");
        bmi270_collector_stop();
        return false;
    }

    if (!start_gptimer()) {
        bmi270_collector_stop();
        return false;
    }

    ESP_LOGI(TAG, "collector running @ %d Hz (GPTimer %d us)", BMI270_COLLECT_HZ, BMI270_COLLECT_PERIOD_US);
    return true;
}

void bmi270_collector_stop(void)
{
    if (!s_active && s_gptimer == NULL && s_bmi_handle == NULL) {
        return;
    }

    s_active = false;

    if (s_gptimer) {
        gptimer_stop(s_gptimer);
        gptimer_disable(s_gptimer);
        gptimer_del_timer(s_gptimer);
        s_gptimer = NULL;
    }

    if (s_sample_queue) {
        bmi270_sample_tick_t sentinel = {.tick_us = 0};
        xQueueSend(s_sample_queue, &sentinel, pdMS_TO_TICKS(100));
    }
    if (s_print_queue) {
        bmi270_csv_sample_t ps = {0};
        xQueueSend(s_print_queue, &ps, pdMS_TO_TICKS(100));
    }

    for (int i = 0; i < 50 && (s_collect_task != NULL || s_print_task != NULL); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (s_sample_queue) {
        vQueueDelete(s_sample_queue);
        s_sample_queue = NULL;
    }
    if (s_print_queue) {
        vQueueDelete(s_print_queue);
        s_print_queue = NULL;
    }

    if (s_i2c_mutex) {
        vSemaphoreDelete(s_i2c_mutex);
        s_i2c_mutex = NULL;
    }

    if (s_bmi_handle) {
        bmi270_sensor_del(&s_bmi_handle);
        s_bmi_handle = NULL;
        s_bmi_dev = NULL;
    }

    s_i2c_bus = NULL;
}

bool bmi270_collector_is_active(void)
{
    return s_active;
}

i2c_bus_handle_t bmi270_collector_get_i2c_bus(void)
{
    return s_i2c_bus;
}

void bmi270_collector_i2c_lock(void)
{
    if (s_i2c_mutex) {
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
    }
}

void bmi270_collector_i2c_unlock(void)
{
    if (s_i2c_mutex) {
        xSemaphoreGive(s_i2c_mutex);
    }
}

void bmi270_collector_set_csv_sink(bmi270_csv_line_sink_t sink, void *user_data)
{
    s_csv_sink = sink;
    s_csv_sink_user_data = user_data;
}
