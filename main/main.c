#include <stdio.h>
#include <inttypes.h>
#include "esp_err.h"
#include "hal/adc_types.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"

#define MUSCLE_ATTENUATION 1

static TaskHandle_t s_task_handle;

void init_adc(adc_continuous_handle_t *out_handle) {
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 256,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &handle));

    adc_digi_pattern_config_t controller_config = {
        .atten = MUSCLE_ATTENUATION,
        .channel = 1,
        .unit = ADC_UNIT_1,
        .bit_width = 12
    };

    adc_continuous_config_t config = {
        .pattern_num = 1,
        .adc_pattern = &controller_config,
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1
    };

    ESP_ERROR_CHECK(adc_continuous_config(handle, &config));

    *out_handle = handle;
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t must_yield = pdFALSE;

    vTaskNotifyGiveFromISR(s_task_handle, &must_yield);

    return must_yield == pdTRUE;
}

void app_main(void) {
    esp_err_t ret = 0;
    uint32_t ret_num = 0;
    uint8_t result[256] = {0};

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    init_adc(&handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    for(;;) {

        /* clearing task to be 0 after the function is called and waiting max delay for it to not be 0 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for(;;) {
            ret = adc_continuous_read(handle, result, 256, &ret_num, 0);
            if(ret == ESP_OK) {
                ESP_LOGI("TASK", "ret_num is %"PRIu32" bytes", ret_num);

                adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
                uint32_t num_parsed_samples = 0;

                esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
                if(parse_ret == ESP_OK) {
                    for(int i = 0; i < num_parsed_samples; i++) {
                        if(parsed_data[i].valid) {
                            ESP_LOGI("ADC", "ADC%d, Channel: %d, Value: %"PRIu32,
                                parsed_data[i].unit + 1,
                                parsed_data[i].channel,
                                parsed_data[i].raw_data);
                        } else {
                            ESP_LOGW("ADC", "Invalid data [ADC%d Channel: %d, Value: %"PRIu32"]",
                                parsed_data[i].unit + 1,
                                parsed_data[i].channel,
                                parsed_data[i].raw_data);
                        }
                    }
                } else {
                    ESP_LOGE("ADC", "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }

                // When we send data through wifi/bluetooth we'll prob need to remove this
                vTaskDelay(1);
            } else if(ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}

