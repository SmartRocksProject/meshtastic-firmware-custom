#include "INMP441Sensor.h"

#include <driver/i2s.h>

#define I2S_PORT I2S_NUM_0

bool INMP441Sensor::setup(uint32_t sampleRate, int bufferLen) {
    _bufferLen = bufferLen;

    // Set up I2S
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sampleRate,
        .bits_per_sample = i2s_bits_per_sample_t(16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = _bufferLen,
        .use_apll = false
    };

    if(i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
        return false;
    }

    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };

    if(i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
        return false;
    }

    if(i2s_start(I2S_PORT) != ESP_OK) {
        return false;
    }

    return true;
}

size_t INMP441Sensor::readSamples(int16_t* samples) {
    size_t bytesIn = 0;
    esp_err_t result = i2s_read(I2S_PORT, samples, _bufferLen * sizeof(int16_t), &bytesIn, portMAX_DELAY); // Wait 90 ms max for data

    if(result == ESP_OK) {
        int16_t samples_read = bytesIn / sizeof(int16_t);
        return samples_read;
    }
    return 0;
}
