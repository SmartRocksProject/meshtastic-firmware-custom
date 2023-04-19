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
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = _bufferLen,
        .use_apll = false
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };

    i2s_set_pin(I2S_PORT, &pin_config);

    i2s_start(I2S_PORT);

    return true;
}

void INMP441Sensor::readSamples(int16_t* samples) {
    size_t bytesIn = 0;
    int32_t sBuffer[64];
    esp_err_t result = i2s_read(I2S_PORT, &sBuffer, _bufferLen, &bytesIn, pdMS_TO_TICKS(90)); // Wait 90 ms max for data

    for(int i = 0; i < bytesIn/4; i++){
        int32_t sampe = sBuffer[i];
        
        sampe >>= 14;
        samples[i] = sampe;
        ((int16_t *)sBuffer)[i] = samples[i];
        
    }
    bytesIn = bytesIn/2;
    
}
