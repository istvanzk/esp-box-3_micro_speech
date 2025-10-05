/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "audio_provider.h"

#include <cstdlib>
#include <cstring>

// FreeRTOS.h must be included before some of the following dependencies.
// Solves b/150260343.
// clang-format off
#include "freertos/FreeRTOS.h"
// clang-format on

// Use the new esp_codec_dev APIs for IDF v>5 and ADF v2.7 pipeline
// We use ADF pipeline with NS and VAD, that captures dual-mic input and feeds it to micro_speech model
// [ES7210 ADC] → [I2S Stream Reader] → [NS Filter] → [Raw Audio Buffer] → [Downmix to Mono] → [micro_speech Model]
#include "bsp/esp-box-3.h"
#include "audio_common.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "esp_codec_dev.h"
//#include "esp_codec_dev_defaults.h"
//#include "esp_codec_dev_esp_adf.h"
#include "filter_resample.h"
#include "esp_vad.h"

//#include "esp_spi_flash.h"
#include "esp_log.h"
#include "spi_flash_mmap.h"
#include "esp_system.h"
#include "esp_timer.h"
//#include "driver/gpio.h"

#include "freertos/task.h"

#include "micro_model_settings.h"

using namespace std;

// for c2 and c3, I2S support was added from IDF v4.4 onwards
#define NO_I2S_SUPPORT CONFIG_IDF_TARGET_ESP32C2 || \
                          (CONFIG_IDF_TARGET_ESP32C3 \
                          && (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)))

#define ADC_BUFFER_SIZE 2048

static audio_pipeline_handle_t pipeline = NULL;
static audio_element_handle_t i2s_stream = NULL;
static audio_element_handle_t rsp_filter = NULL;
static vad_handle_t vad_inst = NULL;

static const char* TAG = "TF_LITE_AUDIO_PROVIDER";
volatile int32_t g_latest_audio_timestamp = 0;
/* The ML model requires as input 
 * 20ms new captured data and 10ms old data each time
 * storing old data in the histrory buffer , {
 * history_samples_to_keep = 10 * 16 } */
constexpr int32_t history_samples_to_keep =
    ((kFeatureDurationMs - kFeatureStrideMs) *
     (kAudioSampleFrequency / 1000));
/* new samples to get each time from new captured data, 
{ new_samples_to_get =  20 * 16 } */
constexpr int32_t new_samples_to_get =
    (kFeatureStrideMs * (kAudioSampleFrequency / 1000));


namespace {
int16_t g_audio_output_buffer[kMaxAudioSampleSize * 32];
bool g_is_audio_initialized = false;
int16_t g_history_buffer[history_samples_to_keep];
} // namespace

TfLiteStatus InitAudioRecording() {

    /* Initialize the BSP Audio */
    // Declared in: managed_components/espressif__esp-box-3/include/bsp/esp-box-3.h and defined in sp-box-3_idf5.c
    //bsp_audio_init(i2s_std_config_t *i2s_config);  // Pass NULL to use default values (Mono, duplex, 16bit, 22050 Hz)

    /* 
    Intialiaze the BSP Audio microphone codec
    Usage: see managed_components/espressif__esp-box-3/API.md
    - Declared in: managed_components/espressif__esp-box-3/include/bsp/esp-box-3.h
    - Uses bsp_audio_init(NULL) internally!
    */
    esp_codec_dev_handle_t mic_codec = bsp_audio_codec_microphone_init();
    if (mic_codec == NULL) {
        ESP_LOGE(TAG, "Failed to initialize microphone codec");
        return kTfLiteError;
    }
    /* Set input gain (optional) */
    esp_codec_dev_set_in_gain(mic_codec, 30.0);

    /* Modify I2S stream reader cfg for 16kHz sampling rate and STEREO stream output */
    /* For IDF v5 and later */
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT(); 
    i2s_cfg.type = AUDIO_STREAM_READER;
    i2s_cfg.transmit_mode = I2S_COMM_MODE_STD, 
    i2s_cfg.chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    i2s_cfg.std_cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kAudioSampleFrequency); // 16kHz
    i2s_cfg.std_cfg.slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO); // Stereo
    i2s_cfg.task_core = 1; // Use core 1
    i2s_cfg.task_stack = 4096;
    i2s_cfg.task_prio = 5;
    i2s_cfg.stack_in_ext = true;
    
    /* Set up the I2S stream from ADC (but do not start it) */
    i2s_stream = i2s_stream_init(&i2s_cfg);


    /* NS Filter configuration */
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = kAudioSampleFrequency;
    rsp_cfg.src_ch = 2;
    rsp_cfg.dest_rate = kAudioSampleFrequency;
    rsp_cfg.dest_ch = 2;
    rsp_cfg.complexity = 1;  // Enables noise suppression
    rsp_cfg.task_core = 1;
    rsp_filter = rsp_filter_init(&rsp_cfg);

    /* Pipeline configuration */
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    audio_pipeline_register(pipeline, i2s_stream, "es7210");
    audio_pipeline_register(pipeline, rsp_filter, "ns");    
    const char *link_tag[2] = {"es7210", "ns"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);
 
    /*
    ESP-ADF audio_pipeline_run, performs the following internally:
    - Creates and starts FreeRTOS tasks for each registered audio element (e.g., adc_stream_reader, rsp_filter)
    - Handles data flow between elements via ring buffers
    - Manages streaming, buffering, and event propagation 
    We do not need to manually create a separate task just to “pull” audio from the ADC stream.
    We only need to call audio_element_input() from your GetAudioSamples() — no need for a separate CaptureSamples task!
    */
    esp_err_t ret = audio_pipeline_run(pipeline);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start audio pipeline");
        return kTfLiteError;
    }

    /* Set up VAD */
    vad_inst = vad_create(VAD_MODE_0);
    if (vad_inst == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed!");

        ESP_LOGI(TAG, "Destroy VAD");
        vad_destroy(vad_inst);

        ESP_LOGI(TAG, "Stop audio_pipeline and release all resources");
        audio_pipeline_stop(pipeline);
        audio_pipeline_wait_for_stop(pipeline);
        audio_pipeline_terminate(pipeline);

        /* Terminate the pipeline before removing the listener */
        audio_pipeline_remove_listener(pipeline);

        audio_pipeline_unregister(pipeline, i2s_stream);
        audio_pipeline_unregister(pipeline, rsp_filter);

        /* Release all resources */
        audio_pipeline_deinit(pipeline);
        audio_element_deinit(i2s_stream);
        audio_element_deinit(rsp_filter);

        return kTfLiteError;
    }
    
    /* Wait for the first samples to be captured */
    vTaskDelay(1);

    ESP_LOGI(TAG, "Audio Recording started");
    return kTfLiteOk;
}

TfLiteStatus GetAudioSamples(int start_ms, int duration_ms,
                             int* audio_samples_size, 
                             int16_t** audio_samples) {
    char stereo_buffer[2 * new_samples_to_get * sizeof(int16_t)]; // 2 channels × 16-bit

    if (!g_is_audio_initialized) {
      TfLiteStatus init_status = InitAudioRecording();
      if (init_status != kTfLiteOk) {
        return init_status;
      }
      g_is_audio_initialized = true;
    }
    /* Copy previous 160 samples (320 bytes) into output_buff from the history buffer */
    memcpy((void*)(g_audio_output_buffer),
          (void*)(g_history_buffer),
          history_samples_to_keep * sizeof(int16_t));

    /* Read 2-channels samples from NS Filter instead of ADC */
    int bytes_read = audio_element_input(rsp_filter, stereo_buffer, 2 * new_samples_to_get * sizeof(int16_t));
    //int bytes_read = audio_element_input(adc_stream_reader, stereo_buffer, new_samples_to_get * 4);
    if (bytes_read <= 0){
      ESP_LOGE(TAG, " Model Could not read data from Ring Buffer");
      //return kTfLiteError;
    } else if (bytes_read < 2 * new_samples_to_get * sizeof(int16_t)) {
      ESP_LOGD(TAG, " Partial Read of Data by Model ");
      ESP_LOGV(TAG, " Could only read %d bytes when required %d bytes ",
             bytes_read, (int) (2 * new_samples_to_get * sizeof(int16_t)));
    } 

    /* Average of the 2 channels = downmix to mono channel.
    Store 320 samples (640 bytes) to g_audio_output_buffer from 160 index onwards.
    First 160 samples (320 bytes) are from the history buffer.
    */ 
    int samples = bytes_read / ( 2 * sizeof(int16_t) );
    for (int i = 0; i < samples; ++i) {
        int16_t left = ((int16_t *)stereo_buffer)[i * 2];
        int16_t right = ((int16_t *)stereo_buffer)[i * 2 + 1];
        g_audio_output_buffer[i + history_samples_to_keep] = (left + right) / 2;
    }
    
    /* Copy new 160 samples (320 bytes) from output_buff into history buffer */
    memcpy((void*)(g_history_buffer),
          (void*)(g_audio_output_buffer + new_samples_to_get),
          history_samples_to_keep * sizeof(int16_t));

    /* Check VAD status */
    vad_state_t vad_state = vad_process(vad_inst, g_audio_output_buffer, kAudioSampleFrequency, kFeatureDurationMs);
    if (vad_state != VAD_SPEECH) { // No speech detected
        *audio_samples_size = 0;
        *audio_samples = nullptr;
        return kTfLiteOk;  
    }

    /* Speech detected: the output */
    *audio_samples_size = samples;
    *audio_samples = g_audio_output_buffer;

    /* Update timestamp */
    g_latest_audio_timestamp += (samples * 1000) / kAudioSampleFrequency;

    return kTfLiteOk;

}

int32_t LatestAudioTimestamp() { return g_latest_audio_timestamp; }
