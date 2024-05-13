/* 
    UART functionality based on Espressif Systems UART documentation

    This can be found at: 
    https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html

    Distributed under the Apache License, Version 2.0
    Available at: http://www.apache.org/licenses/LICENSE-2.0.

    ------- ------- ------- ------- -------

    description:  Transmit serial image stream from an esp32 cam
                  View stream by running ../../stream_reader.py 

    date: 12/05/24

*/


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "esp_camera.h"

#define TEST_ESP_OK(ret) assert(ret == ESP_OK)

//#define ESP32_WROVER_DEV 1
// #define ESP32S3_WROOM_CAM 1
#define ESP32CAM_AITHINKER 1

#ifdef ESP32S3_WROOM_CAM
// esp32s3 cam, psram set in octal mode 
#define CAM_PIN_PWDN -1  // power down is not used
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_XCLK 15 
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5 
#define CAM_PIN_D7 16 
#define CAM_PIN_D6 17 
#define CAM_PIN_D5 18 
#define CAM_PIN_D4 12 
#define CAM_PIN_D3 10 
#define CAM_PIN_D2 8 
#define CAM_PIN_D1 9
#define CAM_PIN_D0 11 
#define CAM_PIN_VSYNC 6 
#define CAM_PIN_HREF 7 
#define CAM_PIN_PCLK 13 
#endif

#ifdef ESP32_WROVER_DEV
// frame buf in DRAM
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 21
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 19
#define CAM_PIN_D2 18
#define CAM_PIN_D1 5
#define CAM_PIN_D0 4
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22
#endif


#ifdef ESP32CAM_AITHINKER
// ESP32Cam (AiThinker)
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22
#endif


#define UART_PORT_NUM UART_NUM_0
#define BUF_SIZE (4096)
#define BAUDRATE 115200

#define DELIM "\n12345678900STREAM00987654321\n"
static const char* _HEADER = DELIM "Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" DELIM;

static QueueHandle_t uart0_queue;

static const char *TAG = "uart send";


// function declarations
static esp_err_t init_camera(framesize_t frame_size, int jpeg_quality);
static esp_err_t init_uart(void);
static void cam_stream_task(void *pvParameters);




void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    TEST_ESP_OK(init_camera(8,12));
    TEST_ESP_OK(init_uart());    

    xTaskCreate(cam_stream_task, "cam_stream_task", 1024 * 8, NULL, 12, NULL);
}



/** 
 * @brief  innitialise esp32 UART for data transmission 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Error
 */
static esp_err_t init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 20, &uart0_queue, 0);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "UART driver installation failed");
        return err;
    }
    
    err = uart_param_config(UART_PORT_NUM, &uart_config);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "UART parameter configuration failed");
        return err;
    }

    err = uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initiate uart ");
        return err;
    }

    return err;
}


/**
 *  @brief Initialise and config camera 
 *  @return
 *    - ESP_OK   Success
 *    - ESP_FAIL Parameter error
 */
static esp_err_t init_camera(framesize_t frame_size, int jpeg_quality)
{
    camera_config_t camera_config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,     // YUV422,GRAYSCALE,RGB565,JPEG PIXFORMAT_JPEG
        .frame_size = frame_size,          // 1-12 for ov2640

        .jpeg_quality = jpeg_quality,      // 0-63, smaller num = better quality
        .fb_count = 1,                     // 
        .grab_mode = CAMERA_GRAB_LATEST,   // CAMERA_GRAB_WHEN_EMPTY, CAMERA_GRAB_LATEST
        .fb_location = CAMERA_FB_IN_DRAM   // PSRAM or DRAM, psram needs enabling in menuconfig 
    };

    esp_err_t ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error intitialising cam eeek!");
        return ret;
    }

    sensor_t *s = esp_camera_sensor_get();
    // take mirror image if camera is ov5640
    if (s->id.PID == OV5640_PID)
    {
        s->set_hmirror(s,1);
    }

    return ret;
}


/**
 * @brief Stream task to continuously capture and send images
 */
static void cam_stream_task(void *pvParameters)
{
    camera_fb_t *frame;
    char* info[200];

    for(;;)
    {
        frame = esp_camera_fb_get();
        //vTaskDelay(100/portTICK_PERIOD_MS);
        if(!frame)
        {
            ESP_LOGE(TAG,"failed to capture image");
            vTaskDelay(1000/portTICK_PERIOD_MS);
            continue;
        }
        
        // make header string
        snprintf((char *)info, sizeof(info), _HEADER, frame->len);
        
        // write to serial
        uart_write_bytes(UART_PORT_NUM, info, strlen(info));
        uart_write_bytes(UART_PORT_NUM, (const char*)frame->buf, frame->len);
        uart_write_bytes(UART_PORT_NUM, DELIM, strlen(DELIM));

        esp_camera_fb_return(frame);
    }
}