#include <Arduino.h>

#define CAMERA_MODEL_WROVER_KIT
#include "camera_pins.h"

#include <Arduino.h>
#include <esp_camera.h>

// カメラ設定
camera_config_t config;

void initCAM()
{
    Serial.begin(115200);
    // カメラ初期化
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565;

    // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
    //                      for larger pre-allocated frame buffer.
    if (psramFound())
    {
        config.frame_size = FRAMESIZE_QQVGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    }
    else
    {
        config.frame_size = FRAMESIZE_QQVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }
    // 他の設定も追加可能
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
}


void detectOrangeColor()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
        return;
    }
    uint16_t *pixels = (uint16_t *)fb->buf;
    size_t pixelCount = fb->width * fb->height;

    int orangePixelCount = 0;

    // Serial.printf("pixelCount: %d\n", pixelCount);
    // Serial.printf("format: %d\n", fb->format);

    for (size_t i = 0; i < pixelCount; i++)
    {
        uint16_t pixel = pixels[i];
        uint8_t r = (pixel >> 11) & 0x1F; // 5ビットの赤成分
        uint8_t g = (pixel >> 5) & 0x3F;  // 6ビットの緑成分
        uint8_t b = pixel & 0x1F;         // 5ビットの青成分

        r = (r * 255 + 15) / 31;
        g = (g * 255 + 31) / 63;
        b = (b * 255 + 15) / 31;

        if (i == pixelCount / 2)
        {
            Serial.printf("%d, %d, %d\n", r, g, b);
        }
        // オレンジ色の範囲を定義
        if (r >= 200 && g >= 100 && b <= 50)
        {
            orangePixelCount++;
            // Serial.println("found orange pixel");
        }
    }
    esp_camera_fb_return(fb); // メモリ解放
    Serial.printf("Orange pixels: %d\n", orangePixelCount);
}
