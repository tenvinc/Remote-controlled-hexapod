#define _DEFAULT_SOURCE
extern "C" {
#include <FreeRTOS.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <espressif/esp_common.h>
#include <lwip/tcp.h>
#include <semphr.h>
#include <ssid_config.h>
#include <stdio.h>
#include <string.h>
#include <task.h>
}
#include "PCA9685_driver.h"

#define LED_PIN 2

#define MPU9250_SCL_PIN 5
#define MPU9250_SDA_PIN 4

#define SYS_PRINTF(fmt, ...) printf("%s: " fmt, "MAIN", ##__VA_ARGS__)

PCA9685_Servo_Driver driver;

typedef struct servo_intrinsics {
  uint16_t min_pw;      // min pulse width
  uint16_t max_pw;      // max pulse width
  uint8_t angle_range;  // corresponding angle range with 0 degrees when pulse
                        // width = min_pw
} servo_intrinsics_t;

servo_intrinsics_t SERVO_INTRINSICS_LUT[12] = {
    {600, 2620, 180}, {600, 2620, 170}, {600, 2650, 180}, {600, 2650, 175},
    {600, 2650, 175}, {600, 2650, 175}, {700, 2650, 180}, {700, 2650, 170},
    {700, 2650, 175}, {700, 2750, 180}, {600, 2550, 180}, {500, 2550, 180},
};

bool SERVO_INVRT_LUT[12] = {    // indicates which servo is inverted
  false, false, 
  false, false, 
  true, false,
  false, false,
  false, false,
  false, false
};

void sensor_task(void *pvParameters) {
  driver.begin();
  for (int i = 0; i < 12; i++) {
    driver.attach(i, SERVO_INTRINSICS_LUT[i].min_pw,
                  SERVO_INTRINSICS_LUT[i].max_pw,
                  SERVO_INTRINSICS_LUT[i].angle_range);
    if (i%2) {
      driver.writeAngle(i, 75);
    } else {
      driver.writeAngle(i, SERVO_INVRT_LUT[i] ? SERVO_INTRINSICS_LUT[i].angle_range - 0 : 0);
    }
  }
  // while (1) {
  //   for (int ms = min; ms <= max; ms+=10) {
  //     driver.writeMicroseconds(1, ms);
  //     vTaskDelay(5 / portTICK_PERIOD_MS);
  //   }
  //   for (int ms = max; ms >= min; ms-=10) {
  //     driver.writeMicroseconds(1, ms);
  //     vTaskDelay(5 / portTICK_PERIOD_MS);
  //   }
  // }
  // while (1) {
  //   for (int n = 0; n < 11; n++) {
  //     SYS_PRINTF("Adjusting servo %d\n", n);
  //     for (int i=75; i>0; i--) {
  //       driver.writeAngle(n, i);
  //       vTaskDelay(5 / portTICK_PERIOD_MS);
  //     }
  //     for (int i = 0; i < 150; i++) {
  //       driver.writeAngle(n, i);
  //       vTaskDelay(5 / portTICK_PERIOD_MS);
  //     }
  //     for (int i=150; i>=75; i--) {
  //       driver.writeAngle(n, i);
  //       vTaskDelay(5 / portTICK_PERIOD_MS);
  //     }
  //   }
  // }
  vTaskDelete(NULL);
}

extern "C" void user_init(void) {
  uart_set_baud(0, 115200);
  printf("SDK version:%s\n", sdk_system_get_sdk_version());

  struct sdk_station_config config;
  memcpy(config.ssid, WIFI_SSID, sizeof(WIFI_SSID));
  memcpy(config.password, WIFI_PASS, sizeof(WIFI_PASS));

  /* required to call wifi_set_opmode before station_set_config */
  sdk_wifi_set_opmode(STATION_MODE);
  sdk_wifi_station_set_config(&config);
  sdk_wifi_station_connect();

  /* turn off LED */
  gpio_enable(LED_PIN, GPIO_OUTPUT);
  gpio_write(LED_PIN, true);

  // xTaskCreate(&httpd_task, "HTTP Daemon", 1024, NULL, 2, NULL);
  // xTaskCreate(&ws_task, "Websocket Daemon", 1024, NULL, 2, NULL);
  xTaskCreate(&sensor_task, "Sensor task", 1024, NULL, 2, NULL);
}
