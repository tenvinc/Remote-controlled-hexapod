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
#include <algorithm>
#include "hexapod.h"

#define LED_PIN 2

#define MPU9250_SCL_PIN 5
#define MPU9250_SDA_PIN 4

#define SYS_PRINTF(fmt, ...) printf("%s: " fmt, "MAIN", ##__VA_ARGS__)

Hexapod hexy;

void sensor_task(void *pvParameters) {
  hexy.init();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  for (int i=0; i<10; i++) {
    hexy.walkForward();
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
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
