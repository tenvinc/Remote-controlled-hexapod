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
#include "websocket.h"

#define LED_PIN 2

#define MPU9250_SCL_PIN 5
#define MPU9250_SDA_PIN 4

#define SYS_PRINTF(fmt, ...) printf("%s: " fmt, "MAIN", ##__VA_ARGS__)

#define SER_PORT 8080

typedef enum Action_type_t {
  STOP,
  KEEP,
  STAND,
  MOVE_LEFT,
  MOVE_RIGHT,
  MOVE_FORWARD,
  MOVE_BACKWARD
};

typedef struct {
  Action_type_t actionType;
  uint8_t velocity;
} Action_t;

Hexapod hexy;
WebSocket websocket;

QueueHandle_t xActionQueue;

void ws_on_recv(char *data) {
  Action_t action;
  // SYS_PRINTF("Recved %d bytes: %s\n", strlen(data), data);
  if (strncmp(data, "KEEP", 4) == 0) {
    action = {KEEP, 64};
  } else if (strncmp(data, "STAND", 5) == 0) {
    action = {STAND, 64};
  } else if (strncmp(data, "LEFT", 4) == 0) {
    action = {MOVE_LEFT, 64};
  } else if (strncmp(data, "RIGHT", 5) == 0) {
    action = {MOVE_RIGHT, 64};
  } else if (strncmp(data, "FORWARD", 7) == 0) {
    action = {MOVE_FORWARD, 64};
  } else if (strncmp(data, "BACKWARD", 8) == 0) {
    action = {MOVE_BACKWARD, 64};
  } else {
    SYS_PRINTF("Command not registered.\n");
    return;
  }
  xQueueSendToBack(xActionQueue, &action, 0);
}

void sensor_task(void *pvParameters) {
  Action_t action;
  hexy.init();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  while (true) {
    if (xQueueReceive(xActionQueue, &action, portMAX_DELAY) == pdTRUE) {
      switch (action.actionType) {
        case KEEP:
          SYS_PRINTF("Keep\n");
          break;
        case STAND:
          SYS_PRINTF("Stand\n");
          break;  
        case MOVE_LEFT:
          SYS_PRINTF("Move left\n");
          break;
        case MOVE_RIGHT:
          SYS_PRINTF("Move right\n");
          break;
        case MOVE_FORWARD:
          SYS_PRINTF("Move forward\n");
          break;
        case MOVE_BACKWARD:
          SYS_PRINTF("Move backward\n");
          break;
      }
      // for (int i=0; i<10; i++) {
      //   hexy.walkForward();
      //   vTaskDelay(200 / portTICK_PERIOD_MS);
      // }
    }
  }
  vTaskDelete(NULL);
}

void httpd_task(void *pvParameters) {
  err_t err;
  while (1) {
    if (sdk_wifi_get_opmode() == STATION_MODE &&
        sdk_wifi_station_get_connect_status() == STATION_GOT_IP) {
      SYS_PRINTF("IP obtained. Now trying to connect to a live server.\n");
      err = websocket.open(192, 168, 1, 104, SER_PORT);
      if (err != ERR_OK) {
        SYS_PRINTF("Connection could not be established. Please try again.\n");
        continue;
      }
      break;
    }
    vTaskDelay(300 / portTICK_PERIOD_MS);
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

  websocket.on_recv_cb = ws_on_recv;

  xActionQueue = xQueueCreate(10, sizeof(Action_t));
  if (xActionQueue == NULL) {
    SYS_PRINTF("Failed to create xActionQueue!\n");
  }

  xTaskCreate(&httpd_task, "HTTP Daemon", 1024, NULL, 2, NULL);
  xTaskCreate(&sensor_task, "Sensor task", 1024, NULL, 2, NULL);
}
