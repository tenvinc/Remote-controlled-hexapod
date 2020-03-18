extern "C"
{
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
}
#include "Servo.h"

#define LED_PIN 5

#define SR_DATA 4
#define SR_SHIFT 12
#define SR_LATCH 2
#define MAX_PER_SR 8
#define MAX_SR 1

Servo *servos[MAX_SERVOS];

void updateShiftRegister(srReq recvReq)
/** Callback to shift bits onto the shift register. This callback will be called by an ISR, so 
 * need to keep this as short as possible.
 * Tested execution time on ESP8266EX: ~3 us
 */
{
  // printf("Pattern recv: 0x%2x\n", recvReq.bitPattern);
  for (int i = MAX_PER_SR - 1; i >= 0; i--)
  {
    gpio_write(SR_DATA, (bool)(recvReq.bitPattern & 0x80));
    gpio_write(SR_SHIFT, false);
    gpio_write(SR_SHIFT, true);
    recvReq.bitPattern <<= 1;
  }
  gpio_write(SR_LATCH, false);
  gpio_write(SR_LATCH, true);
}

void driver_task(void *pvParameters)
{
  for (int i=7; i>=0; i--) {
    servos[i] = new Servo(updateShiftRegister);
    servos[i]->attach(i+1);
  }
  while (1)
  {
    for (int i = 0; i < 180; i++)
    {
      for (int j=7; j>=0; j--) {
        servos[j]->write(i);
      }
      sdk_os_delay_us(10000);
    }
    for (int i = 180; i >= 0; i--)
    {
      for (int j=7; j>=0; j--) {
        servos[j]->write(i);
      }
      sdk_os_delay_us(10000);
    }
  }
  vTaskDelete(NULL);
}

extern "C" void user_init(void)
{
  uart_set_baud(0, 115200);
  printf("SDK version:%s\n", sdk_system_get_sdk_version());

  /* turn off LED */
  gpio_enable(LED_PIN, GPIO_OUTPUT);
  gpio_write(LED_PIN, true);

  /* initialise pins for SR */
  gpio_enable(SR_DATA, GPIO_OUTPUT);
  gpio_enable(SR_LATCH, GPIO_OUTPUT);
  gpio_enable(SR_SHIFT, GPIO_OUTPUT);

  uint8_t freq = sdk_system_get_cpu_freq();
  printf("Current frequency of cpu is %d\n", freq);

  BaseType_t xReturned = xTaskCreate(&driver_task, "Driver", 1024, NULL, 2, NULL);
  if (xReturned == pdPASS)
  {
    printf("Driver Task created.\n");
  }
}