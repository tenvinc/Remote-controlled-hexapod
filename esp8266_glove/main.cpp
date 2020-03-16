#define _DEFAULT_SOURCE
extern "C"
{
#include <espressif/esp_common.h>
#include <esp8266.h>
#include <esp/uart.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <ssid_config.h>
#include <lwip/tcp.h>
#include <string.h>
}
#include "MPU9250.h"

#define LED_PIN 2

#define SERVER_PORT 8080

#ifdef SERVER_PORT
const u16_t ser_port = SERVER_PORT;
#else
const u16_t ser_port = 80;
#endif

#define POLL_INTERVAL 5

#define MPU9250_I2C_ADDR 0x68
#define MPU9250_SCL_PIN 5
#define MPU9250_SDA_PIN 4
/*********************** Function prototypes *********************************/
void httpd_task(void *pvParameters);
void sensor_task(void *pvParameters);
err_t ws_client_parse(unsigned char *data, u16_t len);
err_t ws_close();
err_t ws_poll(void *arg, struct tcp_pcb *pcb);
void ws_write(struct tcp_pcb *pcb, u8_t *data, u16_t len, u8_t mode);
void ws_ping(struct tcp_pcb *pcb);
/*****************************************************************************/
unsigned int retries = 0;
struct tcp_pcb *ws_pcb = NULL;
bool ws_connected = false;
ip_addr_t ser_addr;

MPU9250 *mpu9250_dev;
i2c_dev_t i2c1{0, MPU9250_I2C_ADDR};

static const char WS_HEADER[] = "Upgrade: websocket\r\n";
static const char WS_SERVER_SWITCH[] = "Switching Protocols";

SemaphoreHandle_t semaphore_ws;

enum ws_opcode_t
{
    OPCODE_TEXT = 0x01,
    OPCODE_BINARY = 0x02,
    OPCODE_CLOSE = 0x08,
    OPCODE_PING = 0x09,
    OPCODE_PONG = 0x0A
};

static err_t ws_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        printf("Connection has been closed.\n");
        ws_close();
        return ERR_CLSD;
    }
    LOCK_TCPIP_CORE();
    tcp_recved(pcb, p->len);
    UNLOCK_TCPIP_CORE();
    unsigned char *data = (unsigned char *)(p->payload);
    u16_t datalen = p->len;
    if (strnstr((char*)data, WS_HEADER, datalen) > 0 &&
        strnstr((char*)data, WS_SERVER_SWITCH, datalen) > 0 )
    {
        printf("Server has accepted the switch to websockets.\n");
        ws_connected = true;
    }
    if (ws_connected)
    {
        ws_client_parse(data, datalen);
    }
    pbuf_free(p);
    return ERR_OK;
}

static err_t ws_tcp_connected_cb(void *arg, struct tcp_pcb *pcb, err_t err)
{
    LOCK_TCPIP_CORE();
    tcp_recv(pcb, ws_recv_cb);
    UNLOCK_TCPIP_CORE();
    static char rsp[512];
    snprintf(rsp, sizeof(rsp),
             "GET / HTTP/1.1\r\n"
             "Host: 192.168.1.92:%d\r\n"
             "Connection: Upgrade\r\n"
             "Upgrade: websocket\r\n"
             "Sec-WebSocket-Version: 13\r\n"
             "Sec-WebSocket-Key: a0YBiKi7u7cdhbz8xu5FWQ==\r\n\r\n",
             ser_port);
    err = tcp_write(pcb, rsp, strlen(rsp), TCP_WRITE_FLAG_MORE);
    tcp_poll(pcb, ws_poll, POLL_INTERVAL);
    xSemaphoreGive(semaphore_ws);
    return err;
}

err_t ws_client_parse(unsigned char *data, u16_t len)
{
    if (data != NULL && len > 1)
    {
        u8_t opcode = data[0] & 0x0F;
        // check for mask bit
        bool is_masked = (data[1] & 0x80);
        if (is_masked)
        {
            printf("This is invalid for a client. Server should not mask.\n");
        }
        len = (data[1] & 0x7F);
        if (len > 125)
        {
            printf("This should not happen given our assumption of max 125 bytes.\n");
        }
        switch (opcode)
        {
        case 0x01:
        case 0x02:
        {
            unsigned char *payload = &data[2];
            printf("Received: %s\n", payload);
            break;
        }
        case 0x08:
            return ERR_CLSD;
            ws_close();
            break;
        }
    }
    return ERR_OK;
}

void ws_ping(struct tcp_pcb *pcb)
{
    char *ping = "ping";
    ws_write(pcb, (unsigned char *)ping, strlen(ping), OPCODE_PING);
}

err_t ws_poll(void *arg, struct tcp_pcb *pcb)
{
    err_t err;
    retries++;
    if (retries > 10)
    {
        printf("No response after 10 polls. Closing now.\n");
        err = ws_close();
        return err;
    }
    ws_ping(pcb);
    return ERR_OK;
}

void ws_write(struct tcp_pcb *pcb, u8_t *data, u16_t len, u8_t mode)
{
    if (pcb == NULL)
    {
        return;
    }
    if (len > 125)
    {
        return;
    }
    unsigned char buf[len + 2];
    buf[0] = 0x80 | mode;
    buf[1] = len;
    memcpy(&buf[2], data, len);
    LOCK_TCPIP_CORE();
    tcp_write(pcb, (void *)buf, sizeof(buf) / sizeof(buf[0]), TCP_WRITE_FLAG_COPY);
    UNLOCK_TCPIP_CORE();
}

err_t ws_close()
{
    err_t err = ERR_OK;
    if (ws_pcb)
    {
        tcp_recv(ws_pcb, NULL);
        err = tcp_close(ws_pcb);
    }
    if (err != ERR_OK)
    {
        return err;
    }
    ws_pcb = NULL;
    ws_connected = false;
    printf("Closed websocket connection.");
    // start the reconnection again
    xTaskCreate(&httpd_task, "HTTP Daemon", 1024, NULL, 2, NULL);
    return err;
}

void ws_task(void *pvParameters)
{
    int msg_idx = 0;
    while (1)
    {
        if (xSemaphoreTake(semaphore_ws, portMAX_DELAY) == pdTRUE)
        {
            char *msg = "This is a test message from the device itself. Simulating sensor reading.";
            msg_idx++;
            ws_write(ws_pcb, (u8_t *)msg, strlen(msg), OPCODE_TEXT);
            xSemaphoreGive(semaphore_ws);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void httpd_task(void *pvParameters)
{
    err_t err;
    while (1)
    {
        if (sdk_wifi_get_opmode() == STATION_MODE && sdk_wifi_station_get_connect_status() == STATION_GOT_IP)
        {
            printf("IP obtained. Now trying to connect to a live server.\n");
            IP4_ADDR(&ser_addr, 192, 168, 1, 92);
            LOCK_TCPIP_CORE(); // Mutex to make tcp threadsafe
            ws_pcb = tcp_new();
            err = tcp_connect(ws_pcb, &ser_addr, ser_port, ws_tcp_connected_cb);
            UNLOCK_TCPIP_CORE();
            if (err != ERR_OK)
            {
                printf("Connection could not be established. Please try again.\n");
            }
            break;
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// void sensor_task(void *pvParameters) {
//     if (i2c_init(i2c1.bus, MPU9250_SCL_PIN, MPU9250_SDA_PIN, I2C_FREQ_80K) != 0) {
//         printf("Error has occurred while initializing MPU9250 I2C.\n");
//     }
//     // mpu9250_dev = new MPU9250(i2c1, NULL);
//     uint8_t buf;
//     uint8_t reg = 0xAB;
//     while (1) {
//         printf("Read something from slave %d", i2c_slave_read(0, 0x68, &reg, &buf, 1));
//         vTaskDelay(300 / portTICK_PERIOD_MS);
//     }
//     // uint8_t data = 0x01;
//     // uint8_t err = mpu9250_dev->writeRegister((uint8_t) MPU9250_ACCEL_CONFIG, &data);
//     // if (err != MPU9250_ERR_OK) {
//     //     printf("This is not supposed to happen.\n");
//     // }
//     // printf("Deleting the task now.\n");
//     // free(mpu9250_dev);
//     vTaskDelete(NULL);
// }

extern "C" void user_init(void)
{
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

    /* initialize tasks */
    semaphore_ws = xSemaphoreCreateBinary();

    xTaskCreate(&httpd_task, "HTTP Daemon", 1024, NULL, 2, NULL);
    xTaskCreate(&ws_task, "Websocket Daemon", 1024, NULL, 2, NULL);
    // xTaskCreate(&sensor_task, "Sensor task", 1024, NULL, 2, NULL);
}
