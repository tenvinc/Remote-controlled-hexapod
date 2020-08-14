#include "websocket.h"

extern "C" {
#include <lwip/tcp.h>
#include <stdio.h>
#include <string.h>
}

#define DEBUG_PRINTF(fmt, ...) printf("Websocket: " fmt "\n", ##__VA_ARGS__);

#define POLL_INTERVAL 5
#define MAX_RETRIES 100

static const char WS_HEADER[] = "Upgrade: websocket\r\n";
static const char WS_SERVER_SWITCH[] = "Switching Protocols";

WebSocket *connected_websockets[5];
uint8_t connectedSize = 0;
WebSocket
    *connecting[5];  // temporary cache to hold websockets before finalising
uint8_t connectingSize = 0;  // current number of temporary websocket objects

enum ws_opcode_t {
  OPCODE_TEXT = 0x01,
  OPCODE_BINARY = 0x02,
  OPCODE_CLOSE = 0x08,
  OPCODE_PING = 0x09,
  OPCODE_PONG = 0x0A
};

/*********************************************************************
 * Websocket Class specific functions
 *********************************************************************/

WebSocket::WebSocket(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
                     uint16_t port) {
  IP4_ADDR(&ser_addr, a, b, c, d);
  LOCK_TCPIP_CORE();  // Mutex to make tcp threadsafe
  ws_pcb = tcp_new();
  err_t err = tcp_connect(ws_pcb, &ser_addr, port, ws_tcp_connected_cb);
  UNLOCK_TCPIP_CORE();
  if (err != ERR_OK) {
    state = State_t::DISCONNECTED;
    // TODO: to deallocate the pcb
    DEBUG_PRINTF("TCP connection failed.");
  }
  retries = 0;
  state = State_t::CONNECTING;
  idx = connectingSize;
  this->port = port;
  connecting[connectingSize++] = this;
}

err_t WebSocket::client_parse(unsigned char *data, u16_t len) {
  if (data != NULL && len > 1) {
    u8_t opcode = data[0] & 0x0F;
    // check for mask bit
    bool is_masked = (data[1] & 0x80);
    if (is_masked) {
      printf("This is invalid for a client. Server should not mask.\n");
    }
    len = (data[1] & 0x7F);
    if (len > 125) {
      printf("This should not happen given our assumption of max 125 bytes.\n");
    }
    switch (opcode) {
      case 0x01:
      case 0x02: {
        unsigned char *payload = &data[2];
        printf("Received: %s\n", payload);
        memcpy(data, payload, strlen((char *)payload)+1);
        break;
      }
      case 0x08:
        return ERR_CLSD;
        close();
        break;
    }
  }
  return ERR_OK;
}

err_t WebSocket::close() {
  err_t err = ERR_OK;
  if (ws_pcb) {
    tcp_recv(ws_pcb, NULL);
    err = tcp_close(ws_pcb);
  }
  if (err != ERR_OK) {
    return err;
  }
  ws_pcb = NULL;
  // Deallocate websocket
  removeWebsocket(this);
  state = State_t::DISCONNECTED;
  DEBUG_PRINTF("Closed websocket connection.");
  // // start the reconnection again
  // xTaskCreate(&httpd_task, "HTTP Daemon", 1024, NULL, 2, NULL);
  return err;
}

err_t WebSocket::write(char* data, int len) {
  if (ws_pcb == NULL) {
    return ERR_CLSD;
  }
  ws_write(ws_pcb, (uint8_t *) data, len, OPCODE_TEXT);
  return ERR_OK;
}

err_t WebSocket::read(char *toread) {
  if (ws_pcb == NULL) {
    return ERR_CLSD;
  }
  memcpy(toread, data, strlen(data)+1);
  return ERR_OK;
}

/*********************************************************************
 * File scope functions (Comprises of callbacks for LWIP/TCP)
 *********************************************************************/

static err_t ws_tcp_connected_cb(void *arg, struct tcp_pcb *pcb, err_t err) {
  LOCK_TCPIP_CORE();
  tcp_recv(pcb, ws_recv_cb);
  UNLOCK_TCPIP_CORE();
  static char rsp[512];
  // TODO: change hardcode IP to something else
  snprintf(rsp, sizeof(rsp),
           "GET / HTTP/1.1\r\n"
           "Host: 192.168.1.92:%d\r\n"
           "Connection: Upgrade\r\n"
           "Upgrade: websocket\r\n"
           "Sec-WebSocket-Version: 13\r\n"
           "Sec-WebSocket-Key: a0YBiKi7u7cdhbz8xu5FWQ==\r\n\r\n",
           pcb->remote_port);
  err_t err = tcp_write(pcb, rsp, strlen(rsp), TCP_WRITE_FLAG_MORE);
  tcp_poll(pcb, ws_poll, POLL_INTERVAL);
  return err;
}

static err_t ws_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p,
                        err_t err) {
  WebSocket *websocket;

  if ((websocket = findWebsocket(pcb)) == nullptr) return ERR_CLSD;

  if (p == NULL) {
    printf("Connection has been closed.\n");
    websocket->close();
    return ERR_CLSD;
  }

  LOCK_TCPIP_CORE();
  tcp_recved(pcb, p->len);
  UNLOCK_TCPIP_CORE();
  unsigned char *data = (unsigned char *)(p->payload);
  u16_t datalen = p->len;
  if (strnstr((char *)data, WS_HEADER, datalen) > 0 &&
      strnstr((char *)data, WS_SERVER_SWITCH, datalen) > 0) {
    removeFromConnecting(
        websocket->idx);  // Move websocket from tmp to the connected pile
    websocket->idx = connectedSize;
    connected_websockets[connectedSize++] = websocket;
    websocket->state = WebSocket::State_t::CONNECTED;
    printf("Server has accepted the switch to websockets.\n");
  }

  if (websocket->state == WebSocket::State_t::CONNECTED) {
    websocket->client_parse(data, datalen);
  }

  pbuf_free(p);
  return ERR_OK;
}

err_t ws_poll(void *arg, struct tcp_pcb *pcb) {
  err_t err;
  WebSocket *websocket;
  if ((websocket = findWebsocket(pcb)) == nullptr) return ERR_CLSD;

  websocket->retries++;
  if (websocket->retries > MAX_RETRIES) {
    err = websocket->close();
    printf("No response after 100 polls. Closed.\n");
    return err;
  }
  ws_ping(pcb);
  return ERR_OK;
}

void ws_ping(struct tcp_pcb *pcb) {
  char *ping = "ping";
  ws_write(pcb, (unsigned char *)ping, strlen(ping), OPCODE_PING);
}

void ws_write(struct tcp_pcb *pcb, u8_t *data, u16_t len, u8_t mode) {
  if (pcb == NULL) {
    return;
  }
  if (len > 125) {
    return;
  }
  unsigned char buf[len + 2];
  buf[0] = 0x80 | mode;
  buf[1] = len;
  memcpy(&buf[2], data, len);
  LOCK_TCPIP_CORE();
  tcp_write(pcb, (void *)buf, sizeof(buf) / sizeof(buf[0]),
            TCP_WRITE_FLAG_COPY);
  UNLOCK_TCPIP_CORE();
}

/*********************************************************************
 * Helper functions for locating corresponding WebSocket objects
 *********************************************************************/
WebSocket *findWebsocket(struct tcp_pcb *pcb) {
  int idx;
  if ((idx = findInConnected(pcb)) < connectedSize) {
    return connected_websockets[idx];
  } else if ((idx = findInConnecting(pcb)) < connectingSize) {
    return connecting[idx];
  }
  return nullptr;
}

uint8_t findInConnecting(struct tcp_pcb *pcb) {
  for (int i = 0; i < connectingSize; i++) {
    if (connecting[i]->ws_pcb == pcb) return i;
  }
  return connectingSize;
}

uint8_t findInConnected(struct tcp_pcb *pcb) {
  for (int i = 0; i < connectedSize; i++) {
    if (connected_websockets[i]->ws_pcb == pcb) return i;
  }
  return connectedSize;
}

void removeWebsocket(WebSocket *websocket) {
  switch (websocket->state) {
    case WebSocket::State_t::CONNECTING:
      removeFromConnecting(websocket->idx);
      break;
    case WebSocket::State_t::CONNECTED:
      removeFromConnected(websocket->idx);
      break;
    default:
      DEBUG_PRINTF("This should not be here. STATE %d", websocket->state);
  }
}

void removeFromConnecting(int toremove) {
  // Remove and compact
  for (int i = toremove + 1; i < connectingSize; i++) {
    connecting[i - 1] = connecting[i];
  }
  connectingSize--;
}

void removeFromConnected(int toremove) {
  // Remove and compact
  for (int i = toremove + 1; i < connectedSize; i++) {
    connected_websockets[i - 1] = connected_websockets[i];
  }
  connectedSize--;
}