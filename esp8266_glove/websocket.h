#ifndef _WEBSOCKET_H_
#define _WEBSOCKET_H_

#include "stdint.h"

class WebSocket {
public:
  enum State_t {
    CONNECTING,   // Websocket fully established
    CONNECTED,    // Websocket still establishing
    DISCONNECTED  // Websocket not connected
  };

  struct tcp_pcb *ws_pcb;
  int retries = 0;
  State_t state;
  uint8_t idx;
  
  WebSocket(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint16_t port);
  err_t client_parse(unsigned char *data, u16_t len);
  err_t read(char *toread);
  err_t write(char* data, int len);
  err_t close();

private:
  char data[256];
  ip_addr_t ser_addr;
  uint16_t port;
};

#endif