#ifndef _WEBSOCKET_H_
#define _WEBSOCKET_H_

extern "C" {
#include "stdint.h"
#include <lwip/tcp.h>
}



class WebSocket {
public:
  typedef void (*on_connected_cb_fn) ();
  typedef void (*on_recv_cb_fn) (char *data);
  typedef void (*on_close_cb_fn) ();

  enum State_t {
    CONNECTING,   // Websocket fully established
    CONNECTED,    // Websocket still establishing
    DISCONNECTED  // Websocket not connected
  };

  struct tcp_pcb *ws_pcb;
  int retries = 0;
  State_t state = State_t::DISCONNECTED;
  uint8_t idx;
  
  err_t open(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint16_t port);
  err_t client_parse(unsigned char *data, u16_t len);
  err_t read(char *toread);
  err_t write(char* data, int len);
  err_t close();

  // Wrapper functions to call the callbacks
  void on_connected();
  void on_recv();
  void on_close();

private:
  char data[256];
  ip_addr_t ser_addr;
  uint16_t ser_port;
  
  // Callback functions
  on_connected_cb_fn on_connected_cb;
  on_recv_cb_fn on_recv_cb;
  on_close_cb_fn on_close_cb;
};

#endif