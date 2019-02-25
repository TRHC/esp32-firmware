#ifndef APP_WIFI_H
#define APP_WIFI_H
  esp_err_t wifi_event_handler(void*, system_event_t);
  void wifi_init_sta();
#endif