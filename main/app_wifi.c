#include "trhc.h"
#include <esp_http_server.h>
#include <esp_https_ota.h>
#include "esp_netif.h"
#include "esp_ota_ops.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

char*   wifi_conf_ssid;
char*   wifi_conf_pass;
static  httpd_handle_t server = NULL;

static int getConnectionInfo() {
    nvs_handle wifi_nvs_handle;
    size_t req_size;
    printf("Start");
    nvs_open("wifiConf", NVS_READWRITE, &wifi_nvs_handle);
    
    // nvs_set_str(wifi_nvs_handle, "ssid", "ssid");
    // nvs_set_str(wifi_nvs_handle, "pass", "pass");
    // nvs_commit(wifi_nvs_handle);

    nvs_get_str(wifi_nvs_handle, "ssid", NULL, &req_size);
    wifi_conf_ssid = malloc(req_size);
    nvs_get_str(wifi_nvs_handle, "ssid", wifi_conf_ssid, &req_size);

    nvs_get_str(wifi_nvs_handle, "pass", NULL, &req_size);
    wifi_conf_pass = malloc(req_size);
    nvs_get_str(wifi_nvs_handle, "pass", wifi_conf_pass, &req_size);


    if(strlen(wifi_conf_ssid) == 0) {
        printf("Ret");
        nvs_set_str(wifi_nvs_handle, "ssid", "ssid");
        nvs_set_str(wifi_nvs_handle, "pass", "pass");
        nvs_commit(wifi_nvs_handle);
        return -1;
    }
    printf("Cls");
    nvs_close(wifi_nvs_handle);

	return 0;
}

static void saveConnectionInfo(const char* ssid, const char* pass) {
    nvs_handle wifi_nvs_handle;
    nvs_open("wifiConf", NVS_READWRITE, &wifi_nvs_handle);

    nvs_set_str(wifi_nvs_handle, "ssid", ssid);
    nvs_set_str(wifi_nvs_handle, "pass", pass);

    nvs_commit(wifi_nvs_handle);
    nvs_close(wifi_nvs_handle);
    
    wifi_config_t sta_config;
    strcpy((void*)sta_config.sta.ssid , ssid);
    strcpy((void*)sta_config.sta.password , pass);

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config) );
    ESP_LOGI("wifiConf", "WiFi conf changed: %s", ssid);
    esp_wifi_connect();
}

static esp_err_t http_index_html(httpd_req_t *req) {
    extern const unsigned char index_start[]    asm("_binary_index_html_start");
    extern const unsigned char index_end[]      asm("_binary_index_html_end");
    const size_t index_size = (index_end - index_start);

    httpd_resp_send_chunk(req, (const char *)index_start, index_size);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

httpd_uri_t index_html = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = http_index_html,
    .user_ctx  = NULL
};


static esp_err_t http_conf_form(httpd_req_t *req) {
    char*  buf;
    size_t buf_len;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char ssid[32];
            char pass[64];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "ssid", ssid, sizeof(ssid)) == ESP_OK) {
                if (httpd_query_key_value(buf, "pass", pass, sizeof(pass)) == ESP_OK) {
                    saveConnectionInfo(ssid, pass);
                    httpd_resp_sendstr_chunk(req, "OK");
                    httpd_resp_sendstr_chunk(req, NULL);
                    return ESP_OK;
                }
            }
            free(buf);
        }
    }
    httpd_resp_sendstr_chunk(req, "FAIL");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_FAIL;
}

httpd_uri_t conf_form = {
    .uri      = "/conf",
    .method   = HTTP_GET,
    .handler  = http_conf_form,
    .user_ctx = NULL
};


esp_err_t do_firmware_upgrade(char* domain) {
    //esp_http_client_config_t config = {
    //    .url = "http://trhc.shpag.ga/ota.bin",
    //};
    esp_http_client_config_t config = {
        .host = domain,
        .path = "/ota.bin",
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
    };
    //sprintf(buf, "http://trhc.shpag.ga/ota.bin");
    //config.url = "http://trhc.shpag.ga/ota.bin";

    ESP_LOGI("ota", "Performing OTA Update from %s", config.host);

    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        // esp_restart();
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

static esp_err_t http_ota_form(httpd_req_t *req) {
    char*  buf;
    size_t buf_len;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char *domain;
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "domain", domain, sizeof(domain)) == ESP_OK) {
                if(do_firmware_upgrade(domain) == ESP_OK) {
                    httpd_resp_sendstr_chunk(req, "OK, REBOOT NOW");
                    httpd_resp_sendstr_chunk(req, NULL);
                    free(buf);
                    return ESP_OK;
                }
            }
        }
    }
    httpd_resp_sendstr_chunk(req, "FAIL");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_FAIL;
}

httpd_uri_t ota_form = {
    .uri      = "/ota",
    .method   = HTTP_GET,
    .handler  = http_ota_form,
    .user_ctx = NULL
};

httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_html);
        httpd_register_uri_handler(server, &conf_form);
        httpd_register_uri_handler(server, &ota_form);
        return server;
    }

    ESP_LOGE("http", "Error starting server!");
    return NULL;
}

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if(event_base != WIFI_EVENT || event_base != IP_EVENT) {
        return;
    }
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        start_webserver();
        break;
    case IP_EVENT_STA_GOT_IP:
        // xEventGroupSetBits(s_network_event_group, BIT0);
        // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
        s_retry_num = 0;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("wifiConf", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        mqtt_app_start();
        break;
    case WIFI_CONNECTED_BIT:
        if (s_retry_num < CONFIG_WIFI_MAXIMUM_RETRY) {
            // xEventGroupClearBits(s_network_event_group, BIT0);
            esp_wifi_connect();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            s_retry_num++;
            ESP_LOGI("wifiConf", "retry to connect to the AP");
        } else {
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA) );
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            vTaskDelay(50000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI("wifiConf","connect to the AP fail");
            break;
        }
    default:
        break;
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA) );

    wifi_config_t sta_config = {
        .sta = {
            .ssid       = CONFIG_WIFI_SSID,
            .password   = CONFIG_WIFI_PASS
        },
    };

    if(getConnectionInfo() == 0) {
        strcpy((void*)sta_config.sta.ssid ,     wifi_conf_ssid);
        strcpy((void*)sta_config.sta.password , wifi_conf_pass);
        ESP_LOGI("wifiConf", "WiFi conf loaded from mem: %s", wifi_conf_ssid);
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config) );

    wifi_config_t ap_config = {
        .ap = {
            .ssid           = CONFIG_AP_WIFI_SSID,
            .ssid_len       = strlen(CONFIG_AP_WIFI_SSID),
            .password       = CONFIG_AP_WIFI_PASS,
            .max_connection = 5,
            .authmode       = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI("wifiConf", "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("wifiConf", "connected to ap");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI("wifiConf", "Failed to connect");
    } else {
        ESP_LOGE("wifiConf", "UNEXPECTED EVENT");
    }
}

// void wifi_init_sta() {
//     tcpip_adapter_init();
//     ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL) );

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA) );

//     wifi_config_t sta_config = {
//         .sta = {
//             .ssid       = CONFIG_WIFI_SSID,
//             .password   = CONFIG_WIFI_PASS
//         },
//     };

//     if(getConnectionInfo() == 0) {
//         strcpy((void*)sta_config.sta.ssid , wifi_conf_ssid);
//         strcpy((void*)sta_config.sta.password , wifi_conf_pass);
//         ESP_LOGI("wifiConf", "WiFi conf loaded from mem: %s", wifi_conf_ssid);
//     }

//     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config) );
//     wifi_config_t ap_config = {
//         .ap = {
//             .ssid           = CONFIG_AP_WIFI_SSID,
//             .ssid_len       = strlen(CONFIG_AP_WIFI_SSID),
//             .password       = CONFIG_AP_WIFI_PASS,
//             .max_connection = 5,
//             .authmode       = WIFI_AUTH_WPA_WPA2_PSK
//         },
//     };
//     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config) );
//     ESP_ERROR_CHECK(esp_wifi_start() );
// }