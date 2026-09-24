#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "driver/gpio.h"

#define WIFI_SSID      "SEU_WIFI_SSID"
#define WIFI_PASS      "SEU_WIFI_SENHA"
#define MQTT_BROKER_URI "mqtt://10.1.133.82:1883"

#define LED_GPIO       GPIO_NUM_2  // Ajuste o pino do LED/Relé do Bat-Sinal conforme seu hardware

static const char *TAG = "BAT_SINAL";
static esp_mqtt_client_handle_t client = NULL;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Tentando reconectar ao Wi-Fi...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP obtido:" IPSTR, IP2STR(&event->ip_addr));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Conectado ao Broker MQTT. Inscrevendo-se no tópico...");
            esp_mqtt_client_subscribe(client, "gotham/dpgc/batsignal", 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Desconectado do Broker MQTT.");
            break;
        case MQTT_EVENT_DATA:
            // Garante que a string recebida está terminada em nulo para comparações seguras
            if (strncmp(event->topic, "gotham/dpgc/batsignal", event->topic_len) == 0) {
                char data_buffer[64] = {0};
                int len = event->data_len < sizeof(data_buffer) ? event->data_len : sizeof(data_buffer) - 1;
                memcpy(data_buffer, event->data, len);

                if (strcmp(data_buffer, "BAT_SIGNAL_ON") == 0) {
                    gpio_set_level(LED_GPIO, 1);
                    printf("\n[ALERTA] Bat-Sinal Ativado! O Cavaleiro das Trevas foi convocado.\n\n");
                } else if (strcmp(data_buffer, "BAT_SIGNAL_OFF") == 0) {
                    gpio_set_level(LED_GPIO, 0);
                    printf("\n[INFO] Bat-Sinal Desativado.\n\n");
                }
            }
            break;
        default:
            break;
    }
}

void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void heartbeat_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        if (client) {
            char payload[64];
            int uptime = (int)(esp_timer_get_time() / 1000000);
            snprintf(payload, sizeof(payload), "{\"device\": \"bat_sinal\", \"status\": \"ONLINE\", \"uptime_s\": %d}", uptime);
            esp_mqtt_client_publish(client, "gotham/dpgc/status", payload, 0, 1, 0);
            ESP_LOGI(TAG, "Heartbeat enviado: %s", payload);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Inicializando Node B (Bat-Sinal)...");
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Configuração da GPIO do LED
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0); // Inicializa apagado

    wifi_init_sta();
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    
    mqtt_app_start();
    xTaskCreate(heartbeat_task, "heartbeat_task", 2048, NULL, 5, NULL);
}