#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include <sys/time.h>
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_client.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "protocol_examples_common.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"

void WaterMeasurementTask(void *pvParameters);
void toggle_led();
void process_mqtt_message(void *pvParameters);
void RelayController(void *pvParameters);

#define RMT_TX_CHANNEL 1 
#define RMT_TX_GPIO_NUM PIN_TRIGGER 
#define RMT_RX_CHANNEL 0
#define RMT_RX_GPIO_NUM PIN_ECHO
#define RMT_CLK_DIV 100 
#define RMT_TX_CARRIER_EN 0
#define rmt_item32_tIMEOUT_US 9500 
#define RMT_TICK_10_US (80000000/RMT_CLK_DIV/100000) 
#define ITEM_DURATION(d) ((d & 0x7fff)*10/RMT_TICK_10_US)
#define PIN_TRIGGER 18
#define PIN_ECHO 19
#define PIN_RELAY 5
#define ONBOARD_LED 2

/*
VCC AZUL
GND VERDE
ECHO LARANJA
TRIGGER MARROM
*/


#define GPIO_OUTPUT_PIN_SEL  (1ULL<<PIN_RELAY)
#define EXAMPLE_ESP_WIFI_SSID      "esptst"
#define EXAMPLE_ESP_WIFI_PASS      "uapabiluba"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64         

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_7;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_11db;
static const adc_unit_t unit = ADC_UNIT_1;

static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG = "eel7801_TAG";
static int s_retry_num = 0;
static const double minDistance;
esp_mqtt_client_handle_t mqtt_client = NULL;
int wifi_connected = 0;

typedef struct  {
	int topic_len;
	int data_len;
	char* topic;
	char* data;
} mqtt_msg_struct_t;

typedef struct {
	double minDistance;
	bool relayActive;
	int relayPeriod;
	int relayDutyCycle;
} relayStatus;

static relayStatus relaystatus;
mqtt_msg_struct_t mqtt_msg;

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

            msg_id = esp_mqtt_client_subscribe(client, "/topic/relay", 0);
            ESP_LOGI(TAG, "sent subscribe successful (relay), msg_id=%d", msg_id);
            msg_id = esp_mqtt_client_subscribe(client, "/topic/plant1", 0);
            ESP_LOGI(TAG, "sent subscribe successful (plant1), msg_id=%d", msg_id);

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            
            mqtt_msg.topic_len = event->topic_len;
            mqtt_msg.data_len = event->data_len;
            mqtt_msg.topic = event->topic;
            mqtt_msg.data = event->data;


            xTaskCreate(process_mqtt_message, "Process MQTT data", 2048, &mqtt_msg, 1, NULL);

            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}


esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
	switch(evt->event_id) {
		case HTTP_EVENT_ERROR:
			ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
			break;
		case HTTP_EVENT_ON_CONNECTED:
			ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
			break;
		case HTTP_EVENT_HEADER_SENT:
			ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
			break;
		case HTTP_EVENT_ON_HEADER:
			ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
			break;
		case HTTP_EVENT_ON_DATA:
			ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
			if (!esp_http_client_is_chunked_response(evt->client)) {
				// Write out data
				// printf("%.*s", evt->data_len, (char*)evt->data);
			}

			break;
		case HTTP_EVENT_ON_FINISH:
			ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
			break;
		case HTTP_EVENT_DISCONNECTED:
			ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
			break;
	}
	return ESP_OK;
}


static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(TAG, "got ip:%s",
				 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		s_retry_num = 0;
		wifi_connected = 1;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		{
			if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
				esp_wifi_connect();
				xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
				s_retry_num++;
				ESP_LOGI(TAG,"retry to connect to the AP");
			}
			ESP_LOGI(TAG,"connect to the AP fail\n");
			break;
		}
	default:
		break;
	}
	return ESP_OK;
}

static void check_efuse()
{
	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
		printf("eFuse Two Point: Supported\n");
	} else {
		printf("eFuse Two Point: NOT supported\n");
	}

	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
		printf("eFuse Vref: Supported\n");
	} else {
		printf("eFuse Vref: NOT supported\n");
	}
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
	if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
		printf("Characterized using Two Point Value\n");
	} else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
		printf("Characterized using eFuse Vref\n");
	} else {
		printf("Characterized using Default Vref\n");
	}

}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://ftaylsiy:dyygblVZaO0l@postman-01.cloudmqtt.com:17841",
        .event_handle = mqtt_event_handler,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}

static void HCSR04_tx_init()
{
	rmt_config_t rmt_tx;

	rmt_tx.channel = RMT_TX_CHANNEL;
	rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
	rmt_tx.mem_block_num = 1;
	rmt_tx.clk_div = RMT_CLK_DIV;

	rmt_tx.tx_config.loop_en = false;
	rmt_tx.tx_config.carrier_duty_percent = 50;
	rmt_tx.tx_config.carrier_freq_hz = 3000;
	rmt_tx.tx_config.carrier_level = 1;
	rmt_tx.tx_config.carrier_en = RMT_TX_CARRIER_EN;
	rmt_tx.tx_config.idle_level = 0;
	rmt_tx.tx_config.idle_output_en = true;

	rmt_tx.rmt_mode = 0;

	rmt_config(&rmt_tx);
	rmt_driver_install(rmt_tx.channel, 0, 0);
}

static void HCSR04_rx_init()
{
	rmt_config_t rmt_rx;

	rmt_rx.channel = RMT_RX_CHANNEL;
	rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
	rmt_rx.clk_div = RMT_CLK_DIV;
	rmt_rx.mem_block_num = 1;

	rmt_rx.rx_config.filter_en = true;
	rmt_rx.rx_config.filter_ticks_thresh = 100;
	rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
	
	rmt_rx.rmt_mode = RMT_MODE_RX;

	rmt_config(&rmt_rx);
	rmt_driver_install(rmt_rx.channel, 1000, 0);
}

static void HCSR04_init()
{
	HCSR04_tx_init();
	HCSR04_rx_init();
}

static double HCSR04_measure(uint32_t num_measurements)
{
	int i;
	double distance;

	rmt_item32_t item;
	item.level0 = 1;
	item.duration0 = RMT_TICK_10_US;
	item.level1 = 0;
	item.duration1 = RMT_TICK_10_US; 

	size_t rx_size = 0;
	RingbufHandle_t rb = NULL;
	rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
	rmt_rx_start(RMT_RX_CHANNEL, 1);

	distance = 0;

	for (i=0; i<num_measurements; i++){
		rmt_write_items(RMT_TX_CHANNEL, &item, 1, true);
		rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);

		rmt_item32_t* item = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, 1000);
		distance += 100 * 340.29 * ITEM_DURATION(item->duration0) / (1000 * 1000 * 2); // distance in meters
		vRingbufferReturnItem(rb, (void*) item);
		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
	//vRingbufferDelete(rb);
	distance = distance/num_measurements;
	return distance;

}

void wifi_init_sta()
{
	s_wifi_event_group = xEventGroupCreate();

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL) );

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = EXAMPLE_ESP_WIFI_SSID,
			.password = EXAMPLE_ESP_WIFI_PASS
		},
	};

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
	ESP_ERROR_CHECK(esp_wifi_start() );

	ESP_LOGI(TAG, "wifi_init_sta finished.");
	ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
			 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}


void post_distance(float distance)
{

	esp_http_client_config_t config = {
		.url = "https://tranquil-forest-64117.herokuapp.com/waterlevels/",
		.event_handler = _http_event_handler,
	};

	esp_http_client_handle_t client = esp_http_client_init(&config);

	esp_err_t err = esp_http_client_perform(client);

	//char *post_data; //= "{\"water_level\": 0}"
	char *post_data = malloc(25);

	sprintf(post_data, "{\"water_level\": %.2f}", distance);
	
	esp_http_client_set_url(client, "https://tranquil-forest-64117.herokuapp.com/waterlevels/");
	esp_http_client_set_method(client, HTTP_METHOD_POST);
	esp_http_client_set_post_field(client, post_data, strlen(post_data));
	esp_http_client_set_header(client, "Content-Type", "application/json");
	err = esp_http_client_perform(client);
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
				esp_http_client_get_status_code(client),
				esp_http_client_get_content_length(client));
	} else {
		ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
	}
	free(post_data);
	esp_http_client_close(client);
	esp_http_client_cleanup(client);
}

void toggle_led()
{

    if (gpio_get_level(ONBOARD_LED) == 1) {
    	printf("LED ON, NOW OFF\n");
    	gpio_set_level(ONBOARD_LED, 0);
    } else if (gpio_get_level(ONBOARD_LED) == 0){
    	printf("LED OFF, NOW ON\n");
    	gpio_set_level(ONBOARD_LED, 1);
    } else {
    	printf("ESCANGALHOU\n");
    }

}

void init_main()
{
	printf("Config: IO\n");
	/*gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);
	gpio_set_level(PIN_RELAY, 0);*/

	gpio_pad_select_gpio(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(ONBOARD_LED, 0);
    gpio_set_level(ONBOARD_LED, 1);

    gpio_pad_select_gpio(PIN_RELAY);
    gpio_set_direction(PIN_RELAY, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RELAY, 0);

	printf("Init: HCSR04\n");
	HCSR04_init();

	printf("Init: System Flash\n");
	esp_err_t ret = nvs_flash_init();
	system_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
	printf("Init: WiFi\n");

	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	wifi_init_sta();

	while (!wifi_connected) {
		printf("Conectando WiFi...\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	printf("Check Efuse\n");
	check_efuse();

	printf("Init MQTT\n");
	mqtt_app_start();

	//relaystatus.minDistance = HCSR04_measure(20);
	relaystatus.minDistance = 42.56;
	relaystatus.relayActive = false;
	relaystatus.relayPeriod = 10;
	relaystatus.relayDutyCycle = 50;

}

void WaterMeasurementTask(void * pvParameters)
{
	double distance;
	uint32_t num_measurements;
	uint32_t measurements_interval_s;

	num_measurements = 10;
	measurements_interval_s = 5;

	distance = 123;
	while(1){
		//distance = HCSR04_measure(num_measurements);
		distance = 42.56;
		printf("Distance:%f\n", distance);
		//post_distance((float)distance);
		vTaskDelay(measurements_interval_s*1000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(0);
}

void process_mqtt_message(void *pvParameters)
{
	char* topic = "/topic/relay";
	char* data = "1";

	mqtt_msg_struct_t imqtt_msg = *(mqtt_msg_struct_t *) pvParameters;

	if (strncmp(imqtt_msg.topic, topic, imqtt_msg.topic_len) == 0) {
		if (strncmp(imqtt_msg.data, data, imqtt_msg.data_len) == 0) {
			if (!relaystatus.relayActive) {
				xTaskCreate(RelayController, "Relay Controller", 2048, NULL, 1, NULL);
				relaystatus.relayActive = true;
			} else {
				printf("Valvula ja em funcionamento!!\n");
			}
		}
	}
	vTaskDelete(0);
}

void RelayController(void *pvParameters)
{
	float distance;
	float on_time = relaystatus.relayDutyCycle/100*relaystatus.relayPeriod;
	float off_time = relaystatus.relayPeriod - on_time;

	//distance = HCSR04_measure(20);
	distance = 45.56;

	while (distance > relaystatus.minDistance) {
		gpio_set_level(PIN_RELAY, 1);
		vTaskDelay(on_time*1000 / portTICK_PERIOD_MS);
		gpio_set_level(PIN_RELAY, 0);
		vTaskDelay(off_time*1000 / portTICK_PERIOD_MS);
		//distance = HCSR04_measure(10);
		distance = 46.78;
	}

	gpio_set_level(PIN_RELAY, 0);
	relaystatus.relayActive = false;
	vTaskDelete(0);
}

void PlantSensorTask(void *pvParameters)
{
	int i = 0;

	char buf[100];
	
	if (unit == ADC_UNIT_1) {
		adc1_config_width(ADC_WIDTH_BIT_12);
		adc1_config_channel_atten(channel, atten);
	} else {
		adc2_config_channel_atten((adc2_channel_t)channel, atten);
	}

	//Characterize ADC
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
	print_char_val_type(val_type);

	uint32_t adc_reading = 0;
	uint32_t voltage;

	while(1){
		//ADC READ
		for (int i = 0; i < NO_OF_SAMPLES; i++) {
			if (unit == ADC_UNIT_1) {
				adc_reading += adc1_get_raw((adc2_channel_t)channel);
			} else {
				int raw;
				adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
				adc_reading += raw;
			}
		}
		adc_reading /= NO_OF_SAMPLES;
		//Convert adc_reading to voltage in mV
		voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
		printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);

		sprintf(buf, "%d", i);
		esp_mqtt_client_publish(mqtt_client, "/topic/plant1", buf, 0, 0, 0);
		vTaskDelay(2*1000 / portTICK_PERIOD_MS);
		i++;
	}
	vTaskDelete(0);
}


void app_main()
{

	init_main();

	xTaskCreate(WaterMeasurementTask, "Water Measurement Task", 2048, NULL, 1, NULL);
	xTaskCreate(PlantSensorTask, "Plant Sensor Task", 2048, NULL, 1, NULL);
	
	
}

