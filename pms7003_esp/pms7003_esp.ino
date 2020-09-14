#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "Arduino.h"

#include "pms7003.h"

#include "SoftwareSerial.h"

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#include <Wire.h>
#include <BME280I2C.h>

// PMS7003 pins
#define PIN_RX  D1
#define PIN_TX  D2

// BME280 pins
#define PIN_SDA D3
#define PIN_SCL D4

#define MEASURE_INTERVAL_MS 300000
#define STABILISING_MS 30000

#define MQTT_HOST   ""
#define MQTT_PORT   1883
#define MQTT_TOPIC  "homeassistant/sensor/dust"

static SoftwareSerial sensor(PIN_RX, PIN_TX);
static WiFiClient wifiClient;
static WiFiManager wifiManager;
static PubSubClient mqttClient(wifiClient);

static char esp_id[16];
static char device_name[20];
static char mqtt_state_topic[32];
static boolean have_bme280;
static BME280I2C bme280;

typedef struct {
    float temp;
    float hum;
    float pres;
} bme_meas_t;

struct aqi_break {
    float min;
    float max;
    int aqi_min;
    int aqi_max;
};

struct aqi_break pm_25_breakpoints[] = {
  {0.0, 12.0, 0, 50},
  {12.1, 35.4, 51, 100},
  {35.5, 55.4, 101, 150},
  {55.5, 150.4, 151, 200},
  {150.5, 250.4, 201, 300},
  {250.5, 350.4, 301, 400},
  {350.5, 500.0, 401, 500}
};

enum pms_state {IDLE, STABILISING, WAITING};

static pms_state state;

void setup(void)
{
    uint8_t txbuf[8];
    int txlen;

    // welcome message
    Serial.begin(115200);
    Serial.println("PMS7003 ESP reader");

    // get ESP id
    sprintf(esp_id, "%06X", ESP.getChipId());
    sprintf(device_name, "PMS7003-%s", esp_id);
    Serial.print("Device name: ");
    Serial.println(device_name);
    sprintf(mqtt_state_topic, "%s/state", MQTT_TOPIC);

    // connect to wifi or set up captive portal
    Serial.println("Starting WIFI manager ...");
    wifiManager.autoConnect(device_name);

    // initialize the sensor
    sensor.begin(9600);
    txlen = PmsCreateCmd(txbuf, sizeof(txbuf), PMS_CMD_AUTO_MANUAL, 0);
    sensor.write(txbuf, txlen);
    txlen = PmsCreateCmd(txbuf, sizeof(txbuf), PMS_CMD_ON_STANDBY, 0);
    sensor.write(txbuf, txlen);
    PmsInit();
    
    Wire.begin(PIN_SDA, PIN_SCL);
    
    have_bme280 = bme280.begin();
    if (have_bme280) {
        Serial.println("Found BME280 sensor.");
    }

    state = IDLE;

    Serial.println("setup() done");
}

static void mqtt_send_string(const char *topic, const char *string)
{
    if (!mqttClient.connected()) {
        Serial.println("Connecting to MQTT server " MQTT_HOST);
        mqttClient.setServer(MQTT_HOST, MQTT_PORT);
        mqttClient.connect(device_name, topic, 0, true, "{\"alive\":0}");
    }
    if (mqttClient.connected()) {
        Serial.print("Publishing ");
        Serial.print(string);
        Serial.print(" to ");
        Serial.print(topic);
        Serial.print("...");
        bool result = mqttClient.publish(topic, string);
        Serial.println(result ? "OK" : "FAIL");
    }
}

static void mqtt_send_config(const char *type, const char *name, const char *unit)
{
    static char json[256];
    char tmp[128];
    char topic[128];

    sprintf(topic, "%s%s/config", MQTT_TOPIC, name);

    sprintf(json, "{\"name\": \"%s\", \"state_topic\": \"%s\", \"value_template\": \"{{ value_json.%s.%s}}\", \"unique_id\": \"%s_%s\", \"unit_of_measurement\": \"%s\"}",
            name, mqtt_state_topic, type, name, device_name, name, unit);

    mqtt_send_string(topic, json);
}

static void mqtt_send_json(const char *topic, int alive, const pms_meas_t *pms, const bme_meas_t *bme)
{
    static char json[128];
    char tmp[128];
    int aqi = -1;

    for(int i = 0; i < sizeof pm_25_breakpoints; i++) {
      if (pms->concPM2_5_amb > pm_25_breakpoints[i].min && pms->concPM2_5_amb < pm_25_breakpoints[i].max) {
         aqi = ((pms->concPM2_5_amb - pm_25_breakpoints[i].min) * (pm_25_breakpoints[i].aqi_max - pm_25_breakpoints[i].aqi_min) / (pm_25_breakpoints[i].max - pm_25_breakpoints[i].min) + pm_25_breakpoints[i].aqi_min);
         break;
      }
    }
    
    // header
    strcpy(json, "{");

    // always send alive
    sprintf(tmp, "\"alive\":%d", alive);
    strcat(json, tmp);

    // PMS7003
    if (pms != NULL) {
        // AMB, "standard atmosphere" particle
        sprintf(tmp, ",\"pms7003\":{\"pm10\":%d,\"pm2_5\":%d,\"pm1_0\":%d, \"pm2_5aqi\":%d}",
                pms->concPM10_0_amb, pms->concPM2_5_amb, pms->concPM1_0_amb, aqi);
        strcat(json, tmp);
    }

    // BME280, other meteorological data
    if (bme != NULL) {
        sprintf(tmp, ",\"bme280\":{\"t\":%.1f,\"rh\":%.1f,\"p\":%.1f}",
                bme->temp, bme->hum, bme->pres / 100.0);
        strcat(json, tmp);
    }

    // footer
    strcat(json, "}");

    mqtt_send_string(topic, json);
}

void loop(void)
{
    static unsigned long last_sent = 0;
    static unsigned long fan_on = 0;
    static unsigned long alive_count = 0;
    unsigned long ms = millis();
    uint8_t txbuf[8];
    int txlen;

    // keep MQTT alive
    mqttClient.loop();

    switch (state) {
    case IDLE:
        if ((ms - last_sent) > MEASURE_INTERVAL_MS) {
            fan_on = ms;
            txlen = PmsCreateCmd(txbuf, sizeof(txbuf), PMS_CMD_ON_STANDBY, 1);
            sensor.write(txbuf, txlen);
            fan_on = ms;
            state = STABILISING;
        }
        break;
    case STABILISING:
        if ((ms - fan_on) > STABILISING_MS) {
            // consume any command responses that are in the serial buffer
            while (sensor.available()) {
                sensor.read();
            }
            // trigger a measurement
            txlen = PmsCreateCmd(txbuf, sizeof(txbuf), PMS_CMD_TRIG_MANUAL, 0);
            sensor.write(txbuf, txlen);
            state = WAITING;
        }
        break;
    case WAITING:
        // check for incoming measurement data
        while (sensor.available()) {
            uint8_t c = sensor.read();
            if (PmsProcess(c)) {
                // parse it
                pms_meas_t pms_meas;
                PmsParse(&pms_meas);

                mqtt_send_config("pms7003", "pm1_0", "µg/m³");
                mqtt_send_config("pms7003", "pm2_5", "µg/m³");
                mqtt_send_config("pms7003", "pm10", "µg/m³");
                mqtt_send_config("pms7003", "pm2_5aqi", "AQI");

                // read BME sensor
                bme_meas_t *bme280_p;
                if (have_bme280) {
                    bme_meas_t bme_meas;
                    bme280.read(bme_meas.pres, bme_meas.temp, bme_meas.hum);
                    bme280_p = &bme_meas;
                    mqtt_send_config("bme280", "t", "°C");
                    mqtt_send_config("bme280", "rh", "%");
                    mqtt_send_config("bme280", "p", "Pascals");
                } else {
                    bme280_p = NULL;
                }

                // publish it
                alive_count++;
                mqtt_send_json(mqtt_state_topic, alive_count, &pms_meas, bme280_p);

                // Shut down
                txlen = PmsCreateCmd(txbuf, sizeof(txbuf), PMS_CMD_ON_STANDBY, 0);
                sensor.write(txbuf, txlen);

                last_sent = ms;
                fan_on = 0;

                state = IDLE;
                break;
            }
        }
    }
}
