#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include "driver/adc.h"

#define BUTTON_PIN_BITMASK 0x10

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
IPAddress ip(192, 168, 1, 5);

unsigned long lastMsg = 0;
unsigned long first = millis();
String mac, dis_json, dis_topic, state_topic;

volatile unsigned long last_changed = 0;
volatile boolean needs_update = false;

void connectToNetwork()
{
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1);
  }
  Serial.println(WiFi.localIP());
  Serial.println("Connected to network");
}

void set_states()
{
  dis_json = "{\"name\": \"door_sensor_" + mac + "\", \"device_class\": \"door\", \"state_topic\": \"door_" + mac + "/state\", \"unique_id\": \"door_sensor_" + mac + "\"}";
  dis_topic = "homeassistant/binary_sensor/door_" + mac + "/config";
  state_topic = "door_" + mac + "/state";
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }

  Serial.println();
}

void set_mac_string()
{
  byte mact[6];
  WiFi.macAddress(mact);
  String s;

  for (byte i = 3; i < 6; ++i)
  {
    char buf[2];
    sprintf(buf, "%02X", mact[i]);
    s += buf;
  }
  mac = s;
}

void door_changed()
{
  last_changed = millis();
  needs_update = true;
}

void start_sleep()
{
  Serial.println("Going to sleep...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  adc_power_off();
  esp_wifi_stop();
  esp_bt_controller_disable();
  Serial.flush();
  delay(1000);
  // Go to sleep! Zzzz
  esp_deep_sleep_start();
}

void print_GPIO_wake_up()
{
  int GPIO_reason = esp_sleep_get_ext1_wakeup_status();
  Serial.print("GPIO that triggered the wake up: GPIO ");
  Serial.println((log(GPIO_reason)) / log(2), 0);
}

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  connectToNetwork();
  mqtt_client.setServer(ip, 1883);
  mqtt_client.setCallback(mqtt_callback);
  set_mac_string();
  set_states();
  Serial.println(mac);

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  //Print the GPIO used to wake up
  print_GPIO_wake_up();

  if (mqtt_client.connect("door_sensor", MQTT_USER, MQTT_PASS))
  {
    Serial.print("MQTT OK in millis ");
    Serial.println(millis() - first);
    mqtt_client.subscribe("inTest");
    mqtt_client.publish(dis_topic.c_str(), dis_json.c_str());
    if (digitalRead(4) == HIGH)
    {
      mqtt_client.publish(state_topic.c_str(), "OFF");
    }
    else
    {
      mqtt_client.publish(state_topic.c_str(), "ON");
    }
  }
  attachInterrupt(digitalPinToInterrupt(4), door_changed, CHANGE);
}

void loop()
{
  mqtt_client.loop();

  if (needs_update && millis() - last_changed > 50)
  {
    if (digitalRead(4) == HIGH)
    {
      mqtt_client.publish(state_topic.c_str(), "OFF");
    }
    else
    {
      mqtt_client.publish(state_topic.c_str(), "ON");
    }
    needs_update = false;
  }

  if (millis() - last_changed > 10000)
  {
    if (digitalRead(4) == HIGH)
    {
      Serial.print("State is ");
      Serial.print(digitalRead(4));
      Serial.println(" wake up for all low");
      // esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);
      esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);
    }
    else
    {
      Serial.print("State is ");
      Serial.print(digitalRead(4));
      Serial.println(" wake up for any high");
      // esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1);
      esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
    }
    start_sleep();
  }
}