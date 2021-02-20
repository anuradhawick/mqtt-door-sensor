#include <ArduinoJson.h>

#pragma once

std::pair<String, String> get_sensor_discovery_jsons(String mac)
{
    StaticJsonDocument<200> json;
    String jsonString;
    String topicString = "homeassistant/binary_sensor/door_" + mac + "/config";;
    std::pair<String, String> result;

    json["name"] = "door_sensor_" + mac;
    json["device_class"] = "door";
    json["state_topic"] = "door_" + mac + "/state";
    json["unique_id"] = "door_sensor_" + mac;
    
    serializeJson(json, jsonString);
    result = std::make_pair(topicString, jsonString);

    return result;
}

std::pair<String, String> get_battery_discovery_jsons(String mac)
{
    StaticJsonDocument<200> json;
    String jsonString;
    String topicString = "homeassistant/sensor/door_batt_" + mac + "/config";
    std::pair<String, String> result;

    json["name"] = "door_sensor_batt_" + mac;
    json["device_class"] = "battery";
    json["unique_id"] = "door_sensor_batt_" + mac;
    json["state_topic"] = "door_" + mac + "/batt";
    json["value_template"] = "{{ value }}";

    serializeJson(json, jsonString);
    result = std::make_pair(topicString, jsonString);

    return result;
}