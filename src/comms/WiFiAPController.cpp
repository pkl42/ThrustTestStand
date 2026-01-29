#include "comms/WiFiAPController.h"
#include <esp_log.h>

WiFiAPController::WiFiAPController(const char *ssid, const char *password)
{
    this->ssid = ssid;
    this->password = password;
}

void WiFiAPController::setAPConfig(IPAddress ip, IPAddress gateway, IPAddress subnet)
{
    _ip = ip;
    _gateway = gateway;
    _subnet = subnet;
}

void WiFiAPController::begin()
{
    WiFi.mode(WIFI_AP);

    if (!WiFi.softAPConfig(_ip, _gateway, _subnet))
    {
        ESP_LOGE("WiFiAP", "AP IP configuration failed!");
    }
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    ESP_LOGI("WiFiAP", "Access Point started: %s", ssid);
    ESP_LOGI("WiFiAP", "IP-Adresse: %s", IP.toString().c_str());
}
