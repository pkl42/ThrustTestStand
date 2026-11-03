#include <ESPmDNS.h>
#include "comms/WiFiAPController.h"
#include <esp_log.h>

static const char *TAG = "WiFiAP";

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

void WiFiAPController::setLocalConfig(const char *local_ssid, const char *local_password)
{
    this->local_ssid = local_ssid;
    this->local_password = local_password;
}

void WiFiAPController::begin()
{
    WiFi.mode(WIFI_AP_STA); // WIFI_AP

    if (!WiFi.softAPConfig(_ip, _gateway, _subnet))
    {
        ESP_LOGE(TAG, "AP IP configuration failed!");
    }
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    ESP_LOGI(TAG, "Access Point started: %s", ssid);
    ESP_LOGI(TAG, "IP-Adresse: %s", IP.toString().c_str());

    WiFi.begin(local_ssid, local_password);

    ESP_LOGI(TAG, "Connecting Local Network WiFi...: %s", local_ssid);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000)
    {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        ESP_LOGI(TAG, "Local WiFi connection failed → staying in AP mode");
    }
    else
    {
        if (MDNS.begin("thruststand"))
        {
            MDNS.addService("http", "tcp", 80);
            ESP_LOGI(TAG, "mDNS responder started");
        }
        IPAddress localIP = WiFi.localIP();
        ESP_LOGI(TAG, "STA Connected - Local IP-Adresse: %s", localIP.toString().c_str());
    }
}

bool WiFiAPController::startMDNS(const char *hostname)
{
    if (!MDNS.begin(hostname))
    {
        Serial.println("Error starting mDNS");
        return false;
    }

    ESP_LOGI(TAG, "mDNS started: http://%s.local", hostname);

    return true;
}
