#ifndef WIFIAPCONTROLLER_H
#define WIFIAPCONTROLLER_H

#include <WiFi.h>

class WiFiAPController
{
public:
    WiFiAPController(const char *ssid, const char *password);

    void setAPConfig(IPAddress ip, IPAddress gateway, IPAddress subnet);

    void setLocalConfig(const char *local_ssid, const char *local_password);

    void begin();

    bool startMDNS(const char *hostname);

private:
    static const char *TAG;
    const char *ssid;
    const char *password;
    const char *local_ssid;
    const char *local_password;
    IPAddress _ip, _gateway, _subnet;
};

#endif // WIFIAPCONTROLLER_H
