#ifndef WIFIAPCONTROLLER_H
#define WIFIAPCONTROLLER_H

#include <WiFi.h>

class WiFiAPController
{
public:
    WiFiAPController(const char *ssid, const char *password);

    void setAPConfig(IPAddress ip, IPAddress gateway, IPAddress subnet);

    void begin();

private:
    const char *ssid;
    const char *password;
    IPAddress _ip, _gateway, _subnet;
};

#endif // WIFIAPCONTROLLER_H
