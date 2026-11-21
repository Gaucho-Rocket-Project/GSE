#pragma once
#define WIFI_STA 1
struct FakeWiFi { struct { bool started() { return true; } } STA; void mode(int) {} void disconnect() {} };
extern FakeWiFi WiFi;
