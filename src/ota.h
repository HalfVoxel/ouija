
#pragma once
#include <ElegantOTA.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>

WebServer httpServer(80);

const char *ssid = "Octagon 2.4";
const char *password = "qwertydvorak";

void monitorOTA(void *param) {
  while (true) {
    // Serial.println("OTA loop");
    httpServer.handleClient();
    ElegantOTA.loop();
  }
}

bool checkOTAIsActive() {
  HTTPClient http;
  http.setConnectTimeout(100);
  http.begin("http://192.168.1.128:9123");
  auto responseCode = http.GET();
  http.end();
  if (responseCode == 200) {
    Serial.println("OTA active");
    return true;
  } else {
    Serial.println("OTA inactive");
    return false;
  }
}

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 126);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 224, 0);

void setupOTA() {
  Serial.println("Connecting to WiFi");
  WiFi.persistent(true);
  WiFi.setHostname("ESP32");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  Serial.print("Connecting to WiFi.");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Doing a GET request immediately after connecting seems to fail for unclear reasons
  delay(500);

  if (checkOTAIsActive()) {
    // TODO: Blink LED
    httpServer.on("/", []() { httpServer.send(200, "text/plain", "Hi! This is ElegantOTA Demo."); });
    httpServer.begin();
    ElegantOTA.begin(&httpServer);
    monitorOTA(nullptr);
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
}
