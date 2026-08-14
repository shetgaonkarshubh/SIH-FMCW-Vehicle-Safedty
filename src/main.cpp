#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <TJpg_Decoder.h>
#include <FastLED.h>

// --- CONFIGURATION ---
const char* WIFI_SSID     = "shubh"; 
const char* WIFI_PASS     = "12345678"; 

const char* CLIENT_ID     = "f05aa45315064d00ad2dc11564566428";
const char* CLIENT_SECRET = "c85e5a669b244035847b569d2da99ce6";
const char* REFRESH_TOKEN = "AQBR2_MpNZHHuTFSG2tZ81w8LcoyICZGS6aN2m8jO-APjcOXlOqOXNh2l7llZa-qNvFOACEoPjMIY9H-_I85txmwQLtp7GCYty5zDIjWhBjXMl-PG2ym18AQR0EIRoHrWDs";

#define DATA_PIN     13
#define NUM_LEDS     64
#define BRIGHTNESS   25
#define LED_TYPE     WS2812B
#define COLOR_ORDER  GRB

CRGB leds[NUM_LEDS];
String accessToken = "";
String lastImageUrl = "";

uint16_t XY(uint8_t x, uint8_t y) {
    if (y % 2 == 1) return (y * 8) + (7 - x);
    return (y * 8) + x;
}

bool tjpgOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
    if (x % 8 == 0 && y % 8 == 0) {
        uint8_t matrixX = x / 8;
        uint8_t matrixY = y / 8;
        
        if (matrixX < 8 && matrixY < 8) {
            uint16_t color = bitmap[0];
            uint8_t r = ((color >> 11) & 0x1F) * 8;
            uint8_t g = ((color >> 5) & 0x3F) * 4;
            uint8_t b = (color & 0x1F) * 8;
            
            leds[XY(matrixX, matrixY)] = CRGB(r, g, b);
        }
    }
    return true;
}

void refreshSpotifyToken() {
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(4);

    HTTPClient http;
    http.setTimeout(4000);
    http.begin(client, "https://accounts.spotify.com/api/token");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String body = "grant_type=refresh_token&refresh_token=" + String(REFRESH_TOKEN) +
                  "&client_id=" + String(CLIENT_ID) + "&client_secret=" + String(CLIENT_SECRET);

    int httpCode = http.POST(body);
    if (httpCode == HTTP_CODE_OK) {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, http.getString());
        if (!err) {
            accessToken = doc["access_token"].as<String>();
            Serial.println("Spotify Access Token Refreshed!");
        }
    } else {
        Serial.printf("Token Refresh Failed! HTTP %d\n", httpCode);
    }
    http.end();
    client.stop();
}

void downloadAndDisplayImage(String url) {
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(5);

    HTTPClient http;
    http.setTimeout(5000);
    http.begin(client, url);

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        int len = http.getSize();
        if (len > 0 && ESP.getFreeHeap() > (len + 4096)) {
            uint8_t* buffer = (uint8_t*)malloc(len);
            if (buffer) {
                WiFiClient* stream = http.getStreamPtr();
                int bytesRead = stream->readBytes(buffer, len);
                if (bytesRead > 0) {
                    FastLED.clear();
                    TJpgDec.drawJpg(0, 0, buffer, bytesRead);
                    FastLED.show();
                    Serial.println("Album cover updated successfully!");
                }
                free(buffer);
            }
        }
    } else {
        Serial.printf("Failed to download image (HTTP %d)\n", httpCode);
    }
    http.end();
    client.stop();
}

void fetchCurrentlyPlaying() {
    if (accessToken.length() == 0) {
        refreshSpotifyToken();
        if (accessToken.length() == 0) return;
    }

    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(4);

    HTTPClient http;
    http.setTimeout(4000);
    http.begin(client, "https://api.spotify.com/v1/me/player/currently-playing");
    http.addHeader("Authorization", "Bearer " + accessToken);

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, http.getString());

        if (!err && doc.containsKey("item") && !doc["item"].isNull()) {
            JsonObject album = doc["item"]["album"];
            JsonArray images = album["images"];
            
            if (images.size() > 0) {
                String imageUrl = images[images.size() - 1]["url"].as<String>();

                if (imageUrl != lastImageUrl && imageUrl.length() > 0) {
                    lastImageUrl = imageUrl;
                    Serial.println("New Album Art URL: " + imageUrl);
                    
                    // Terminate API connection before opening image socket
                    http.end();
                    client.stop();
                    
                    downloadAndDisplayImage(imageUrl);
                    return;
                }
            }
        }
    } else if (httpCode == 401) {
        accessToken = ""; // Token expired -> refresh next loop
    }
    
    http.end();
    client.stop();
}

void setup() {
    Serial.begin(115200);
    
    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show();

    TJpgDec.setJpgScale(1);
    TJpgDec.setCallback(tjpgOutput);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi!");
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        fetchCurrentlyPlaying();
    } else {
        WiFi.reconnect();
    }
    
    // Non-blocking wait that yields to FreeRTOS watchdog
    for (int i = 0; i < 30; i++) {
        delay(100);
        yield();
    }
}