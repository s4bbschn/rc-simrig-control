/**
 * ESP32 ESC/Servo Controller
 * 
 * Steuert ein RC-Car ESC (Traxxas Drifter etc.) und einen Lenkservo
 * direkt per PWM vom ESP32.
 * 
 * Features:
 * - WiFi Access Point mit Web-UI
 * - WebSocket für Echtzeit-Steuerung
 * - REST API für zukünftige Pi-Integration
 * - ESC Arming-Sequenz (plattformabhängig)
 * - Sicherheits-Timeout (Failsafe auf Neutral)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <ESP32Servo.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

// ============================================================
// Pin-Konfiguration
// ============================================================
#define SERVO_STEERING_PIN  16   // GPIO für Lenkservo
#define SERVO_THROTTLE_PIN  19   // GPIO für ESC (Throttle)
#define LED_STATUS_PIN      2    // Onboard-LED für Status

// ============================================================
// PWM-Einstellungen
// ============================================================
#define PWM_FREQUENCY       50   // 50Hz = Standard für RC-Servos/ESCs
#define PWM_RESOLUTION      16   // 16-bit Auflösung

// ============================================================
// ESC-Plattformen
// ============================================================
enum ESCPlatform {
    PLATFORM_TRAXXAS = 0,
    PLATFORM_HOBBYWING,
    PLATFORM_GENERIC,
    PLATFORM_COUNT
};

struct ESCProfile {
    const char* name;
    int neutralPulse;       // Neutral-Position (µs)
    int minPulse;           // Volle Bremse/Rückwärts (µs)
    int maxPulse;           // Vollgas (µs)
    int armingDelay;        // Zeit für Arming in ms
    bool needsBrakeBeforeReverse;  // Traxxas: erst Bremse, dann Rückwärts
    int brakeThreshold;     // Ab welchem negativen Wert wird gebremst (µs unter neutral)
};

// ESC-Profile
const ESCProfile escProfiles[PLATFORM_COUNT] = {
    // Traxxas XL-5 / VXL: Neutral=1500, braucht Brems-Sequenz für Rückwärts
    {"Traxxas", 1500, 1000, 2000, 3000, true, 50},
    // Hobbywing QuicRun: ähnlich, aber ohne Brems-Pflicht
    {"Hobbywing", 1500, 1000, 2000, 2000, false, 0},
    // Generic: Standard-Werte
    {"Generic", 1500, 1000, 2000, 2000, false, 0},
};

// ============================================================
// WiFi-Konfiguration
// ============================================================
// STA-Modus: Versucht sich in dieses bestehende Netzwerk einzuwählen
const char* STA_SSID = "RC-ESC-Controller";   // <-- Hier Handy-Hotspot SSID eintragen
const char* STA_PASS = "rc123456";            // <-- Hier Handy-Hotspot Passwort eintragen
const int   STA_MAX_RETRIES = 15;             // Versuche à 500ms = 7.5 Sek
const char* HOSTNAME = "rc-esc";              // mDNS Hostname (.local)

// AP-Modus: Fallback wenn kein bestehendes Netz gefunden
const char* AP_SSID = "RC-ESC-Controller";
const char* AP_PASS = "rc123456";

// Status
bool wifiIsStation = false;  // true = im bestehenden Netz, false = eigener AP

// ============================================================
// Globale Variablen
// ============================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Servo steeringServo;
Servo throttleServo;

ESCPlatform currentPlatform = PLATFORM_TRAXXAS;
bool escArmed = false;
bool armingInProgress = false;

// Aktuelle Werte (-1000 bis +1000, wird auf Pulsweite gemappt)
int currentSteering = 0;    // -1000=links, 0=mitte, +1000=rechts
int currentThrottle = 0;    // -1000=rückwärts/bremse, 0=neutral, +1000=vollgas

// Smoothing: Zielwerte und aktuelle interpolierte Werte
int targetSteering = 0;
int targetThrottle = 0;
float smoothSteering = 0.0f;
float smoothThrottle = 0.0f;
const float STEERING_SMOOTHING = 0.15f;  // 0.0=kein Smooth, 1.0=sofort (je kleiner desto weicher)
const float THROTTLE_SMOOTHING = 0.20f;  // Throttle etwas direkter als Lenkung

// Failsafe
unsigned long lastCommandTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 2000;  // 2 Sekunden ohne Befehl → Neutral

// Traxxas Rückwärts-Logik
enum TraxxasReverseState {
    REVERSE_IDLE,
    REVERSE_BRAKE_SENT,
    REVERSE_NEUTRAL_SENT,
    REVERSE_ACTIVE
};
TraxxasReverseState reverseState = REVERSE_IDLE;
unsigned long reverseStateTime = 0;

// ============================================================
// Hilfsfunktionen
// ============================================================

/**
 * Mappt einen Wert (-1000..+1000) auf die Pulsweite für das ESC/Servo
 */
int mapToPulse(int value, int minPulse, int neutralPulse, int maxPulse) {
    if (value == 0) return neutralPulse;
    if (value > 0) {
        return map(value, 0, 1000, neutralPulse, maxPulse);
    } else {
        return map(value, -1000, 0, minPulse, neutralPulse);
    }
}

/**
 * Setzt die Lenkung
 */
void setSteering(int value) {
    value = constrain(value, -1000, 1000);
    currentSteering = value;
    targetSteering = value;
    Serial.printf("[Steering] value=%d pulse=%d\n", value, map(value, -1000, 1000, 1000, 2000));
}

/**
 * Setzt das Throttle (mit Traxxas-Rückwärts-Logik)
 */
void setThrottle(int value) {
    value = constrain(value, -1000, 1000);
    currentThrottle = value;
    targetThrottle = value;
    
    if (!escArmed) return;
    
    const ESCProfile& profile = escProfiles[currentPlatform];

    // Traxxas Rückwärts-Handling
    if (profile.needsBrakeBeforeReverse && value < 0) {
        if (reverseState == REVERSE_IDLE) {
            reverseState = REVERSE_BRAKE_SENT;
            reverseStateTime = millis();
        }
    } else if (value >= 0) {
        reverseState = REVERSE_IDLE;
    }
}

/**
 * ESC Arming-Sequenz
 */
void armESC() {
    if (armingInProgress || escArmed) return;
    
    armingInProgress = true;
    const ESCProfile& profile = escProfiles[currentPlatform];
    
    Serial.printf("[ESC] Arming '%s' — sende Neutral (%d µs) für %d ms...\n",
                  profile.name, profile.neutralPulse, profile.armingDelay);
    
    // Neutral-Signal senden
    throttleServo.writeMicroseconds(profile.neutralPulse);
    
    // Nicht-blockierendes Arming (wird im Loop geprüft)
    // Für den ersten Test machen wir es blockierend
    delay(profile.armingDelay);
    
    escArmed = true;
    armingInProgress = false;
    Serial.println("[ESC] Armed!");
    
    // Status an alle WebSocket-Clients senden
    JsonDocument doc;
    doc["type"] = "status";
    doc["armed"] = true;
    doc["platform"] = profile.name;
    String json;
    serializeJson(doc, json);
    ws.textAll(json);
}

/**
 * ESC Disarm
 */
void disarmESC() {
    const ESCProfile& profile = escProfiles[currentPlatform];
    throttleServo.writeMicroseconds(profile.neutralPulse);
    escArmed = false;
    reverseState = REVERSE_IDLE;
    currentThrottle = 0;
    
    Serial.println("[ESC] Disarmed!");
    
    JsonDocument doc;
    doc["type"] = "status";
    doc["armed"] = false;
    doc["platform"] = profile.name;
    String json;
    serializeJson(doc, json);
    ws.textAll(json);
}

// ============================================================
// WebSocket Handler
// ============================================================

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("[WS] Client #%u verbunden von %s\n", client->id(),
                          client->remoteIP().toString().c_str());
            {
                // Status senden
                JsonDocument doc;
                doc["type"] = "status";
                doc["armed"] = escArmed;
                doc["platform"] = escProfiles[currentPlatform].name;
                doc["steering"] = currentSteering;
                doc["throttle"] = currentThrottle;
                String json;
                serializeJson(doc, json);
                client->text(json);
            }
            break;
            
        case WS_EVT_DISCONNECT:
            Serial.printf("[WS] Client #%u getrennt\n", client->id());
            break;
            
        case WS_EVT_DATA: {
            AwsFrameInfo* info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                data[len] = 0;  // Null-terminate
                
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, (char*)data);
                if (error) {
                    Serial.printf("[WS] JSON Parse Error: %s\n", error.c_str());
                    return;
                }
                
                const char* cmd = doc["cmd"];
                if (!cmd) return;
                
                lastCommandTime = millis();
                
                if (strcmp(cmd, "steer") == 0) {
                    int value = doc["value"] | 0;
                    setSteering(value);
                }
                else if (strcmp(cmd, "throttle") == 0) {
                    int value = doc["value"] | 0;
                    setThrottle(value);
                }
                else if (strcmp(cmd, "arm") == 0) {
                    armESC();
                }
                else if (strcmp(cmd, "disarm") == 0) {
                    disarmESC();
                }
                else if (strcmp(cmd, "platform") == 0) {
                    int p = doc["value"] | 0;
                    if (p >= 0 && p < PLATFORM_COUNT) {
                        disarmESC();  // Erst disarmen bei Plattformwechsel
                        currentPlatform = (ESCPlatform)p;
                        Serial.printf("[ESC] Plattform gewechselt: %s\n",
                                      escProfiles[currentPlatform].name);
                    }
                }
                
                // Nur bei arm/disarm/platform ein ack senden, NICHT bei steer/throttle
                // (zu viele Antworten überlasten den WS-Buffer)
                if (strcmp(cmd, "arm") == 0 || strcmp(cmd, "disarm") == 0 || strcmp(cmd, "platform") == 0) {
                    JsonDocument resp;
                    resp["type"] = "ack";
                    resp["steering"] = currentSteering;
                    resp["throttle"] = currentThrottle;
                    resp["armed"] = escArmed;
                    String json;
                    serializeJson(resp, json);
                    client->text(json);
                }
            }
            break;
        }
        
        default:
            break;
    }
}

// ============================================================
// REST API (für zukünftige Pi-Integration)
// ============================================================

void setupAPI() {
    // Status abrufen
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
        JsonDocument doc;
        doc["armed"] = escArmed;
        doc["platform"] = escProfiles[currentPlatform].name;
        doc["platformId"] = (int)currentPlatform;
        doc["steering"] = currentSteering;
        doc["throttle"] = currentThrottle;
        doc["uptime"] = millis() / 1000;
        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
    });
    
    // Steuerbefehl senden (POST)
    server.on("/api/control", HTTP_POST, [](AsyncWebServerRequest* request) {},
        NULL,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, (char*)data);
            if (error) {
                request->send(400, "application/json", "{\"error\":\"invalid json\"}");
                return;
            }
            
            lastCommandTime = millis();
            
            if (doc.containsKey("steering")) {
                setSteering(doc["steering"] | 0);
            }
            if (doc.containsKey("throttle")) {
                setThrottle(doc["throttle"] | 0);
            }
            
            JsonDocument resp;
            resp["status"] = "ok";
            resp["steering"] = currentSteering;
            resp["throttle"] = currentThrottle;
            String json;
            serializeJson(resp, json);
            request->send(200, "application/json", json);
        }
    );
    
    // Arm/Disarm
    server.on("/api/arm", HTTP_POST, [](AsyncWebServerRequest* request) {
        armESC();
        request->send(200, "application/json", "{\"status\":\"armed\"}");
    });
    
    server.on("/api/disarm", HTTP_POST, [](AsyncWebServerRequest* request) {
        disarmESC();
        request->send(200, "application/json", "{\"status\":\"disarmed\"}");
    });
    
    // Plattform-Liste
    server.on("/api/platforms", HTTP_GET, [](AsyncWebServerRequest* request) {
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        for (int i = 0; i < PLATFORM_COUNT; i++) {
            JsonObject obj = arr.add<JsonObject>();
            obj["id"] = i;
            obj["name"] = escProfiles[i].name;
            obj["neutralPulse"] = escProfiles[i].neutralPulse;
            obj["minPulse"] = escProfiles[i].minPulse;
            obj["maxPulse"] = escProfiles[i].maxPulse;
            obj["needsBrakeBeforeReverse"] = escProfiles[i].needsBrakeBeforeReverse;
        }
        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
    });
    
    // Plattform setzen
    server.on("/api/platform", HTTP_POST, [](AsyncWebServerRequest* request) {},
        NULL,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            JsonDocument doc;
            deserializeJson(doc, (char*)data);
            int p = doc["platform"] | -1;
            if (p >= 0 && p < PLATFORM_COUNT) {
                disarmESC();
                currentPlatform = (ESCPlatform)p;
                request->send(200, "application/json", "{\"status\":\"ok\"}");
            } else {
                request->send(400, "application/json", "{\"error\":\"invalid platform\"}");
            }
        }
    );
}

// ============================================================
// Setup
// ============================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== ESP32 RC ESC Controller ===");
    
    // LED
    pinMode(LED_STATUS_PIN, OUTPUT);
    digitalWrite(LED_STATUS_PIN, LOW);
    
    // SPIFFS für Web-UI
    if (!SPIFFS.begin(true)) {
        Serial.println("[ERROR] SPIFFS Mount fehlgeschlagen!");
    }
    
    // Servos initialisieren
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    
    steeringServo.setPeriodHertz(PWM_FREQUENCY);
    throttleServo.setPeriodHertz(PWM_FREQUENCY);
    
    steeringServo.attach(SERVO_STEERING_PIN, 1000, 2000);
    throttleServo.attach(SERVO_THROTTLE_PIN, 1000, 2000);
    
    // Neutral-Position
    steeringServo.writeMicroseconds(1500);
    throttleServo.writeMicroseconds(escProfiles[currentPlatform].neutralPulse);
    
    Serial.printf("[Servo] Steering: GPIO %d\n", SERVO_STEERING_PIN);
    Serial.printf("[Servo] Throttle: GPIO %d\n", SERVO_THROTTLE_PIN);
    
    // ============================================================
    // WiFi: Erst versuchen sich in bestehendes Netz einzuwählen,
    // dann Fallback auf eigenen AP
    // ============================================================
    WiFi.setHostname(HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    
    Serial.printf("[WiFi] Suche Netzwerk '%s'...\n", STA_SSID);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < STA_MAX_RETRIES) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        // Erfolgreich im bestehenden Netz
        wifiIsStation = true;
        Serial.printf("[WiFi] Verbunden mit '%s'\n", STA_SSID);
        Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[WiFi] Hostname: %s.local\n", HOSTNAME);
    } else {
        // Fallback: Eigenen AP aufmachen
        wifiIsStation = false;
        WiFi.disconnect();
        WiFi.mode(WIFI_AP);
        delay(100);
        
        bool apStarted = WiFi.softAP(AP_SSID, AP_PASS);
        if (apStarted) {
            Serial.printf("[WiFi] AP gestartet: %s (Passwort: %s)\n", AP_SSID, AP_PASS);
            Serial.printf("[WiFi] IP: %s\n", WiFi.softAPIP().toString().c_str());
        } else {
            Serial.println("[ERROR] WiFi AP konnte NICHT gestartet werden!");
            delay(1000);
            WiFi.softAP(AP_SSID, AP_PASS);
            Serial.printf("[WiFi] Retry — IP: %s\n", WiFi.softAPIP().toString().c_str());
        }
    }
    
    // mDNS starten (rc-esc.local)
    if (MDNS.begin(HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("ws", "tcp", 80);
        Serial.printf("[mDNS] Erreichbar unter: http://%s.local\n", HOSTNAME);
    } else {
        Serial.println("[mDNS] Fehler beim Start");
    }
    
    // WebSocket
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    
    // REST API
    setupAPI();
    
    // Web-UI aus SPIFFS servieren
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    
    // Fallback: eingebettete Web-UI wenn kein SPIFFS
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (!SPIFFS.exists("/index.html")) {
            request->send(200, "text/html", 
                "<html><body><h1>ESP32 ESC Controller</h1>"
                "<p>SPIFFS nicht gefunden. Bitte 'pio run -t uploadfs' ausfuehren.</p>"
                "</body></html>");
        }
    });
    
    server.begin();
    Serial.println("[HTTP] Server gestartet auf Port 80");
    if (wifiIsStation) {
        Serial.println("\n--- Bereit! ESP32 im Netzwerk '" + String(STA_SSID) + "' ---");
        Serial.println("--- Öffne http://" + WiFi.localIP().toString() + " oder http://" + String(HOSTNAME) + ".local ---\n");
    } else {
        Serial.println("\n--- Bereit! Verbinde dich mit WiFi '" + String(AP_SSID) + "' ---");
        Serial.println("--- Dann öffne http://192.168.4.1 ---\n");
    }
    
    // OTA Setup
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.setPassword("rc123456");
    ArduinoOTA.onStart([]() {
        // Servos auf Neutral bei OTA-Update
        steeringServo.writeMicroseconds(1500);
        throttleServo.writeMicroseconds(1500);
        escArmed = false;
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "Firmware" : "Filesystem";
        Serial.println("[OTA] Start: " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Fertig! Neustart...");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("[OTA] Bereit (Hostname: rc-esc-controller)");
    
    // ESC Arming NICHT mehr im Setup (blockiert WiFi), sondern verzögert
    // armESC() wird jetzt nach 2s im Loop aufgerufen
    
    digitalWrite(LED_STATUS_PIN, HIGH);
}

// ============================================================
// Loop
// ============================================================

void loop() {
    // OTA Handle
    ArduinoOTA.handle();
    
    // WebSocket Cleanup (max 8 Clients erlauben)
    ws.cleanupClients(8);
    
    // Verzögertes Auto-Arming (2s nach Boot, non-blocking)
    static bool autoArmDone = false;
    if (!autoArmDone && millis() > 2000) {
        autoArmDone = true;
        Serial.println("[ESC] Auto-Arming nach Boot...");
        armESC();
    }
    
    // ============================================================
    // Smoothing: Servo-Werte sanft interpolieren (läuft jeden ms)
    // ============================================================
    static unsigned long lastSmoothUpdate = 0;
    if (millis() - lastSmoothUpdate >= 1) {  // ~1000Hz Update-Rate
        lastSmoothUpdate = millis();
        
        // Exponentielles Smoothing (Lowpass-Filter)
        smoothSteering += (targetSteering - smoothSteering) * STEERING_SMOOTHING;
        smoothThrottle += (targetThrottle - smoothThrottle) * THROTTLE_SMOOTHING;
        
        // Steering Servo updaten
        int steerPulse = map((int)smoothSteering, -1000, 1000, 1000, 2000);
        steeringServo.writeMicroseconds(steerPulse);
        
        // Throttle Servo updaten (nur wenn armed)
        if (escArmed) {
            const ESCProfile& profile = escProfiles[currentPlatform];
            
            // Traxxas Rückwärts State-Machine
            if (profile.needsBrakeBeforeReverse && targetThrottle < 0) {
                if (reverseState == REVERSE_BRAKE_SENT) {
                    int brakePulse = mapToPulse((int)smoothThrottle, profile.minPulse, profile.neutralPulse, profile.maxPulse);
                    throttleServo.writeMicroseconds(brakePulse);
                    if (millis() - reverseStateTime > 100) {
                        throttleServo.writeMicroseconds(profile.neutralPulse);
                        reverseState = REVERSE_NEUTRAL_SENT;
                        reverseStateTime = millis();
                    }
                } else if (reverseState == REVERSE_NEUTRAL_SENT) {
                    throttleServo.writeMicroseconds(profile.neutralPulse);
                    if (millis() - reverseStateTime > 100) {
                        reverseState = REVERSE_ACTIVE;
                    }
                } else if (reverseState == REVERSE_ACTIVE) {
                    int throttlePulse = mapToPulse((int)smoothThrottle, profile.minPulse, profile.neutralPulse, profile.maxPulse);
                    throttleServo.writeMicroseconds(throttlePulse);
                }
            } else {
                int throttlePulse = mapToPulse((int)smoothThrottle, profile.minPulse, profile.neutralPulse, profile.maxPulse);
                throttleServo.writeMicroseconds(throttlePulse);
            }
        }
    }
    
    // Failsafe: wenn keine Befehle kommen → Neutral
    if (escArmed && lastCommandTime > 0 && (millis() - lastCommandTime > FAILSAFE_TIMEOUT)) {
        setThrottle(0);
        setSteering(0);
        lastCommandTime = 0;  // Nur einmal triggern
        Serial.println("[FAILSAFE] Timeout — auf Neutral gesetzt");
        
        JsonDocument doc;
        doc["type"] = "failsafe";
        doc["message"] = "Timeout - Neutral";
        String json;
        serializeJson(doc, json);
        ws.textAll(json);
    }
    
    // Status-LED blinken wenn nicht armed
    static unsigned long lastBlink = 0;
    if (!escArmed && millis() - lastBlink > 500) {
        digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        lastBlink = millis();
    }
    
    delay(1);  // CPU entlasten
}
