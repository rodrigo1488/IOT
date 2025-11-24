#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "DHT.h"

// === Vital Charger IoT ===
// Projeto de monitoramento remoto de sinais vitais com ESP32 + MQTT.

#define DHTPIN 21
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

const int pinADC = 34;    // Potenciômetro (simula ECG)
const int pinLED = 2;     // LED de alerta
const int pinBuzzer = 26; // Buzzer

// Limiares simulados
const int BPM_LOW = 40;
const int BPM_HIGH = 120;
const float TEMP_HIGH = 37.5;

// Wi-Fi / MQTT
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";
const char* MQTT_HOST = "test.mosquitto.org";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_TELEMETRY_TOPIC = "vitalcharger/telemetry";
const char* MQTT_ALERT_TOPIC = "vitalcharger/alert";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastPublish = 0;

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("Conectando em ");
  Serial.print(WIFI_SSID);
  Serial.println(" ...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("\nWi-Fi pronto, IP: ");
  Serial.println(WiFi.localIP());
}

void connectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Conectando ao broker MQTT...");
    String clientId = "VitalCharger-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" conectado!");
    } else {
      Serial.print(" falhou, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" tentando novamente em 2s");
      delay(2000);
    }
  }
}

void publishTelemetry(float bpm, float temp, float hum, bool alerta, const String& tipo) {
  StaticJsonDocument<256> doc;
  doc["bpm"] = bpm;
  doc["temperature"] = temp;
  doc["humidity"] = hum;
  doc["alert"] = alerta;
  doc["alertReason"] = tipo;

  char payload[256];
  serializeJson(doc, payload);
  mqttClient.publish(MQTT_TELEMETRY_TOPIC, payload);

  if (alerta) {
    mqttClient.publish(MQTT_ALERT_TOPIC, payload);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(pinLED, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  dht.begin();

  connectWiFi();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
}

void loop() {
  connectWiFi();
  if (!mqttClient.connected()) {
    connectMqtt();
  }
  mqttClient.loop();

  const int rawECG = analogRead(pinADC); // valor do potenciômetro
  const float temp = dht.readTemperature();
  const float hum = dht.readHumidity();

  // Converter sinal analógico para BPM aproximado
  const int bpm = map(rawECG, 0, 4095, 50, 130);

  bool alerta = false;
  String tipo = "";

  if (bpm < BPM_LOW || bpm > BPM_HIGH) {
    alerta = true;
    tipo = "BPM fora da faixa";
  } else if (temp > TEMP_HIGH) {
    alerta = true;
    tipo = "Temperatura alta";
  }

  if (alerta) {
    digitalWrite(pinLED, HIGH);
    tone(pinBuzzer, 1000, 300);
  } else {
    digitalWrite(pinLED, LOW);
    noTone(pinBuzzer);
  }

  if (millis() - lastPublish > 3000) {
    lastPublish = millis();
    Serial.println("==========================");
    Serial.printf("BPM: %d\n", bpm);
    Serial.printf("Temp: %.1f °C | Umidade: %.1f %%\n", temp, hum);
    if (alerta) {
      Serial.print("⚠ ALERTA: ");
      Serial.println(tipo);
    } else {
      Serial.println("Status normal");
    }

    publishTelemetry(bpm, temp, hum, alerta, tipo);
  }

  delay(100);
}
