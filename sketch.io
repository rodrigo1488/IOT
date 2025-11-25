#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

// ---------------------- SIMULAÇÃO -------------------------
#define DHTPIN 21
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

const int pinADC = 34;    // Potenciômetro (simula ECG)
const int pinLED = 2;
const int pinBuzzer = 26;

int BPM_LOW = 40;
int BPM_HIGH = 120;
float TEMP_HIGH = 37.5;

unsigned long lastCheck = 0;
int bpm = 0;

bool alertaAnterior = false;   // evita enviar alerta repetido

// ---------------------- WIFI + MQTT -------------------------
const char* ssid = "Wokwi-GUEST";
const char* password = "";

const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient client(espClient);
// ------------------------------------------------------------

void conectaWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }
}

void conectaMQTT() {
  while (!client.connected()) {
    client.connect("VitalCharger_Rodrigo");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(pinLED, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);

  dht.begin();
  conectaWiFi();

  client.setServer(mqtt_server, 1883);
  conectaMQTT();

  Serial.println("=== Vital Charger IoT MQTT ===");
}

void loop() {
  if (!client.connected()) conectaMQTT();
  client.loop();

  // Simular leituras
  int rawECG = analogRead(pinADC);
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  bpm = map(rawECG, 0, 4095, 50, 130);

  bool alerta = false;
  String tipo = "";

  if (bpm < BPM_LOW || bpm > BPM_HIGH) {
    alerta = true;
    tipo = "BPM fora da faixa";
  } else if (temp > TEMP_HIGH) {
    alerta = true;
    tipo = "Temperatura alta";
  }

  // Ações locais (LED/Buzzer)
  if (alerta) {
    digitalWrite(pinLED, HIGH);
    tone(pinBuzzer, 1000, 300);
  } else {
    digitalWrite(pinLED, LOW);
    noTone(pinBuzzer);
  }

  // Checagem a cada 2 segundos
  if (millis() - lastCheck > 2000) {
    lastCheck = millis();

    // ------- ENVIA MQTT SOMENTE NA TRANSIÇÃO NORMAL → ALERTA -------
    if (alerta && !alertaAnterior) {

      String payload = "{";
      payload += "\"bpm\":" + String(bpm) + ",";
      payload += "\"temp\":" + String(temp) + ",";
      payload += "\"umidade\":" + String(hum) + ",";
      payload += "\"alerta\":true,";
      payload += "\"tipo\":\"" + tipo + "\"";
      payload += "}";

      client.publish("vital/charger/rodrigo", payload.c_str());

      Serial.println("🚨 ALERTA ENVIADO VIA MQTT:");
      Serial.println(payload);
    }

    // Debug local
    Serial.printf("BPM: %d | Temp: %.1f°C | Hum: %.1f%% | ALERTA: %s\n",
                  bpm, temp, hum, alerta ? "SIM" : "NÃO");
  }

  // Atualiza estado do alerta para evitar spam
  alertaAnterior = alerta;
}
