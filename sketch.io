#include <WiFi.h>
#include "DHT.h"

// Simulação Vital Charger IoT
#define DHTPIN 21
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

const int pinADC = 34;    // Potenciômetro (simula ECG)
const int pinLED = 2;     // LED de alerta
const int pinBuzzer = 26; // Buzzer

// Limiares simulados
int BPM_LOW = 40;
int BPM_HIGH = 120;
float TEMP_HIGH = 37.5;

unsigned long lastPrint = 0;
int bpm = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pinLED, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  dht.begin();

  Serial.println("=== Vital Charger IoT (Modo Simulado) ===");
  Serial.println("Conectando Wi-Fi (simulado)...");
  delay(1000);
  Serial.println("Conectado! (simulação)");
}

void loop() {
  // Simular leituras
  int rawECG = analogRead(pinADC); // valor do potenciômetro
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  // Converter sinal analógico para BPM aproximado
  bpm = map(rawECG, 0, 4095, 50, 130);

  // Lógica de alerta
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

  // Impressão simulando publicação MQTT
  if (millis() - lastPrint > 3000) {
    lastPrint = millis();
    Serial.println("==========================");
    Serial.printf("BPM: %d\n", bpm);
    Serial.printf("Temp: %.1f °C | Umidade: %.1f %%\n", temp, hum);
    if (alerta) {
      Serial.print("⚠ ALERTA: ");
      Serial.println(tipo);
      Serial.println("(Mensagem seria enviada via MQTT)");
    } else {
      Serial.println("Status normal - dados enviados (simulação)");
    }
  }

  delay(100);
}
