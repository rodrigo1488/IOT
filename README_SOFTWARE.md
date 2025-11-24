## Documentação do Código (Vital Charger IoT)

### 1. Visão geral
O firmware roda em um ESP32 virtual no Wokwi e lê dois sensores:
potenciômetro (simula batimento cardíaco) e DHT22 (temperatura e umidade).
O código compara os valores com limites seguros, aciona LED/buzzer em alertas
e envia tudo para a internet via MQTT.

### 2. Arquivos importantes
- `sketch.io`: código Arduino/ESP32.
- `libraries.txt`: lista de bibliotecas que o Wokwi deve instalar.

### 3. Principais blocos do `sketch.io`
1. **Configurações** – definição de pinos, limites de BPM/temperatura e dados
   da rede Wi-Fi e do broker MQTT.
2. **`connectWiFi()`** – tenta se conectar ao Wi-Fi `Wokwi-GUEST` e só segue
   quando o ESP32 recebe um IP.
3. **`connectMqtt()`** – cria um identificador único e negocia sessão com o
   broker público `test.mosquitto.org`.
4. **`publishTelemetry()`** – monta um JSON com BPM, temperatura, umidade e
   motivo do alerta usando a biblioteca `ArduinoJson`, depois publica em dois
   tópicos MQTT (`vitalcharger/telemetry` e `vitalcharger/alert`).
5. **`loop()`** – roda sem parar; lê sensores, calcula BPM, decide se há alerta,
   comanda LED/buzzer e chama `publishTelemetry()` a cada 3 s.

### 4. Fluxo resumido
1. O Wokwi liga o ESP32 → `setup()` inicializa sensores e Wi-Fi.
2. `loop()` garante conexão Wi-Fi/MQTT.
3. Lê sensores → converte o valor analógico em BPM (escala 50–130).
4. Verifica limites:
   - BPM < 40 ou > 120 ⇒ alerta.
   - Temperatura > 37,5 °C ⇒ alerta.
5. Atualiza LED/buzzer e envia os dados em JSON.

### 5. Como rodar no Wokwi
1. Abra https://wokwi.com/, crie um projeto ESP32 e substitua os arquivos pelos
   deste diretório (`diagram.json`, `sketch.io`, `libraries.txt`).
2. Clique em **Start Simulation**.
3. Acompanhe o `Serial Monitor` para ver BPM/temperatura e status do MQTT.

### 6. Ajustando limites e mensagens
- **BPM**: altere `BPM_LOW` e `BPM_HIGH`.
- **Temperatura**: altere `TEMP_HIGH`.
- **Tópicos MQTT**: mude `MQTT_TELEMETRY_TOPIC` e `MQTT_ALERT_TOPIC`.
Salve o arquivo, reinicie a simulação e verifique se o cliente MQTT recebe os
novos dados.

### 7. Bibliotecas utilizadas
- `WiFi.h` – conecta o ESP32 ao Wi-Fi.
- `PubSubClient` – cuida do protocolo MQTT.
- `ArduinoJson` – ajuda a montar o texto JSON.
- `DHT sensor library` – faz a leitura de temperatura/umidade do DHT22.

### 8. Dicas para iniciantes
- Para simular febre, clique no DHT22 no Wokwi e aumente a temperatura.
- Para subir ou descer o BPM, gire o potenciômetro virtual.
- Se o terminal mostrar “rc = -2”, o broker MQTT pode estar fora do ar; espere
  alguns segundos ou use outro broker público.


