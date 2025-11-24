## 1. Descrição do funcionamento e uso
O **Vital Charger IoT** é um protótipo de monitoramento remoto de sinais vitais baseado em ESP32. O potenciômetro simula o sinal cardíaco (ECG) para gerar BPM e o sensor DHT22 mede temperatura corporal e umidade ambiente ao redor do paciente. O firmware compara as leituras com limiares configuráveis e publica telemetria e alertas via MQTT sobre TCP/IP. O LED vermelho e o buzzer também sinalizam localmente quando há anomalias. Para reproduzir, abra o projeto em [Wokwi](https://wokwi.com/), carregue `diagram.json` e `sketch.io`, conecte-se à rede padrão `Wokwi-GUEST` e inicie a simulação.

## 2. Software e documentação do código
- Arquivo principal: `sketch.io` (Arduino/ESP32).
- Bibliotecas usadas: `WiFi.h`, `PubSubClient`, `ArduinoJson`, `DHT sensor library`.
- Funções principais:
  - `connectWiFi()` mantém o ESP32 conectado ao AP `Wokwi-GUEST`.
  - `connectMqtt()` negocia sessão MQTT com `test.mosquitto.org`.
  - `publishTelemetry()` monta um JSON com BPM, temperatura, umidade e status de alerta e publica em `vitalcharger/telemetry` e `vitalcharger/alert`.
  - `loop()` realiza as leituras simuladas, aplica regras de negócio e chama o envio MQTT a cada 3 segundos.

## 3. Hardware utilizado
- `ESP32 DevKit C V4`: unidade de processamento e conectividade Wi-Fi/MQTT.
- `DHT22`: sensor digital para temperatura (°C) e umidade relativa (%).
- `Potenciômetro`: emula um sensor de batimentos, permitindo gerar diferentes valores de BPM durante a simulação.
- `LED vermelho`: indicador visual de alerta.
- `Buzzer`: indicador sonoro para estados críticos.
Todos os componentes e conexões estão descritos em `diagram.json`, pronto para importação direta na Wokwi.

## 4. Interfaces, protocolos e módulos de comunicação
- **TCP/IP (Wi-Fi 802.11 b/g/n)** — provê a camada de transporte para o ESP32 se comunicar com a Internet.
- **MQTT 3.1.1** — protocolo publish/subscribe usado para telemetria (`vitalcharger/telemetry`) e alertas (`vitalcharger/alert`). A biblioteca `PubSubClient` gerencia QoS 0, reconexão automática e envio assíncrono.
- **Serial (USB)** — saída de debug via `Serial Monitor` do Wokwi ou IDE Arduino para acompanhar leituras e status de conexão.
- **GPIOs** — entradas analógicas/digitais para sensores e atuadores locais (DHT22, potenciômetro, LED e buzzer).

## 5. Comunicação/controle via internet (TCP/IP + MQTT)
O firmware conecta-se automaticamente ao SSID `Wokwi-GUEST`, obtém endereço IP via DHCP e estabelece uma sessão MQTT com `test.mosquitto.org:1883`. As mensagens JSON publicadas podem ser monitoradas por qualquer cliente MQTT externo (por exemplo, MQTT Explorer) assinando os tópicos `vitalcharger/telemetry` e `vitalcharger/alert`. Alterações no potenciômetro ou aquecimento do DHT22 durante a simulação refletem imediatamente nas mensagens recebidas, permitindo controle/monitoramento remoto em tempo real.

### Como executar no Wokwi
1. Acesse https://wokwi.com/ e faça login.
2. Crie um novo projeto ESP32 e substitua o conteúdo dos arquivos padrão pelos arquivos deste diretório (`diagram.json`, `sketch.io`, `libraries.txt`).
3. Clique em **Start Simulation**.
4. Ajuste o potenciômetro ou aqueça o DHT virtual para gerar alertas.
5. Opcional: abra um cliente MQTT externo e assine os tópicos para visualizar os JSONs enviados.

