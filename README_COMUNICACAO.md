## Interfaces, Protocolos e Comunicação

Este documento explica, em linguagem simples, como o Vital Charger IoT conversa
com o mundo externo e como você pode acompanhar os dados.

### 1. Camada física: Wi-Fi
- O ESP32 usa a rede sem fio **Wokwi-GUEST** (já pronta no simulador).
- A conexão é do tipo **TCP/IP**, a mesma utilizada na maioria dos roteadores
  domésticos.
- Após conectar, o ESP32 recebe um endereço IP. Ele aparece no `Serial Monitor`
  logo no início da simulação.

### 2. Protocolo de aplicação: MQTT
- MQTT é um protocolo leve de publicação/assinatura, muito usado em IoT.
- O projeto utiliza o broker público `test.mosquitto.org` na porta 1883.
- Não há senha, o que facilita testes. Para produção, recomenda-se um broker
  privado e conexões seguras (TLS).

#### Tópicos utilizados
| Tópico | Conteúdo | Quando é enviado |
| ------ | -------- | ---------------- |
| `vitalcharger/telemetry` | JSON com BPM, temperatura, umidade e status de alerta | A cada 3 s |
| `vitalcharger/alert` | Mesmo JSON da telemetria | Somente quando há alerta |

#### Estrutura da mensagem JSON
```json
{
  "bpm": 78,
  "temperature": 36.8,
  "humidity": 55.2,
  "alert": false,
  "alertReason": ""
}
```
- `alert` é `true` apenas se algo estiver fora da faixa segura.
- `alertReason` informa se o motivo foi BPM ou temperatura.

### 3. Interface Serial
- O `Serial Monitor` do Wokwi/Arduino IDE mostra:
  - Status da conexão Wi-Fi.
  - Tentativas de conexão ao MQTT.
  - Leituras atuais de BPM, temperatura e umidade.
  - Mensagens de alerta quando existem.
- Útil para depuração em tempo real sem depender do MQTT.

### 4. Como visualizar as mensagens MQTT
1. Baixe um cliente simples, como **MQTT Explorer** ou **MQTTX**.
2. Configure o host como `test.mosquitto.org` e a porta 1883.
3. Clique em conectar e assine os tópicos:
   - `vitalcharger/telemetry`
   - `vitalcharger/alert`
4. Execute o projeto no Wokwi. As mensagens aparecerão em tempo real.

### 5. Como reutilizar a comunicação em outro projeto
- Altere as constantes `MQTT_HOST`, `MQTT_PORT` e os nomes dos tópicos no
  `sketch.io`.
- Atualize também `WIFI_SSID` e `WIFI_PASS` para a sua rede local.
- Se usar TLS (porta 8883), troque `WiFiClient` por `WiFiClientSecure` e
  configure certificados.

### 6. Segurança básica
- Evite expor dados pessoais em brokers públicos. Use apelidos ou IDs sem
  referência direta a pacientes.
- Em produção, habilite autenticação no MQTT e criptografia (MQTTS).
- Monitore o consumo de mensagens para evitar limite em brokers gratuitos.


