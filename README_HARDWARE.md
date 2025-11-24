## Hardware, Sensores e Plataforma

Este projeto foi pensado para ser reproduzido 100% no simulador Wokwi, então
você não precisa de componentes físicos para testar. Ainda assim, todos os
itens a seguir existem no mundo real e podem ser montados em uma protoboard.

### 1. Placa de desenvolvimento
- **ESP32 DevKit C V4**  
  Atua como “cérebro” do sistema. Possui Wi-Fi integrado, várias entradas
  analógicas e digitais e funciona com 3,3 V. No Wokwi ele já vem com USB e
  conexões seriais prontas.

### 2. Sensores e atuadores
- **DHT22**  
  Sensor digital de temperatura e umidade. O fio de dados está ligado ao pino 21
  do ESP32. Usa alimentação 3,3 V.
- **Potenciômetro**  
  Simula um sensor cardíaco. O pino central (sinal) vai para o pino analógico 34
  do ESP32. Girar o cursor altera o valor lido e, portanto, o BPM calculado.
- **LED vermelho**  
  Indica visualmente que algo está fora do normal. Ligado ao pino 2 com resistor
  interno do Wokwi.
- **Buzzer piezo**  
  Emite um bipe curto quando há alerta. Controlado pelo pino 26.

### 3. Conexões no Wokwi
```
ESP32 3V3  →  VCC do DHT22 e do potenciômetro
ESP32 GND  →  GND de todos os componentes
Pino 21    →  Dados do DHT22
Pino 34    →  Sinal do potenciômetro
Pino 2     →  Anodo do LED
Pino 26    →  Pino positivo do buzzer
```
Todas essas ligações já estão descritas no arquivo `diagram.json`, bastando
importar para a plataforma.

### 4. Plataforma de simulação
- **Wokwi** (https://wokwi.com/) permite simular ESP32, sensores e rede Wi-Fi.
- Basta arrastar os arquivos do projeto para o editor e clicar em **Start
  Simulation**.
- Vantagens: não queima componentes, permite pausar/retomar e registra o
  terminal serial automaticamente.

### 5. Como adaptar para hardware real
1. Monte os componentes em uma protoboard seguindo as ligações acima.
2. Alimente o ESP32 com cabo USB ou fonte 5 V (o regulador interno converte
   para 3,3 V).
3. Faça o upload do mesmo código via Arduino IDE.
4. Use um sensor de batimento real (por exemplo, MAX30102) conectando-o em vez
   do potenciômetro e ajustando o código para ler a biblioteca correspondente.

### 6. Boas práticas para iniciantes
- Use jumpers coloridos para diferenciar alimentação (vermelho) e terra
  (preto) para evitar inversões.
- Fixe o DHT22 longe de fontes de calor para obter leituras reais.
- Se instalar fisicamente, considere uma caixa impressa em 3D para proteger os
  componentes. Um molde simples em PLA com aberturas para ventilação já é
  suficiente.


