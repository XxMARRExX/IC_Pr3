/* ---------------------------------------------------------------------
 *  Ejemplo MKR1310_LoRa_SendReceive_Binary
 *  Práctica 3
 *  Asignatura (GII-IoT)
 *  
 *  Basado en el ejemplo MKR1310_LoRa_SendReceive_WithCallbacks,
 *  muestra cómo es posible comunicar los parámetros de 
 *  configuración del transceiver entre nodos LoRa en
 *  formato binario *  
 *  
 *  Este ejemplo requiere de una versión modificada
 *  de la librería Arduino LoRa (descargable desde 
 *  CV de la asignatura.
 *  
 *  También usa la librería Arduino_BQ24195 
 *  https://github.com/arduino-libraries/Arduino_BQ24195
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS          10000

// NOTA: Ajustar estas variables 
const uint8_t localAddress = 0xB0;     // Dirección de este dispositivo
uint8_t destination = 0xFF;            // Dirección de destino, 0xFF es la dirección de broadcast

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;

volatile int flagSlaveMessage = 0; // 0 no ha recibido mensaje, 1 no ha recibido mensaje 2 vez, 2 ha recibido mensaje

volatile int param = 0;

// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t nextConf   = { 5, 10, 5, 2};
LoRaConfig_t currentConf = nextConf;
LoRaConfig_t LastConf = currentConf;
volatile int remoteRSSI = 0;
volatile float remoteSNR = 0;

volatile int lastTime = 0;
volatile int lastCorrectTime = 0;

volatile bool finishConfig = false;

// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);  
  while (!Serial); 

  Serial.print("\nNodo MASTER\n_______________________\n");

  Serial.println("LoRa Duplex with TxDone and Receive callbacks");
  Serial.println("Using binary packets");
  
  // Es posible indicar los pines para CS, reset e IRQ pins (opcional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  
  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }
  else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                
  }

  // Configuramos algunos parámetros de la radio
  LoRa.setSignalBandwidth(long(bandwidth_kHz[nextConf.bandwidth_index])); 
                                  // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3
                                  // 41.7E3, 62.5E3, 125E3, 250E3, 500E3 
                                  // Multiplicar por dos el ancho de banda
                                  // supone dividir a la mitad el tiempo de Tx
                                  
  LoRa.setSpreadingFactor(nextConf.spreadingFactor);     
                                  // [6, 12] Aumentar el spreading factor incrementa 
                                  // de forma significativa el tiempo de Tx
                                  // SPF = 6 es un valor especial
                                  // Ver tabla 12 del manual del SEMTECH SX1276
  
  LoRa.setCodingRate4(nextConf.codingRate);         
                                  // [5, 8] 5 da un tiempo de Tx menor
                                  
  LoRa.setTxPower(nextConf.txPower, PA_OUTPUT_PA_BOOST_PIN); 
                                  // Rango [2, 20] en dBm
                                  // Importante seleccionar un valor bajo para pruebas
                                  // a corta distancia y evitar saturar al receptor
  LoRa.setSyncWord(0x12);         // Palabra de sincronización privada por defecto para SX127X 
                                  // Usaremos la palabra de sincronización para crear diferentes
                                  // redes privadas por equipos
  LoRa.setPreambleLength(8);      // Número de símbolos a usar como preámbulo

  
  // Indicamos el callback para cuando se reciba un paquete
  LoRa.onReceive(onReceive);
  
  // Activamos el callback que nos indicará cuando ha finalizado la 
  // transmisión de un mensaje
  LoRa.onTxDone(TxFinished);

  // Nótese que la recepción está activada a partir de este punto
  LoRa.receive();

  Serial.println("LoRa init succeeded.\n");
}

// --------------------------------------------------------------------
// Loop function
// --------------------------------------------------------------------
void loop() 
{
  if (!finishConfig) {
    sendMessageLogic();
  }
}

void sendMessageLogic(){
  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;
      
  if ((!transmitting && ((millis() - lastSendTime_ms) > txInterval_ms))) {
    
    /*
    if (flagSlaveMessage != 2) {
      if (flagSlaveMessage == 1) {
        finishParamSet();
        finishConfig = true;
        return;
      }
      flagSlaveMessage = 1;
    }*/
    
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();
  
    sendConfigurationMessage(nextConf);
    Serial.print("Sending packet ");
    //Serial.print(msgCount++);
    //Serial.print(": ");

    //printBinaryPayload(payload, payloadLength);
  }                  
  
  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs");
    lastTime = TxTime_ms;
    
    // Ajustamos txInterval_ms para respetar un duty cycle del 1% 
    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms; 
    float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;

    Serial.print("Used parameters:");
    printLoRaConfig(currentConf);

    Serial.print("Parameters send:");
    printLoRaConfig(nextConf);
    
    Serial.print("Duty cycle: ");
    Serial.print(duty_cycle, 1);
    Serial.println(" %\n");

    // Solo si el ciclo de trabajo es superior al 1% lo ajustamos
    if (duty_cycle > 1.0f) {
      txInterval_ms = TxTime_ms * 100;
    }

    /*
    if (duty_cycle <= 0.5f) {
      txInterval_ms = TxTime_ms / 100;
    }
    */
    
    transmitting = false;
    if (flagSlaveMessage == 2) flagSlaveMessage = 0;
    
    // Reactivamos la recepción de mensajes, que se desactiva
    // en segundo plano mientras se transmite
    LoRa.receive();   
  }
}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendConfigurationMessage(LoRaConfig_t config) 
{
  uint16_t datos = encodeLoRaConfig(config);

  while(!LoRa.beginPacket()) {            // Comenzamos el empaquetado del mensaje
    delay(10);                            // 
  }

  LoRa.write(destination);                // Añadimos el ID del destinatario
  LoRa.write(localAddress);               // Añadimos el ID del remitente
  LoRa.write(highByte(datos));   // Enviar byte alto del mensaje
  LoRa.write(lowByte(datos));    // Enviar byte bajo del mensaje
  LoRa.endPacket(true);
}

// --------------------------------------------------------------------
// Receiving message function
// --------------------------------------------------------------------
void onReceive(int packetSize) 
{
  if (transmitting && !txDoneFlag) txDoneFlag = true;
  
  if (packetSize == 0) return;          // Si no hay mensajes, retornamos

  // Leemos los primeros bytes del mensaje
  uint8_t buffer[10];                   // Buffer para almacenar el mensaje
  int recipient = LoRa.read();          // Dirección del destinatario
  uint8_t sender = LoRa.read();         // Dirección del remitente
                                        // msg ID (High Byte first)
  uint16_t sendResult = ((uint16_t)LoRa.read() << 8) | 
                            (uint16_t)LoRa.read();

  Serial.print("Received from: 0x");
  Serial.println(sender, HEX);
  // Verificamos si se trata de un mensaje en broadcast o es un mensaje
  // dirigido específicamente a este dispositivo.
  // Nótese que este mecanismo es complementario al uso de la misma
  // SyncWord y solo tiene sentido si hay más de dos receptores activos
  // compartiendo la misma palabra de sincronización
  if ((recipient & localAddress) != localAddress ) {
    Serial.println("Receiving error: This message is not for me.");
    return;
  }

  flagSlaveMessage = 2;

  calculateParamChanges(sendResult);
}

void TxFinished()
{
  txDoneFlag = true;
}

uint16_t encodeLoRaConfig(LoRaConfig_t config) {
    uint16_t result = 0;

    // Primer byte: 4 bits de bandwidth_index, 3 bits de spreadingFactor, 1 bit cualquiera (0)
    result |= (config.bandwidth_index & 0x0F) << 12; // 4 bits de bandwidth_index
    result |= ((config.spreadingFactor-6) & 0x07) << 9;  // 3 bits de spreadingFactor
    result |= 0x01 << 8;                             // 1 bit cualquiera (0)

    // Segundo byte: 2 bits de codingRate, 5 bits de txPower, 1 bit cualquiera (0)
    result |= ((config.codingRate-5) & 0x03) << 6;       // 2 bits de codingRate
    result |= ((config.txPower-2) & 0x1F) << 1;          // 5 bits de txPower
    result |= 0x01;                                  // 1 bit cualquiera (0)

    return result;
}

void printLoRaConfig(LoRaConfig_t config) {
    Serial.print("BW: ");
    Serial.print(bandwidth_kHz[config.bandwidth_index]);
    Serial.print(" kHz, SPF: ");
    Serial.print(config.spreadingFactor);
    Serial.print(", CR: ");
    Serial.print(config.codingRate);
    Serial.print(", TxPwr: ");
    Serial.print(config.txPower);
    Serial.println(" dBm");
}

void calculateParamChanges(uint16_t results) {
    remoteRSSI = results >> 8;
    remoteRSSI = - remoteRSSI / 2;

    remoteSNR = results & 0xFF;
    remoteSNR = remoteSNR - 148;


    Serial.println("Parameters received:");
    Serial.print("RSSI: ");
    Serial.print(remoteRSSI);
    Serial.print(" dBm, SNR: ");
    Serial.println(remoteSNR);

    //los parametros estan en sus rangos normales
    if (checkGoodParams(remoteRSSI, remoteSNR)) {
        lastCorrectTime = lastTime;
        LastConf = currentConf;
        currentConf = nextConf;
        nextParamSet(false);
        configureLoRa(currentConf);
        
    }
    else {
        Serial.println("Bad parameters received. Keeping last configuration.");
        currentConf = LastConf;
        nextConf = currentConf;
        nextParamSet(true);
        printLoRaConfig(currentConf);
        configureLoRa(currentConf);
        
    }
    Serial.println();
}

bool checkGoodParams(int RSSI, float SNR) {
    if (RSSI < -100) return false;
    if (SNR < 0) return false;
    return true;
}

void configureLoRa(LoRaConfig_t config) {
  LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidth_index]));
  LoRa.setSpreadingFactor(config.spreadingFactor);
  LoRa.setCodingRate4(config.codingRate);
  LoRa.setTxPower(config.txPower, PA_OUTPUT_PA_BOOST_PIN);
}

void nextParamSet(bool nextParam) {
  if (nextParam) param++;

  switch (param)
  {
  case 0:
    param += nextBandwidth();
    break;

  case 1:
    param += nextSpreadingFactor();
    break;
  
  default:
    finishParamSet();
    break;
  }    
}

int nextBandwidth() {
  int next = 0;
  if (nextConf.bandwidth_index < 8) {
    nextConf.bandwidth_index++;
  } else {
    next = 1;
  }
  return next;
}

int nextSpreadingFactor() {
  int next = 0;
  if (nextConf.spreadingFactor > 7) {
    nextConf.spreadingFactor--;
  } else {
    next = 1;
  }
  return next;
}

void finishParamSet() {
  finishConfig = true;
  Serial.println();
  Serial.println("---------------- Configuration finished ----------------");
  
  Serial.println("Parameters set:\n");
  printLoRaConfig(currentConf);

  Serial.println("_______________________\n");
  Serial.println("Final stability test:");
  Serial.print("RSSI: "); Serial.print(remoteRSSI); Serial.print(" dBm, SNR: "); Serial.println(remoteSNR);
  Serial.println("_______________________\n");

  Serial.print("Final transmission time: "); Serial.print(lastCorrectTime); Serial.println(" msecs");
}