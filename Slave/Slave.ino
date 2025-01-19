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
const uint8_t localAddress = 0xB2;     // Dirección de este dispositivo
uint8_t destination = 0xFF;            // Dirección de destino, 0xFF es la dirección de broadcast

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;

// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf   = { 6, 10, 5, 2};
LoRaConfig_t LastConf = thisNodeConf;
int remoteRSSI = 0;
float remoteSNR = 0;

// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);  
  while (!Serial); 

  Serial.print("\nNodo SLAVE\n_______________________\n");

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
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index])); 
                                  // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3
                                  // 41.7E3, 62.5E3, 125E3, 250E3, 500E3 
                                  // Multiplicar por dos el ancho de banda
                                  // supone dividir a la mitad el tiempo de Tx
                                  
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);     
                                  // [6, 12] Aumentar el spreading factor incrementa 
                                  // de forma significativa el tiempo de Tx
                                  // SPF = 6 es un valor especial
                                  // Ver tabla 12 del manual del SEMTECH SX1276
  
  LoRa.setCodingRate4(thisNodeConf.codingRate);         
                                  // [5, 8] 5 da un tiempo de Tx menor
                                  
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN); 
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

}

void sendMessageLogic(){
  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;

  transmitting = true;
  txDoneFlag = false;
  tx_begin_ms = millis();

  sendMessage();
  Serial.print("Sending packet ");
  //Serial.print(msgCount++);
  //Serial.print(": ");

  //printBinaryPayload(payload, payloadLength);
                  
  uint32_t TxTime_ms = millis() - tx_begin_ms;
  Serial.print("----> TX completed in ");
  Serial.print(TxTime_ms);
  Serial.println(" msecs");
  
  // Ajustamos txInterval_ms para respetar un duty cycle del 1% 
  uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
  lastSendTime_ms = tx_begin_ms; 
  float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;
  
  Serial.print("Duty cycle: ");
  Serial.print(duty_cycle, 1);
  Serial.println(" %\n");

  // Solo si el ciclo de trabajo es superior al 1% lo ajustamos
  if (duty_cycle > 1.0f) {
    txInterval_ms = TxTime_ms * 100;
  }
  
  transmitting = false;
  
  // Reactivamos la recepción de mensajes, que se desactiva
  // en segundo plano mientras se transmite
  LoRa.receive();   
}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendMessage() 
{
  while(!LoRa.beginPacket()) {            // Comenzamos el empaquetado del mensaje
    delay(10);                            // 
  }
  LoRa.write(destination);                // Añadimos el ID del destinatario
  LoRa.write(localAddress);               // Añadimos el ID del remitente
  // Incluimos el RSSI y el SNR del último paquete recibido
  // RSSI puede estar en un rango de [0, -127] dBm
  LoRa.write(uint8_t(-remoteRSSI * 2));
  // SNR puede estar en un rango de [20, -148] dBm
  LoRa.write(uint8_t(148 + remoteSNR));
  LoRa.endPacket(true);                   // Finalizamos el paquete, pero no esperamos a
                                          // finalice su transmisión
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
  uint16_t cofiguration = ((uint16_t)LoRa.read() << 8) | 
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

  bool goodParams = checkGoodParams();

  sendMessageLogic();

  if (goodParams) {
    changeLoRaParameters(cofiguration);
  } else {
    configureLoRa(LastConf);
    Serial.println("Bad parameters received. Returning to last good configuration.");
  }
  
}

void TxFinished()
{
  txDoneFlag = true;
}

void configureLoRa(LoRaConfig_t config) {
  LoRa.setSignalBandwidth(long(bandwidth_kHz[config.bandwidth_index]));
  LoRa.setSpreadingFactor(config.spreadingFactor);
  LoRa.setCodingRate4(config.codingRate);
  LoRa.setTxPower(config.txPower, PA_OUTPUT_PA_BOOST_PIN);
}

void changeLoRaParameters(uint16_t newConfig) {
  LastConf = thisNodeConf;

  // Cambiar los parámetros de LoRa en tiempo de ejecución
  thisNodeConf.bandwidth_index = (newConfig >> 12) & 0b00001111;
  thisNodeConf.spreadingFactor = ((newConfig >> 9) & 0b00000111)+6;
  thisNodeConf.codingRate = ((newConfig >> 6) & 0b00000011)+5;
  thisNodeConf.txPower = ((newConfig >> 1) & 0b00011111)+2;

  // Aplicar los nuevos parámetros
  configureLoRa(thisNodeConf);

  Serial.println("LoRa parameters changed:");
  Serial.print("Bandwidth: ");
  Serial.println(bandwidth_kHz[thisNodeConf.bandwidth_index]);
  Serial.print("Spreading Factor: ");
  Serial.println(thisNodeConf.spreadingFactor);
  Serial.print("Coding Rate: ");
  Serial.println(thisNodeConf.codingRate);
  Serial.print("Tx Power: ");
  Serial.println(thisNodeConf.txPower);
  Serial.println();
}

bool checkGoodParams() {
  // Comprobar que los parámetros recibidos están en un rango aceptable
  remoteRSSI = LoRa.packetRssi();
  remoteSNR = LoRa.packetSnr();

  if (remoteRSSI < -100) return false;
  if (remoteSNR < 0) return false;
  return true;
}