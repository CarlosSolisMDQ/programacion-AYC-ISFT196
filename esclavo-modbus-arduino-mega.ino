/*
 * ============================================================================
 * ARDUINO MEGA - ESCLAVO MODBUS RTU
 * ============================================================================
 * 
 * DESCRIPCIÓN GENERAL:
 * Este programa convierte un Arduino Mega en un esclavo Modbus RTU para 
 * comunicación industrial a través de puerto serial (RS-485/RS-232).
 * 
 * CARACTERÍSTICAS:
 * - Dirección del esclavo: 17 (configurable)
 * - Velocidad: 9600 baudios
 * - 8 Entradas digitales (Dpin1-Dpin8): Lectura de sensores/señales
 * - 8 Salidas digitales (Qpin1-Qpin8): Control de actuadores/LEDs
 * 
 * FUNCIONES MODBUS IMPLEMENTADAS:
 * - Función 01: Leer Bobinas (Read Coils) - Lee el estado de las entradas
 * - Función 05: Escribir Bobina Individual (Write Single Coil) - Controla 1 salida
 * - Función 15: Escribir Múltiples Bobinas (Write Multiple Coils) - Controla varias salidas
 * 
 * FUNCIONAMIENTO:
 * 1. El Arduino espera mensajes Modbus por serial mediante interrupciones
 * 2. Valida que la dirección del esclavo coincida con la configurada (SC = 17)
 * 3. Verifica la integridad del mensaje usando CRC-16 Modbus
 * 4. Ejecuta la función solicitada (lectura o escritura)
 * 5. Envía respuesta al maestro con el resultado y su CRC
 * 6. Lee continuamente las entradas digitales para mantener estados actualizados
 * 
 * CONEXIONES:
 * - Entradas: Pines digitales 2-9 (D1-D8)
 * - Salidas: Pines A4-A7, 10-13 (Q1-Q8)
 * - Serial: TX0/RX0 para comunicación Modbus
 * 
 * NOTAS:
 * - Usa comunicación UART directa (registros AVR) para mayor control
 * - Procesamiento basado en interrupciones para no perder datos
 * - Compatible con cualquier maestro Modbus RTU estándar (PLC, SCADA, etc.)
 * 
 * WIP:
 * - Hay que implementar la lectura y escritura de pines analogicos
 * ============================================================================
 */

#include <avr/interrupt.h> 
#include <avr/io.h> 

// Cálculo del valor para el registro de baud rate (9600 baudios, 16MHz)
#define myubbr (16000000/16/9600-1)

// Dirección del esclavo Modbus (ID del dispositivo)
unsigned char SC = 17;

// ========== BUFFER DE RECEPCIÓN SERIAL ==========
#define MAX_RX_BUFFER 20
volatile unsigned char SerialData[MAX_RX_BUFFER];  // Buffer para datos recibidos
volatile unsigned char RxIndex = 0;                 // Índice actual en el buffer
volatile boolean MessageComplete = false;           // Bandera: mensaje completo recibido

// ========== ARRAYS DE DATOS ==========
byte digital[8];  // Estado de las 8 bobinas digitales (coils)
unsigned char sendData[20];      // Buffer para datos a enviar
unsigned char sendDataLength = 0; // Longitud de datos a enviar

// ========== CÓDIGOS DE FUNCIÓN MODBUS ==========
unsigned char readCoil = 1;            // Leer bobinas (Read Coils)
unsigned char writeSingleCoil = 5;     // Escribir una bobina (Write Single Coil)
unsigned char writeMultipleCoils = 15; // Escribir múltiples bobinas (Write Multiple Coils)

// ========== DEFINICIÓN DE PINES ==========
// Pines de entrada digital (D1-D8)
const int Dpin1 = 2; const int Dpin2 = 3; const int Dpin3 = 4; const int Dpin4 = 5;
const int Dpin5 = 6; const int Dpin6 = 7; const int Dpin7 = 8; const int Dpin8 = 9;

// Pines de salida digital (Q1-Q8)
const int Qpin1 = A4; // LED indicador
const int Qpin2 = A5; const int Qpin3 = A6; const int Qpin4 = A7; const int Qpin5 = 10;
const int Qpin6 = 11; const int Qpin7 = 12; const int Qpin8 = 13;

void setup() {
  // ========== CONFIGURACIÓN UART ==========
  // Configurar baud rate para comunicación serial
  UBRR0H = (unsigned char)(myubbr>>8);  // Byte alto del baud rate
  UBRR0L = (unsigned char)myubbr;       // Byte bajo del baud rate
  
  // Habilitar recepción (RXEN0), transmisión (TXEN0) e interrupción de recepción (RXCIE0)
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  interrupts();  // Habilitar interrupciones globales
  
  // ========== CONFIGURACIÓN DE PINES ==========
  // Configurar pines de entrada (lectura de sensores/señales)
  pinMode(Dpin1, INPUT); pinMode(Dpin2, INPUT); pinMode(Dpin3, INPUT); pinMode(Dpin4, INPUT);
  pinMode(Dpin5, INPUT); pinMode(Dpin6, INPUT); pinMode(Dpin7, INPUT); pinMode(Dpin8, INPUT);
  
  // Configurar pines de salida (control de actuadores/LEDs)
  pinMode(Qpin1, OUTPUT); pinMode(Qpin2, OUTPUT); pinMode(Qpin3, OUTPUT); pinMode(Qpin4, OUTPUT);
  pinMode(Qpin5, OUTPUT); pinMode(Qpin6, OUTPUT); pinMode(Qpin7, OUTPUT); pinMode(Qpin8, OUTPUT);
  
  // Inicializar todas las bobinas en 0 (apagadas)
  for(byte i = 0; i < 8; i++) digital[i] = 0;
  updateOutputPins();  // Aplicar estados iniciales a los pines físicos
  
  // ========== SECUENCIA DE INICIO (LED parpadeante) ==========
  digitalWrite(Qpin1, HIGH); delay(500);
  digitalWrite(Qpin1, LOW); delay(500);
}

void loop() {
  // ========== LECTURA DE ENTRADAS DIGITALES ==========
  // Leer constantemente el estado de los pines de entrada
  digital[0] = digitalRead(Dpin1);
  digital[1] = digitalRead(Dpin2);
  digital[2] = digitalRead(Dpin3);
  digital[3] = digitalRead(Dpin4);
  digital[4] = digitalRead(Dpin5);
  digital[5] = digitalRead(Dpin6);
  digital[6] = digitalRead(Dpin7);
  digital[7] = digitalRead(Dpin8);

  // ========== PROCESAMIENTO DE MENSAJES MODBUS ==========
  if(MessageComplete) {
    MessageComplete = false;  // Resetear bandera
    
    // Preparar respuesta: copiar dirección de esclavo y código de función
    sendData[0] = SerialData[0];  // Dirección del esclavo
    sendData[1] = SerialData[1];  // Código de función
    sendDataLength = 2;
    
    // ========== EJECUTAR FUNCIÓN SEGÚN CÓDIGO RECIBIDO ==========
    if(SerialData[1] == readCoil) 
      Read_Coil();  // Leer estado de bobinas
    else if(SerialData[1] == writeSingleCoil) 
      Write_Single_Coil();  // Escribir una bobina
    else if(SerialData[1] == writeMultipleCoils) 
      Write_Multiple_Coils();  // Escribir múltiples bobinas
    
    // ========== CALCULAR Y AGREGAR CRC ==========
    unsigned short crc = CRC16(sendData, sendDataLength);
    sendData[sendDataLength++] = crc & 0xFF;  // CRC byte bajo (primero)
    sendData[sendDataLength++] = crc >> 8;    // CRC byte alto
    
    // ========== TRANSMITIR RESPUESTA ==========
    for(byte i = 0; i < sendDataLength; i++) {
      while (!(UCSR0A & (1<<UDRE0)));  // Esperar a que el buffer esté listo
      UDR0 = sendData[i];               // Enviar byte
    }
    
    RxIndex = 0;  // Resetear índice para próxima recepción
  }
}

// ========== INTERRUPCIÓN DE RECEPCIÓN SERIAL ==========
ISR(USART0_RX_vect) {
  SerialData[RxIndex] = UDR0;  // Leer byte recibido
  
  // Verificar si el primer byte coincide con la dirección del esclavo
  if(RxIndex == 0 && SerialData[0] != SC) return;
  
  RxIndex++;  // Incrementar índice del buffer
  
  // ========== DETECTAR FIN DE MENSAJE SEGÚN FUNCIÓN ==========
  // Read Coil y Write Single Coil: 8 bytes totales
  if(SerialData[1] == readCoil || SerialData[1] == writeSingleCoil) {
    if(RxIndex == 8) MessageComplete = true;
  } 
  // Write Multiple Coils: longitud variable (depende del byte count)
  else if(SerialData[1] == writeMultipleCoils) {
    if(RxIndex > 6) {
      byte byteCount = SerialData[6];  // Cantidad de bytes de datos
      if(RxIndex == (9 + byteCount)) MessageComplete = true;
    }
  }
}

// ========== FUNCIÓN CRC16 MODBUS ==========
// Calcula el checksum CRC-16 (Modbus) para validar integridad de datos
unsigned short CRC16(const unsigned char* data, unsigned char length) {
  unsigned short crc = 0xFFFF;  // Valor inicial del CRC
  
  for (unsigned char i = 0; i < length; i++) {
    crc ^= (unsigned short)data[i];  // XOR con el byte actual
    
    // Procesar cada bit del byte
    for (unsigned char j = 0; j < 8; j++) {
      if ((crc & 0x0001) != 0) {     // Si el bit menos significativo es 1
        crc >>= 1;                    // Desplazar a la derecha
        crc ^= 0xA001;                // XOR con polinomio Modbus
      } else {
        crc >>= 1;                    // Solo desplazar
      }
    }
  }
  return crc;
}

// ========== FUNCIÓN MODBUS 01: LEER BOBINAS ==========
void Read_Coil() {
  // Extraer dirección inicial y cantidad de bobinas a leer
  unsigned short start = (SerialData[2] << 8) | SerialData[3];
  unsigned short qty = (SerialData[4] << 8) | SerialData[5];
  
  // Verificar CRC del mensaje recibido
  unsigned short rx_crc = CRC16(SerialData, 6);
  if ((SerialData[6] == (rx_crc & 0xFF)) && (SerialData[7] == (rx_crc >> 8))) {
    // Limitar cantidad de bobinas (máximo 8)
    if (qty > 8) qty = 8;
    if (start + qty > 8) qty = 8 - start;
    
    // Calcular cantidad de bytes necesarios (8 bits por byte)
    unsigned char bytes = (qty + 7) / 8;
    sendData[sendDataLength++] = bytes;  // Agregar byte count a respuesta
    
    // Empaquetar estados de bobinas en bytes (formato Modbus)
    unsigned char val = 0;
    unsigned char bit = 0;
    for (unsigned short i = 0; i < qty; i++) {
      if (digital[start + i]) val |= (1 << bit);  // Setear bit si bobina está ON
      bit++;
      
      // Cuando se completa un byte o es la última bobina
      if (bit == 8 || i == qty - 1) {
        sendData[sendDataLength++] = val;
        val = 0;
        bit = 0;
      }
    }
  }
}

// ========== FUNCIÓN MODBUS 05: ESCRIBIR UNA BOBINA ==========
void Write_Single_Coil() {
  // Extraer dirección y valor de la bobina
  unsigned short addr = (SerialData[2] << 8) | SerialData[3];
  unsigned short value = (SerialData[4] << 8) | SerialData[5];
  
  // Verificar CRC del mensaje recibido
  unsigned short rx_crc = CRC16(SerialData, 6);
  if ((SerialData[6] == (rx_crc & 0xFF)) && (SerialData[7] == (rx_crc >> 8))) {
    if (addr < 8) {  // Verificar dirección válida
      // 0xFF00 = ON, cualquier otro valor = OFF (estándar Modbus)
      digital[addr] = (value == 0xFF00) ? 1 : 0;
      updateOutputPins();  // Actualizar salidas físicas
      
      // Respuesta: eco de los datos recibidos
      sendData[sendDataLength++] = SerialData[2];
      sendData[sendDataLength++] = SerialData[3];
      sendData[sendDataLength++] = SerialData[4];
      sendData[sendDataLength++] = SerialData[5];
    }
  }
}

// ========== FUNCIÓN MODBUS 15: ESCRIBIR MÚLTIPLES BOBINAS ==========
void Write_Multiple_Coils() {
  // Extraer parámetros del mensaje
  unsigned short start = (SerialData[2] << 8) | SerialData[3];  // Dirección inicial
  unsigned short qty = (SerialData[4] << 8) | SerialData[5];     // Cantidad de bobinas
  unsigned char byteCount = SerialData[6];                       // Bytes de datos
  
  // Verificar CRC del mensaje recibido
  unsigned short rx_crc = CRC16(SerialData, 7 + byteCount);
  if ((SerialData[7 + byteCount] == (rx_crc & 0xFF)) && (SerialData[8 + byteCount] == (rx_crc >> 8))) {
    if (start + qty <= 8) {  // Verificar que no exceda el rango
      // Desempaquetar bits de los bytes recibidos
      unsigned char bit = 0;
      unsigned char byteIdx = 7;  // Índice donde comienzan los datos
      
      for (unsigned short i = 0; i < qty; i++) {
        // Extraer bit individual y asignarlo a la bobina
        digital[start + i] = (SerialData[byteIdx] & (1 << bit)) ? 1 : 0;
        bit++;
        
        // Pasar al siguiente byte cada 8 bits
        if (bit == 8) {
          bit = 0;
          byteIdx++;
        }
      }
      
      updateOutputPins();  // Actualizar todas las salidas físicas
      
      // Respuesta: eco de dirección y cantidad
      sendData[sendDataLength++] = SerialData[2];
      sendData[sendDataLength++] = SerialData[3];
      sendData[sendDataLength++] = SerialData[4];
      sendData[sendDataLength++] = SerialData[5];
    }
  }
}

// ========== ACTUALIZAR PINES FÍSICOS ==========
// Escribe los valores del array digital[] a los pines de salida
void updateOutputPins() {
  digitalWrite(Qpin1, digital[0]);
  digitalWrite(Qpin2, digital[1]);
  digitalWrite(Qpin3, digital[2]);
  digitalWrite(Qpin4, digital[3]);
  digitalWrite(Qpin5, digital[4]);
  digitalWrite(Qpin6, digital[5]);
  digitalWrite(Qpin7, digital[6]);
  digitalWrite(Qpin8, digital[7]);
}
