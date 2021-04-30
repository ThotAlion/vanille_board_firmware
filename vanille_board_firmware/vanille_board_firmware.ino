// Mag Alpha library 1.0.1
#include <MagAlpha.h>
// Adafruit BNO055 library
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*
Description :
TBC.
Here is the schematic :
                      +-----+
         +------------| USB |------------+
         |            +-----+            |
  MAG702 | [ ]D13/CLK        MISO/D12[ ] | MAG702   
         | [ ]3.3V           MOSI/D11[ ]~| MAG702  
         | [ ]V.ref     ___    SS/D10[ ]~|   
         | [ ]A0/D14   / N \       D9[ ]~| 
         | [ ]A1/D15  /  A  \      D8[ ] | 
         | [ ]A2/D16  \  N  /      D7[ ] | 
         | [ ]A3/D17   \_0_/       D6[ ]~| 
     BNO | [ ]A4/D18/SDA           D5[ ]~| 
     BNO | [ ]A5/D19/SCL           D4[ ] |   
         | [ ]A6                   D3[ ]~| CSFRONTRIGHTHIPX  
         | [ ]A7                   D2[ ] | CSFRONTLEFTHIPX 
         | [ ]5V                  GND[ ] |     
         | [ ]RST                 RST[ ] |   
         | [ ]GND                 RX1[ ] | UPBOARD  
         | [ ]Vin                 TX1[ ] | UPBOARD  
         |                               |
         |                               |
         | NANO EVERY                    |
         +-------------------------------+
*/  
#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)       
#define CSFRONTRIGHTHIPX 3
#define CSFRONTLEFTHIPX 2
#define SPI_SCLK_FREQUENCY  100000      //SPI SCLK Clock frequency in Hz

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

//Temps d'echantillonnage (ms)
const int dt = 10;

char c0,c1;

MagAlpha magAlphaFRX;
MagAlpha magAlphaFLX;

void setup() {
  // start serial port
  Serial.begin(UART_BAUDRATE);
  Serial1.begin(UART_BAUDRATE);
  // start magnetic encoders
  magAlphaFRX.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, CSFRONTRIGHTHIPX);
  magAlphaFLX.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, CSFRONTLEFTHIPX);
  // start BNO IMU
  bno.begin();
  bno.setExtCrystalUse(true);
  
}

void loop() {
  // lecture du BNO
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  uint16_t angleRawFRX;
  bool errorFRX;
  angleRawFRX = magAlphaFRX.readAngleRaw(&errorFRX);
  uint16_t angleRawFLX;
  bool errorFLX;
  angleRawFLX = magAlphaFLX.readAngleRaw(&errorFLX);
  if(Serial.available()>0){
    c0 = (char)Serial.read();
    if(c0=='A'){
      Serial.print("{\"ANG\":{\"X\":");
      Serial.print(-(float)orientationData.orientation.y);
      Serial.print(",\"Y\":");
      Serial.print((float)orientationData.orientation.z);
      Serial.print(",\"Z\":");
      Serial.print((float)orientationData.orientation.x);
      Serial.print("},\"FRX\":{\"V\":");
      Serial.print(angleRawFRX);
      Serial.print(",\"B\":");
      Serial.print(!errorFRX);
      Serial.print("},\"FLX\":{\"V\":");
      Serial.print(angleRawFLX);
      Serial.print(",\"B\":");
      Serial.print(!errorFLX);
      Serial.print("}}\n");
    }
  }
  if(Serial1.available()>0){
    c1 = (char)Serial1.read();
    if(c1=='A'){
      Serial1.print("{\"ANG\":{\"X\":");
      Serial1.print(-(float)orientationData.orientation.y);
      Serial1.print(",\"Y\":");
      Serial1.print((float)orientationData.orientation.z);
      Serial1.print(",\"Z\":");
      Serial1.print((float)orientationData.orientation.x);
      Serial1.print("},\"FRX\":{\"V\":");
      Serial1.print(angleRawFRX);
      Serial1.print(",\"B\":");
      Serial1.print(!errorFRX);
      Serial1.print("},\"FLX\":{\"V\":");
      Serial1.print(angleRawFLX);
      Serial1.print(",\"B\":");
      Serial1.print(!errorFLX);
      Serial1.print("}}\n");
    }
  }
  delay(dt);
}
