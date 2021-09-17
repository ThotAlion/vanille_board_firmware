// Mag Alpha library 1.0.1
#include <MagAlpha.h>
// Adafruit BNO055 library
#include <Wire.h>
#include <I2C_LCD.h>
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
         | [ ]V.ref     ___    SS/D10[ ]~| ROT BP
  ROT DT | [ ]A0/D14   / N \       D9[ ]~|
  ROT CK | [ ]A1/D15  /  A  \      D8[ ] |
  AN VOLT | [ ]A2/D16  \  N  /      D7[ ] |
  AN AMP | [ ]A3/D17   \_0_/       D6[ ]~|
  LCD/BNO | [ ]A4/D18/SDA           D5[ ]~|
  LCD/BNO | [ ]A5/D19/SCL           D4[ ] |
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
#define ROT_CK 14
#define ROT_DT 15
#define ROT_BP 10
#define VOLTAGE A2
#define CURRENT A3
#define SPI_SCLK_FREQUENCY  100000      //SPI SCLK Clock frequency in Hz

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// LCD
I2C_LCD LCD;
extern GUI_Bitmap_t bmdog;       //Declare bitmap data package.
uint8_t I2C_LCD_ADDRESS = 0x51;


//Temps d'echantillonnage (ms)
const int dt = 10;

//Temps de rafraichissement écran LCD
const int dt_LCD = 1000;

int BP, BP_old, CK, CK_old, DT,aVal,aLast;
int state = -1;
int icur = 0;

unsigned long t0;


float voltage, current;

char c0, c1;

MagAlpha magAlphaFRX;
MagAlpha magAlphaFLX;

void setup() {
  // Rotary encoder
  pinMode (ROT_CK, INPUT);
  pinMode (ROT_DT, INPUT);
  pinMode (ROT_BP, INPUT_PULLUP);
  // start serial port
  Serial.begin(UART_BAUDRATE);
  Serial1.begin(UART_BAUDRATE);
  // start magnetic encoders
  magAlphaFRX.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, CSFRONTRIGHTHIPX);
  magAlphaFLX.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, CSFRONTLEFTHIPX);
  // start BNO IMU
  bno.begin();
  bno.setExtCrystalUse(true);
  LCD.CleanAll(WHITE);
  LCD.WorkingModeConf(ON, ON, WM_BitmapMode);
  LCD.DrawScreenAreaAt(&bmdog, 30, 0);
  LCD.BacklightConf(LOAD_TO_RAM, 50);
  delay(1000);            //Delay for 5s.
  LCD.CleanAll(WHITE);
  LCD.WorkingModeConf(ON, ON, WM_CharMode);
  LCD.BacklightConf(LOAD_TO_RAM, 0);
  LCD.FontModeConf(Font_6x8, FM_ANL_AAA, BLACK_BAC);
  aLast = digitalRead(ROT_CK);
}

void loop() {
  // lecture du BNO
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat = bno.getQuat();

  // lecture des codeurs absolus
  uint16_t angleRawFRX;
  bool errorFRX;
  angleRawFRX = magAlphaFRX.readAngleRaw(&errorFRX);
  uint16_t angleRawFLX;
  bool errorFLX;
  angleRawFLX = magAlphaFLX.readAngleRaw(&errorFLX);

  // lecture du codeur rotatif d'interface
  aVal = digitalRead(ROT_CK);
  if (aVal != aLast) { // L'axe est en rotation
    // si l'axe est en rotation, le sens doit être déterminé
    // La broche DT (B) doit être lue.

    if (digitalRead(ROT_DT) != aVal) {  // Signie que A change avant B - la rotation est donc horaire

      icur ++;

    } else {// Sinon, c'est que B change avant A, la rotation est anti-horaire
      icur--;
    }
  }
  
  
  BP_old = BP;
  BP = digitalRead(ROT_BP);
  
    //lecture de la tension
    voltage=analogRead(VOLTAGE)*0.052;

    //lecture du courant
    current=((analogRead(CURRENT)*5.0/1024.0)-2.5)/0.028;

    // Serial port management
    if(Serial.available()>0){
      c0 = (char)Serial.read();
      if(c0=='A'){
        Serial.print("{\"ANG\":{\"X\":");
        Serial.print(-(float)orientationData.orientation.y);
        Serial.print(",\"Y\":");
        Serial.print((float)orientationData.orientation.z);
        Serial.print(",\"Z\":");
        Serial.print((float)orientationData.orientation.x);
        Serial.print("},\"QUAT\":{\"W\":");
        Serial.print((float)quat.w());
        Serial.print(",\"X\":");
        Serial.print((float)quat.x());
        Serial.print(",\"Y\":");
        Serial.print((float)quat.y());
        Serial.print(",\"Z\":");
        Serial.print((float)quat.z());
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
        Serial1.print("},\"QUAT\":{\"W\":");
        Serial1.print((float)quat.w());
        Serial1.print(",\"X\":");
        Serial1.print((float)quat.x());
        Serial1.print(",\"Y\":");
        Serial1.print((float)quat.y());
        Serial1.print(",\"Z\":");
        Serial1.print((float)quat.z());
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
  //if((millis()-t0)>dt_LCD){
  switch (state) {
    case -1:
      LCD.CleanAll(WHITE);
      LCD.CharGotoXY(0, 0);
      LCD.println("Etat 0");
      state = 0;
      break;
    case 0:
      if (BP == 0 && BP_old == 1) {
        LCD.CleanAll(WHITE);
        LCD.CharGotoXY(0, 0);
        LCD.println(">\tEtat 1");
        icur = 0;
        state = 1;
      }
      break;
    case 1:
      if (aVal != aLast) {
        Serial.println(icur);
        /*LCD.println(" ");
        LCD.println(" ");
        LCD.println(" ");
        LCD.println(" ");
        LCD.println(" ");
        LCD.println(" ");
        LCD.println(" ");
        LCD.CharGotoXY(0, 8 * icur);
        LCD.println(">");*/
      }
      break;
  }
  aLast = aVal;
  //  delay(dt);
}
