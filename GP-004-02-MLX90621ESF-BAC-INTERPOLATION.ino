/*
* Adapted by Josh Long (https://github.com/longjos) Oct 2015
* Based on a https://github.com/robinvanemden/MLX90621_Arduino_Processing
* Modified by Robert Chapman and Mike Blankenship to work with the GP-001-01 (MLX906XX EVAL Arduino shield) and GP-004-02 (Thermal Imager)
* TFT graphics and Interpolation sketch by Sandbox Electronics http://sandboxelectronics.com/
* Verified to work on the Arduino UNO R3 and Leonardo
* Original work by:
* 2013 by Felix Bonowski
* Based on a forum post by maxbot: http://forum.arduino.cc/index.php/topic,126244.msg949212.html#msg949212
* This code is in the public domain.
*/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <Wire.h>
#include <OneWire.h>
#include "MLX90621.h"

#define TEMP_MIN      (0) // 0 original
#define TEMP_MAX     (60) // 35 original
#define TEMP_STEPS (2040) // 1020 original
#define BLOCK_SIZE   (10)

//TWBR = ((CPU_FREQ / TWI_FREQ_NUNCHUCK) - 16) / 2;

#define TFT_CS     9
#define TFT_RST    12 // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     6

#define TFT_CS_PORT PORTB
#define TFT_CS_PIN  5
#define TFT_DC_PORT PORTD
#define TFT_DC_PIN  7

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Instanciate One Wire Library
OneWire ds(4);  // mikey was on pin 2

// Instance of the Mlexis sensor
MLX90621 sensor; // create an instance of the Sensor class

float    AmbientTemp;
float    DS18B20_temp;
uint32_t LoopTimer;
byte     LoopCounter = 0;

void setup(void) {
    tft.initR(INITR_BLACKTAB);
    tft.fillScreen(ST7735_BLACK);
    tft.setRotation(3);
    tft.setTextColor(ST7735_RED);
    tft.setTextSize(1);
  
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    Serial.begin(115200);
    sensor.initialise(16); // Mikey was 2
    sensor.measure(true);
}

void loop() {
    byte     x, y;
    uint16_t grids[4][16];
    float    t;

    if (LoopCounter == 0) {
        showVBATvolt();
        showMLXvolt();
        showAmbientTemp();
        showExternalTemp();
        LoopTimer = millis();
    }

    sensor.measure(true); //get new readings from the sensor
    
    Serial.println("IRSTART");
    Serial.println("MLX90621"); // modified

    for(y=0;y<4;y++){ //go through all the rows
        for(x=0;x<16;x++){ //go through all the columns
            t = sensor.getTemperature(y+x*4);
            grids[y][15-x] = mapT(t);
            Serial.print(t); Serial.print(",");
        }
        Serial.println();
    }

    for (y=0; y<3; y++) {
        for (x=0; x<15; x++) {
            drawBlock(y, x, grids[y][x], grids[y][x+1], grids[y+1][x], grids[y+1][x+1]);
        }
    }    

    Serial.print("TA=");
    Serial.print(AmbientTemp);
    Serial.print(",");

    Serial.print("CPIX=");
    Serial.print(sensor.get_CPIX()); // GET VALUE MIKEY red shOW
    Serial.print(",");

    Serial.print("PTAT=");
//    Serial.print(sensor.get_PTAT()); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("EMISSIVITY=");
    Serial.print("1"); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("V_TH=");
    //Serial.print(sensor.get_KT1());
    Serial.print("6760"); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("K_T1=");
    Serial.print("23.03"); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("K_T2=");
    Serial.print("0.02"); // GET VALUE MIKEY
    Serial.print(",");

    Serial.print("MY_TEMP="); //DS18B20 TEMP
    Serial.print(DS18B20_temp);
    Serial.print(",");

    Serial.println("IREND");

    if (++LoopCounter == 50) {
        tft.setTextColor(ST7735_BLUE);
        tft.setTextSize(1);
        tft.setCursor(115,120);
        tft.print("FPS: ");
        tft.setTextColor(ST7735_RED);
        tft.setTextSize(1);
        tft.fillRect(140,120,25,7,0x000000);
        tft.setCursor(140,120);
        Serial.print("LoopCounter: ");
        Serial.println((float)LoopCounter * 1000);
        Serial.print("LoopTimer: ");
        Serial.println(LoopTimer);
        Serial.print("millis: ");
        Serial.println(millis());
        tft.print((50 * 1000.0) / (millis() - LoopTimer));
        LoopCounter = 0;
    }
}


void showExternalTemp(){
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  //byte data[9];
  byte addr[8];
  float celsius, fahrenheit;

  if ( !ds.search(addr)) {
    ///Serial1111.println("No more addresses.");
    ///Serial1111.println();
    ds.reset_search();
    delay(1);
    return;
  }

  for( i = 0; i < 8; i++) {
      }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      ///Serial1111.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();

      }

  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }

  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  DS18B20_temp = celsius;

  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(1);
  tft.setCursor(0,110);
  tft.print("EXTTEMP: ");
  tft.fillRect(50,110,30,7,0x000000);
  tft.setTextColor(ST7735_RED);
  tft.setCursor(50, 110);
  tft.print(DS18B20_temp);
   tft.setTextColor(ST7735_YELLOW);
  tft.print((char)248);
  tft.print("C");
}

void showMLXvolt() {
    int sensorValue2 = analogRead(A10);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltage2 = sensorValue2 * (5.0 / 1023.0);
    
    tft.setCursor(0,100);
    tft.setTextColor(ST7735_BLUE);
    tft.setTextSize(1);
    tft.print("VMLX: ");
    tft.fillRect(30,100,25,7,0x000000);
    tft.setTextColor(ST7735_RED);
    tft.setCursor(30,100);
    tft.print(voltage2);
    tft.setTextColor(ST7735_YELLOW);
    tft.print(" Volts ");
}


void showVBATvolt() {
    int sensorValue = analogRead(A5);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltage = sensorValue * (5.0 / 1023.0);
  
    tft.setTextColor(ST7735_BLUE);
    tft.setTextSize(1);
    tft.setCursor(0,90);
    tft.print("VBAT: ");
    tft.fillRect(30,90,25,7,0x000000);
    tft.setTextColor(ST7735_RED);
    tft.setCursor(30,90);
    tft.print(voltage);
    tft.setTextColor(ST7735_YELLOW);
    tft.print( " Volts");
}


void showAmbientTemp() {
    AmbientTemp = sensor.getAmbient();

    tft.setTextColor(ST7735_BLUE);
    tft.setTextSize(1);
    tft.setCursor(0,120);
    tft.print("INTTEMP: ");
    tft.fillRect(50,120,35,7,0x000000);
    tft.setTextColor(ST7735_RED);
    tft.setCursor(50, 120);
    tft.print(AmbientTemp);
    tft.setTextColor(ST7735_YELLOW);
    tft.print((char)248);
    tft.print("C");
}


uint16_t mapT(float temp) {
    int16_t t;

    t = (int16_t)((temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * TEMP_STEPS);

    if (t > TEMP_STEPS) {
        t = TEMP_STEPS;
    } else if (t < 0) {
        t = 0;
    }

    return t;
}


void drawBlock(byte row, byte column, uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    byte i, j, x, y;
    byte red, green, blue;
    uint16_t color;

    uint16_t ac[BLOCK_SIZE];
    uint16_t bd[BLOCK_SIZE];
    uint16_t line[BLOCK_SIZE];
    
    interpolateT(ac, a, c);
    interpolateT(bd, b, d);

    x = column * BLOCK_SIZE + 5;
    y = row * BLOCK_SIZE * 2;

    tft.setAddrWindow(x, y, x + BLOCK_SIZE - 1, y + BLOCK_SIZE * 2 - 1);

    for (i=0; i<BLOCK_SIZE * 2; i++) {
        
        if (i % 2 == 0) {//uint32_t start = micros();
            interpolateC(line, ac[i/2], bd[i/2]);//Serial.println(micros() - start);
        }

        for (j=0; j<BLOCK_SIZE; j++) {
            TFT_DC_PORT |= 1 << TFT_DC_PIN;
            TFT_CS_PORT &= ~(1 << TFT_CS_PIN);

            SPDR = (line[j] >> 8);
            while(!(SPSR & _BV(SPIF)));
            SPDR = (line[j]);
            while(!(SPSR & _BV(SPIF)));

            TFT_CS_PORT |= 1 << TFT_CS_PIN;
        }
    }
}


void interpolateT(uint16_t *line, uint16_t p, uint16_t q) {
    byte i;

    for (i=0; i<BLOCK_SIZE; i++) {
        line[i] = (p * (BLOCK_SIZE - i) + q * i) / BLOCK_SIZE;
    }
}


void interpolateC(uint16_t *line, uint16_t p, uint16_t q) {
    byte     i, r, g, b;
    uint16_t t;

    for (i=0; i<BLOCK_SIZE; i++) {
        t = (p * (BLOCK_SIZE - i) + q * i) / BLOCK_SIZE;

        //RED
        if (t >= 765) {
            r = 255;
        } else if (t > 510) {
            r = t - 510;
        } else {
            r = 0;
        }
    
        //GREEN
        if (t > 765) {
            g = 1020 - t;
        } else if (t >= 255) {
            g = 255;
        } else {
            g = t;
        }
    
        //BLUE
        if (t >= 510) {
            b = 0;
        } else if (t > 255) {
            b = 510 - t;
        } else {
            b = 255;
        }

        line[i] = r >> 3 << 11 | g >> 2 << 5 | b >> 3;
    }
}

