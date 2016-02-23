/*
* This project creates a small thermal camera using the MELEXIS MLX90621
*
* Adapted by Josh Long (https://github.com/longjos) Oct 2015
* Based on a https://github.com/robinvanemden/MLX90621_Arduino_Processing
* Modified string output by Mike Blankenship (Mike.Blankenship@garageproto.com)
* for use with the GP-001-XX and GP-004-XX eval baords)
* Original work by:
* 2013 by Felix Bonowski
* Based on a forum post by maxbot: http://forum.arduino.cc/index.php/topic,126244.msg949212.html#msg949212
* This code is in the public domain.
*/
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include "MLX90621.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library

int myLoop=0;
int MyX=0;

//float senArray[64];
float f_val=00.00;
char outstr[15];
boolean running = FALSE;



// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     9 // Mikey
#define TFT_RST    12  // mikey you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     6 //mikey

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
// Option 2: use any pins but a little slower!
#define TFT_SCLK 13   // set these to be whatever pins you like!
#define TFT_MOSI 11   // set these to be whatever pins you like!
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

OneWire  ds(4);  // mikey was on pin 2

float tmpTemp;
float DS18B20_temp;

MLX90621 sensor; // create an instance of the Sensor class

void setup(){
  Serial.begin(9600);
 
  sensor.initialise (2);
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  testroundrects ;
  // delay(2000);
  tft.fillScreen(ST7735_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST7735_WHITE);
  // delay(2000);
  tft.fillScreen(ST7735_BLACK);
 }
void loop(){
  running = !running; // toggle running variable
  //String stringOne = "TA: ";
  //senArray[1] = senArray[1] + .001; //REMOVE TEST ONLY
 tft.fillScreen(ST7735_WHITE);
 tft.setTextSize(1);
  f_val = f_val + 0.001;
  dtostrf(f_val,7, 3, outstr);
  //testdrawtext(outstr, ST7735_GREEN);
  
  calculate_DS18B20();
  sensor.measure(true); //get new readings from the sensor
  Serial.println("IRSTART");
  Serial.println("MLX90621"); // modified
  for(int y=0;y<4;y++){ //go through all the rows

    for(int x=0;x<16;x++){ //go through all the columns
      //int16_t valueAtXY= sensor.irData[y+x*4]; // extract the temperature at position x/y
      //uint16_t color = getScaleValue(sensor.getTemperature(y+x*4), sensor.getMinTemp(), sensor.getMaxTemp());

      myLoop=++MyX;
       
      tmpTemp = (sensor.getTemperature(y+x*4));
      Serial.print(tmpTemp);
      tmpTemp = 0;
      Serial.print(",");
if (myLoop>15)
{
  // do something here
  MyX=0;
  myLoop=0;
  Serial.println();
}
         }
          
  }
  // original Serial.print("MLX96021,");
  tmpTemp = (sensor.getAmbient());

   tft.setCursor(0, 10);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(2);
  tft.println(tmpTemp);
  
  Serial.print("TA=");
  Serial.print(tmpTemp);
  Serial.print(",");

  tft.setCursor(0, 20);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(2);
  tft.println(tmpTemp);
  
  Serial.print("CPIX=");
  Serial.print(sensor.get_CPIX()); // GET VALUE MIKEY red shOW
  Serial.print(",");
  
  Serial.print("PTAT=");
  Serial.print("6646"); // GET VALUE MIKEY
  Serial.print(",");
  
  Serial.print("EMISSIVITY=");
  Serial.print("1"); // GET VALUE MIKEY
  Serial.print(",");
  
  Serial.print("V_TH=");
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
  delay(1);

//tft.setCursor(0, 10);
//  tft.setTextColor(ST7735_RED);
//  tft.setTextSize(3);
//  tft.println(DS18B20_temp);
 
};

float getScaleValue(float temp, float minTemp, float maxTemp){
  uint8_t scale = (31.0 / (maxTemp - minTemp))*(temp) + (-1.0) * minTemp * (31.0)/(maxTemp - minTemp);
  uint16_t color = scale << 11 | scale << 6 | 1 << 5 | scale;
  return color;
}

void calculate_DS18B20(){

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
  
  delay(100);     // maybe 750ms is enough, maybe not
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
  //Serial1111.print("  Temperature = ");
  //Serial1111.print(celsius);
  //Serial1111.print(" Celsius, ");
  //Serial1111.print(fahrenheit);
  //Serial1111.println(" Fahrenheit");
  
}

// TFT CODE STARTS HERE
void testroundrects() {
  tft.fillScreen(ST7735_BLACK);
  int color = 100;
  int i;
  int t;
  for(t = 0 ; t <= 4; t+=1) {
    int x = 0;
    int y = 0;
    int w = tft.width()-2;
    int h = tft.height()-2;
    for(i = 0 ; i <= 16; i+=1) {
      tft.drawRoundRect(x, y, w, h, 5, color);
      x+=2;
      y+=3;
      w-=4;
      h-=6;
      color+=1100;
    }
    color+=100;
  }
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}
