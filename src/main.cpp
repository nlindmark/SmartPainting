/* ESP8266/32 Audio Spectrum Analyser on an SSD1306/SH1106 Display
 * The MIT License (MIT) Copyright (c) 2017 by David Bird. 
 * The formulation and display of an AUdio Spectrum using an ESp8266 or ESP32 and SSD1306 or SH1106 OLED Display using a Fast Fourier Transform
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files 
 * (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, but not to use it commercially for profit making or to sub-license and/or to sell copies of the Software or to 
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:  
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 * See more at http://dsbird.org.uk 
*/

#include <Arduino.h>
#include <Wire.h>
#include "arduinoFFT.h" // Standard Arduino FFT library
#include "credentials.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <FastLED.h>      //include the FastLED Library

// https://github.com/kosme/arduinoFFT, in IDE, Sketch, Include Library, Manage Library, then search for FFT
arduinoFFT FFT = arduinoFFT();


#include <math.h>         //include library for mathematic funcions
#define DATA_PIN D2        //DATA PIN WHERE YOUR LEDS ARE CONNECTED
#define COLS 7
#define ROWS 7
#define NUM_LEDS COLS * ROWS      //amount of LEDs in your matrix
#define SECOND 60 
CRGB leds[NUM_LEDS];



//#include "SSD1306.h"  // https://github.com/squix78/esp8266-oled-ssd1306
//SSD1306 display(0x3c, D3,D4);  // 0.96" OLED display object definition (address, SDA, SCL) Connect OLED SDA , SCL pins to ESP SDA, SCL pins
/////////////////////////////////////////////////////////////////////////

// Initialize the OLED display using SPI
// D5 -> CLK
// D7 -> MOSI (DOUT)
// D0 -> RES
// D2 -> DC
// D8 -> CS

 #include "SSD1306Spi.h"
// #include "SH1106SPi.h"

 SSD1306Spi        display(D3, D1, D8);
// or
// SH1106Spi         display(D0, D2);


#define SAMPLES 256              //Must be a power of 2
#define SAMPLING_FREQUENCY 10000 //Hz, must be 10000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT.
#define amplitude 50

void displayBand(int band, int dsize);
void drawBand(int band, int size);
void drawBand(int band, int size, CRGB color);
void drawPixel(int x, int y, CRGB color);
void displayBoth(int dsize,int band);
long map2(long x, long in_min, long in_max, long out_min, long out_max);
void clearLights();
bool setup_wifi();
boolean setup_mqtt();

boolean poweron = true;
int dimlevel = 180;


unsigned int sampling_period_us;
unsigned long microseconds;
byte peak[] = {0,0,0,0,0,0,0};
byte peak2[]= {0,0,0,0,0,0,0};
double freqBoost[]= {0.6,0.7,1.2,1.8,2.2,1.9,1.9};
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime, oldTime;
char ssid[] = SSIDNAME;
char password[] = PASSWORD;
WiFiClient espClient;
IPAddress mqttServer(192, 168, 1, 200);
void mqttCallback(char* topic, byte* payload, unsigned int length);
PubSubClient mqttClient(mqttServer, 1883, mqttCallback, espClient);


/////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  //Wire.begin(5,4); // SDA, SCL

  setup_wifi();

  FastLED.addLeds<WS2812B, DATA_PIN, GRB> (leds, NUM_LEDS);
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.flipScreenVertically(); // Adjust to suit or remove
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
}

void loop() {

  if (!mqttClient.connected()) {
    setup_mqtt(); 
     
     } 
  else {
    // Client connected
    mqttClient.loop();
  }

  if(poweron){

  

    display.clear();
    display.drawString(0,0,"0.1 0.2 0.5 1K  2K  4K  8K");

    clearLights();


    for (int i = 0; i < SAMPLES; i++) {
      newTime = micros()-oldTime;
      oldTime = newTime;
      vReal[i] = analogRead(A0); // A conversion takes about 1mS on an ESP8266
      vImag[i] = 0;
      while (micros() < (newTime + sampling_period_us)) { /* do nothing to wait */ }
    }
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    for (int i = 2; i < (SAMPLES/2); i++){ // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
      if (vReal[i] > 200) { // Add a crude noise filter, 4 x amplitude or more
        if (i<=5 )             displayBoth(0,(int)vReal[i]/amplitude); // 125Hz
        if (i >5   && i<=12 )  displayBoth(1,(int)vReal[i]/amplitude); // 250Hz
        if (i >12  && i<=32 )  displayBoth(2,(int)vReal[i]/amplitude); // 500Hz
        if (i >32  && i<=62 )  displayBoth(3,(int)vReal[i]/amplitude); // 1000Hz
        if (i >62  && i<=105 ) displayBoth(4,(int)vReal[i]/amplitude); // 2000Hz
        if (i >105 && i<=120 ) displayBoth(5,(int)vReal[i]/amplitude); // 4000Hz
        if (i >120 && i<=146 ) displayBoth(6,(int)vReal[i]/amplitude); // 8000Hz
        //Serial.println(i);
      }
      for (byte band = 0; band <= 6; band++) display.drawHorizontalLine(18*band,64-peak[band],14);
    }
    if (millis()%4 == 0) {
      for (byte band = 0; band <= 6; band++) {if (peak[band] > 0) peak[band] -= 1;}
      for (byte band = 0; band <= 6; band++) {if (peak2[band] > 0) peak2[band] -= 1;}
    } 
    // Decay the peak
    display.display();
    FastLED.show();

  }
   
}


void displayBoth(int band, int size){
  displayBand(band,size);

  
  int val = map2(size * freqBoost[band],0,35,0,ROWS);
  if (val > peak2[band]) {peak2[band] = val;}

  CRGB barColor = CHSV(band * 35,200,dimlevel);
  CRGB peakColor = CHSV(band * 35,200,dimlevel);
  drawBand(band,val,barColor);
  //drawPixel(band,peak2[band],peakColor);
}


long map2(long x, long in_min, long in_max, long out_min, long out_max)
{
  if(x < in_min){
    x = in_min;
  } else if (x> in_max){
    x = in_max;
  }

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
 

//flip x,y -> y,x for portrait
//flip y,x -> x,y for landscape
void drawPixel(int y, int x, CRGB color){
  int pixelIndex = 0;

  int fullCols = x * ROWS;
  pixelIndex += fullCols;

  if(x % 2 == 0){
    pixelIndex += y;
  }else{
    pixelIndex += ROWS - y - 1;
  }

  if(pixelIndex >= NUM_LEDS){
    pixelIndex = NUM_LEDS - 1;
  }
  
  leds[pixelIndex] = color;
    
}

void clearLights(){
  for(int i= 0; i < COLS; i++){
    drawBand(i,ROWS,CRGB::Black);
  } 
}

void flashLights(){

  CRGB c[] = {CRGB::DarkBlue, CRGB::DarkCyan, CRGB::DarkGoldenrod, CRGB::DarkGreen, CRGB::DarkMagenta, CRGB::DarkOrange, CRGB::DarkViolet};
  for(int i= 0; i < COLS; i++){
    drawBand(i,ROWS,c[i]);
  } 
}

void drawBand(int band, int size, CRGB color){
    for(int i = 0; i < ROWS; i++){
    if(i < size){
      drawPixel(ROWS - band - 1,i,color); //mirror the bands. Bass frequencies to the left
    }else{
      drawPixel(ROWS - band - 1,i,CRGB::Black);
    }
  }
}


void drawBand(int band, int size){  
  drawBand(band,size,CHSV(band * 35,200,200));
}



void displayBand(int band, int dsize){
  int dmax = 50;
  if (dsize > dmax) dsize = dmax;
  for (int s = 0; s <= dsize; s=s+2){display.drawHorizontalLine(18*band,64-s, 14);}
  if (dsize > peak[band]) {peak[band] = dsize;}

}




bool setup_wifi() {

  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SSIDNAME);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempt = 30;

  while(attempt){
    if(WiFi.status() == WL_CONNECTED){

    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
    }
    delay(500);
    attempt--;
  }

return false;
}

boolean setup_mqtt(){

  char clientName[50];


    snprintf (clientName, 50, "%ld", system_get_chip_id());

    if (mqttClient.connect(clientName, MQTTUSER, MQTTPASSWORD)) {
      char str[30];
      strcpy(str, "The Frame is connected to MQTT ");
      mqttClient.publish("stat/frame/hello", str);
      mqttClient.subscribe("cmnd/frame/power");
      mqttClient.subscribe("cmnd/frame/state");
    } else {

      delay(1000);
      Serial.println("Connecting to MQTT server ...");

    }
  
  return mqttClient.connected();
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived

  // Internally, the mqtt client uses the same buffer for both inbound and outbound
  // messages. The application should create its own copy of the values if they
  // are required beyond this.
  char t[50], p[50], buffer[50];
  snprintf (t, 50, "%s", topic);
  snprintf (p, 50, "%s",payload);
  Serial.println(t);
  Serial.println(p);
  Serial.println(length);

  p[length] = '\n';

  if(strcmp(t, "cmnd/frame/power") == 0){
    if(strncmp(p, "ON",length) == 0){
       poweron = true;
       flashLights();
       FastLED.show();
       delay(300);
       clearLights();
       FastLED.show();
       Serial.println("OK Turning ON");
    } else if(strncmp(p, "OFF",length) == 0) {
       poweron = false;
       flashLights();
       FastLED.show();
       delay(300);
       clearLights();
       FastLED.show();
       Serial.println("OK Turning OFF");
    } else if(strstr(p, ",") != NULL) {
       Serial.println("OK Changing color");
    }
    else {
       
       char dest[length];
       dimlevel = map2(atoi(strncpy(dest, p, length)), 0, 100, 0, 255);
       Serial.println("OK Dimming it to ");
       Serial.println(dimlevel);
    }

  } else if (strcmp(t, "cmnd/frame/state") == 0) {
     mqttClient.publish("stat/frame/state", "Status", true);
  }
}





