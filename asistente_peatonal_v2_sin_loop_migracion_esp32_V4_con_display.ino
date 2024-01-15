#include <analogWrite.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SoftwareSerial.h>
///display
#include <PNGdec.h>
#include "panda.h" // Image is stored here in an 8 bit array
PNG png; // PNG decoder inatance
#define MAX_IMAGE_WDITH 240 // Adjust for your images
int16_t xpos = 0;
int16_t ypos = 0;
#include "SPI.h"
#include <TFT_eSPI.h>              // Hardware-specific library
TFT_eSPI tft = TFT_eSPI();         // Invoke custom library

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0xC8,0x2B,0x96,0x2F,0x79,0x9A};
uint8_t broadcastAddress2[] = {0x48, 0x3F, 0xDA, 0x6F, 0xF8, 0xF9};
typedef struct test_struct {
  int x;
  int y;
} test_struct;

test_struct test;

esp_now_peer_info_t peerInfo;
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
 /// Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
 /// Serial.print(macStr);
  ///Serial.print(" send status:\t");
 /// Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
////////////////// GLOBAL  /////////////////////////////////////////
/// pines de motores delanteros  A b  traceros c d
/// delanteros a y b 
 int Ain1=15 ; int Ain2=5; int Bin1=17 ; int Bin2=16;
/// traceros  c y d 
 int Cin1=27 ; int Cin2=26; int Din1=25 ; int Din2=33;
/// Ens
int Aen=19;int Ben=21;int Cen=14;int Den=32;
/// velocidad
int velocidad =170; int str;
/// sensores magneticos
 int mag1=12;  int mag2=13;  int magstate1 = 0;  int magstate2 = 0;


///https://roboticsbackend.com/arduino-delay/
unsigned long lastTimeLedBlinked = millis();
unsigned long delayBetweenBlink = 14000 ;
int ubicacion=0;

//////////////////////////////////// SETUP ////////////////////////////////////
void setup() {
///Serial.begin(9600);
// Initialise the TFT
  tft.begin();
  tft.fillScreen(TFT_BLACK);
  
WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
////    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
  ///  Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
  ////  Serial.println("Failed to add peer");
    return;
  }
///// declaramos los pines de motores como OUTPUT
 pinMode(Ain1,OUTPUT);pinMode(Ain2,OUTPUT);
 pinMode(Bin1,OUTPUT);pinMode(Bin2,OUTPUT);
 pinMode(Cin1,OUTPUT);pinMode(Cin2,OUTPUT);
 pinMode(Din1,OUTPUT);pinMode(Din2,OUTPUT);
/// declaramos los pines de control de velocidad como OUTPUT
pinMode(Aen,OUTPUT);pinMode(Ben,OUTPUT);pinMode(Cen,OUTPUT);pinMode(Den,OUTPUT);
 ////declaramos los  pines magneticos como INPUT
 pinMode(mag1,INPUT);pinMode(mag2,INPUT); //// sensores magneticos
////// inciamos con los motores apagados 
 digitalWrite(Ain1,LOW);digitalWrite(Ain2,LOW); digitalWrite(Bin1,LOW);digitalWrite(Bin2,LOW); digitalWrite(Cin1,LOW);digitalWrite(Cin2,LOW); digitalWrite(Din1,LOW);digitalWrite(Din2,LOW);
/// el semasforo en estado 0
int semasforo = 0; str =int(semasforo);

///delay(3000);
}

//////////////////////////////////// LOOP   ////////////////////////////////////
void loop() {
/// magstate1 = digitalRead(mag1);
/// magstate2 = digitalRead(mag2);
//// testmotor ();
/*
if (magstate1 == HIGH & magstate2 == LOW   ){
 iradelante();
}
/// if (ubicacion == 1) {delay(10000);}
 if (magstate1 == LOW & magstate2 == HIGH   ){
 iratras();
} 
*/
/// display
  int16_t rc = png.openFLASH((uint8_t *)panda, sizeof(panda), pngDraw);
  if (rc == PNG_SUCCESS) {
    Serial.println("Successfully png file");
    Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());
    tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    Serial.print(millis() - dt); Serial.println("ms");
    tft.endWrite();
    // png.close(); // not needed for memory->memory decode
  }
  delay(2000);
  tft.fillScreen(random(0x10000));
/// display end
if (ubicacion == 0 ){
 iradelante();
}
if (ubicacion == 1 ){
iratras();}
if (ubicacion == 3 ){
 semaforo0 ();
}


///if (ubicacion == 1 ){
 


/// Serial.println(ubicacion);
/// unsigned long timeNow = millis();
///if (timeNow - lastTimeLedBlinked > delayBetweenBlink)
/// { lastTimeLedBlinked = timeNow;  }
///iradelante();
/// iratras();
}

//////////////////////////////////// FUNCIONES    ////////////////////////////////////
void iradelante () {
  //// establecemos la velocidad dependiendo de la variable velocidad 
analogWrite(Aen,velocidad);analogWrite(Ben,velocidad);analogWrite(Cen,velocidad);analogWrite(Den,velocidad);
semaforo2 ();
magstate1 = digitalRead(mag1);

 if (magstate1 == HIGH) { digitalWrite(Ain1,LOW);digitalWrite(Ain2,LOW); digitalWrite(Bin1,LOW);digitalWrite(Bin2,LOW); digitalWrite(Cin1,LOW);digitalWrite(Cin2,LOW); digitalWrite(Din1,LOW);digitalWrite(Din2,LOW); 
ubicacion=1;delay(10000);  } else { digitalWrite(Ain1,HIGH);digitalWrite(Ain2,LOW);digitalWrite(Bin1,HIGH);digitalWrite(Bin2,LOW); //// iadelante motores delanteros a y b
digitalWrite(Cin1,HIGH);digitalWrite(Cin2,LOW);digitalWrite(Din1,HIGH);digitalWrite(Din2,LOW); //// iadelante motores traceros c y d

}
return;
}


void iratras () {
//// establece1mos la velocidad dependiendo de la variable velocidad 
analogWrite(Aen,velocidad);analogWrite(Ben,velocidad);analogWrite(Cen,velocidad);analogWrite(Den,velocidad);
semaforo1 ();
magstate2 = digitalRead(mag2);

 if (magstate2 == HIGH) { digitalWrite(Ain1,LOW);digitalWrite(Ain2,LOW); digitalWrite(Bin1,LOW);digitalWrite(Bin2,LOW); digitalWrite(Cin1,LOW);digitalWrite(Cin2,LOW); digitalWrite(Din1,LOW);digitalWrite(Din2,LOW);
ubicacion=3;} else {digitalWrite(Ain1,LOW);digitalWrite(Ain2,HIGH);  digitalWrite(Bin1,LOW);digitalWrite(Bin2,HIGH);                 //// iadelante motores delanteros a y b
digitalWrite(Cin1,LOW);digitalWrite(Cin2,HIGH);digitalWrite(Din1,LOW);digitalWrite(Din2,HIGH); //// iadelante motores traceros c y d

}
return;  
}

void semaforo0 (){int semasforo = 0;
test.x = (semasforo);
test.y = (semasforo);
esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
   
  if (result == ESP_OK) {
   // Serial.println("Sent with success");
  }
  else {
   /// Serial.println("Error sending the data");
  }
///  delay(2000);
}

void semaforo1 (){int semasforo = 1;
test.x = (semasforo);
test.y = (semasforo);
esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
   
  if (result == ESP_OK) {
  ///  Serial.println("Sent with success");
  }
  else {
   /// Serial.println("Error sending the data");
  }
///  delay(2000);
}

void semaforo2 () {int semasforo = 2;
test.x = (semasforo);
test.y = (semasforo);
esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
   
  if (result == ESP_OK) {
 ///   Serial.println("Sent with success");
  }
  else {
  ///  Serial.println("Error sending the data");
  }
///  delay(2000);
}



//=========================================v==========================================
//                                      pngDraw
//====================================================================================
// This next function will be called during decoding of the png file to
// render each image line to the TFT.  If you use a different TFT library
// you will need to adapt this function to suit.
// Callback function to draw pixels to the display
void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WDITH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}
