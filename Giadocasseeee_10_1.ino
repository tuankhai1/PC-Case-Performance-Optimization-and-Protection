#define BLYNK_TEMPLATE_ID "TMPL6vQzZ7XRJ"
#define BLYNK_TEMPLATE_NAME "giadocase"
#define BLYNK_AUTH_TOKEN "604EpECLnXPOzLM0ce_9XfFTLrgU5pPG"

#include <Adafruit_NeoPixel.h>
#include <BlynkSimpleEsp32.h>
#include <WiFiManager.h>
#include <DHT.h>
#include <HardwareSerial.h>


#define LED_PIN 32      
#define NUM_PIXELS 8    
#define Relay1 13       
#define Relay2 14       
#define coi 19          
#define DHTPIN 18        
#define DHTTYPE DHT11   
#define ZH03B_RX 16     
#define ZH03B_TX 17     
#define BLYNK_PRINT Serial

Adafruit_NeoPixel strip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
DHT dht(DHTPIN, DHTTYPE);
HardwareSerial zh03bSerial(2); 
WiFiManager wm;


float temp, humid, PM2_5;
BlynkTimer timer;

void setup() {
  Serial.begin(115200);
  Serial.println("Connecting to Wi-Fi...");
  bool res = wm.autoConnect("AutoConnectAP", "12345678");
  if (!res) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();  
  } else {
    Serial.println("Connected to Wi-Fi successfully");
  }

  String ssid = WiFi.SSID(); 
  String pass = WiFi.psk();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid.c_str(), pass.c_str());
  dht.begin();
  zh03bSerial.begin(9600, SERIAL_8N1, ZH03B_RX, ZH03B_TX);

  strip.begin();
  strip.show();

  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(coi, OUTPUT);

  timer.setInterval(2000L, readSensorData);
  timer.setInterval(500L, checkThresholds);
}

void loop() {
  Blynk.run();
  timer.run();
}

BLYNK_WRITE(V4) {
  int buttonState = param.asInt();
  digitalWrite(Relay1, buttonState);
  Serial.print("Relay1 (Quạt): ");
  Serial.println(buttonState ? "ON" : "OFF");
}

BLYNK_WRITE(V5) {
  int buttonState = param.asInt();
  digitalWrite(Relay2, buttonState);
  Serial.print("Relay2 (Phun sương): ");
  Serial.println(buttonState ? "ON" : "OFF");
}

void readSensorData() {
  temp = dht.readTemperature();
  humid = dht.readHumidity();

  if (isnan(humid) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  uint8_t data[9];
  if (zh03bSerial.available() >= 9) {
    zh03bSerial.readBytes(data, 9);
    if (data[0] == 0x42 && data[1] == 0x4D) {  
      uint16_t checksum = 0;
      for (int i = 0; i < 8; i++) {
        checksum += data[i];
      }
      uint16_t receivedChecksum = (data[7] << 8) | data[8];
      if (checksum == receivedChecksum) {  
        PM2_5 = (data[4] << 8) | data[5];
      } else {
        Serial.println("Checksum error in PM sensor data!");
        PM2_5 = 0;
      }
    }
  } else {
    PM2_5 = 0;
    Serial.println("Invalid data from PM sensor!");
  }

  Serial.print("Humidity: ");
  Serial.print(humid);
  Serial.print("% || Temperature: ");
  Serial.print(temp);
  Serial.print("°C || Dust: ");
  Serial.print(PM2_5);
  Serial.println(" µg/m³");

  Blynk.virtualWrite(V0, temp);
  Blynk.virtualWrite(V1, humid);
  Blynk.virtualWrite(V2, PM2_5);
}



void checkThresholds() {
  if (temp > 35 || PM2_5 > 100) {  
    digitalWrite(coi, HIGH);
    setNeoPixelColor(255, 0, 0); 
  } else if (humid < 30) {  
    digitalWrite(coi, HIGH);
    setNeoPixelColor(0, 0, 255); 
  } else {  
    digitalWrite(coi, LOW);
    setNeoPixelColor(0, 255, 0);
  }
}

void setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
    strip.show();
    delay(50);  
  }
}

