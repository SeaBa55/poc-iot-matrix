#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <Adafruit_Protomatter.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>

const char* ssid = "NETGEAR94";
const char* password = "imaginaryrabbit652";

uint8_t rgbPins[]  = {4, 12, 13, 14, 15, 21};
uint8_t addrPins[] = {16, 17, 25, 26}; //32x32
// uint8_t addrPins[] = {16, 17, 25, 26, 35}; //64x64
uint8_t clockPin   = 27; // Must be on same port as rgbPins
uint8_t latchPin   = 32;
uint8_t oePin      = 33;

Adafruit_Protomatter matrix( //32x32
  32,          // Width of matrix (or matrix chain) in pixels
  6,           // Bit depth, 1-6
  1, rgbPins,  // # of matrix chains, array of 6 RGB pins for each
  4, addrPins, // # of address pins (height is inferred), array of pins
  clockPin, latchPin, oePin, // Other matrix control pins
  true        // No double-buffering here (see "doublebuffer" example)
);      

// Adafruit_Protomatter matrix( //64x64
//   64,          // Width of matrix (or matrix chain) in pixels
//   4,           // Bit depth, 1-6
//   1, rgbPins,  // # of matrix chains, array of 6 RGB pins for each
//   4, addrPins, // # of address pins (height is inferred), array of pins
//   clockPin, latchPin, oePin, // Other matrix control pins
//   true        // No double-buffering here (see "doublebuffer" example)
// );    

// pikachu_gif_ram_resident 32x32px rgb 565 example
// uint16_t img[][1024] = {};

void setup() {
  // set the LED pin mode
  // pinMode(5, OUTPUT);
  // pinMode(13, OUTPUT);

  // put your setup code here, to run once:
  WiFi.begin(ssid, password);

  Serial.begin(115200);
  Serial.print("Connecting to WiFi");

  // Initialize matrix...
  ProtomatterStatus status = matrix.begin();
  Serial.print("Protomatter begin() status: ");
  Serial.println((int)status);
  if(status != PROTOMATTER_OK) {
    // DO NOT CONTINUE if matrix setup encountered an error.
    for(;;);
  };
  
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    // digitalWrite(13, true);
    delay(250);
    // digitalWrite(13, false);
    delay(250);
  }

  Serial.println("\nConnected to WiFi network");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    // digitalWrite(13, false);
    HTTPClient client;
    client.begin("https://poc-iot-webapp.herokuapp.com/api");
    
    int httpCode = client.GET();

    if (httpCode > 0) {
      String payload = client.getString();
      Serial.println("\nStatus code: " + String(httpCode));
      // Serial.println("\n" + String(payload));

      DynamicJsonDocument doc(24576); //32x32
      // DynamicJsonDocument doc(98304); //64x64

      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
      }

      JsonObject message_0 = doc["message"][0];
      const char* message_0_name = message_0["name"]; // "tumblr_mahd2lV27V1rfjowdo1_500"
      char* prev_message_name;
      // const char* message_0_size = message_0["size"]; // "32x32"

      JsonArray message_0_file = message_0["file"];

      // extract the values
      // JsonArray array = doc["message"][0]["file"].as<JsonArray>();
      // Serial.println(array);
      // for(JsonVariant v : array) {
      //   Serial.print(v.as<uint16_t>());
      // }

      Serial.println(message_0_name);
      Serial.println(prev_message_name);
      if(message_0_name != prev_message_name) {
        uint16_t img[message_0_file.size()];
        size_t len = message_0_file.size();

        copyArray(doc["message"][0]["file"].as<JsonArray>(), img, len);
        Serial.println(char(*img));
        Serial.println(len);
        
        matrix.drawRGBBitmap( 0, 0, img, 32, 32); //32x32
        // matrix.drawRGBBitmap( 0, 0, img, 64, 64); //64x64
        matrix.show();

      }else{
        Serial.println("skipped");
      }

      // size_t charLen = message_0["name"].size();
      // memcpy(prev_message_name, (char*)message_0_name, charLen);

      // char message_0 = 's';
      // char prev_messsage = message_0;


      // char prev_massage = new char (message_0);

      // prev_message_name = (char*)message_0_name;
      Serial.println(message_0_name);
      Serial.println(prev_message_name);

    } else {
      Serial.println("Error on HTTP request");
    }

  } else {
    Serial.println("Connection lost");
  }

  delay(5000);
}