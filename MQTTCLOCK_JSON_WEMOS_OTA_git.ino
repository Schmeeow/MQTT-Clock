#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <TM1637Display.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// YOU MUST INCREASE *MQTT_KEEPALIVE* VALUE in PubSubClient.h LIBRARY TO GET IT WORKING. TO 120[SECONDS], FOR INSTANCE
// YOU MUST INCREASE *MQTT_MAX_PACKET_SIZE* in PubSubClient.h TO 256 FOR BIG JSON DATA

unsigned long lastMillis = 0;

/// WIFI & MQTT CONFIG
IPAddress mqttServer(192, 168, 1, 100);
const char* wifiNetwork = "schmeeow";
const char* wifiPassword = "12345678";
const char* mqttClientName = "MQTT Clock v0.99";
const char* mqttUser = "homeassistant";
const char* mqttPassword = "123456789";
const char* mqttInTopic = "republish/json";
const char* mqttOutTopic = "sensor/mqttclock";
const int displaySeconds = 8;
const int defaulBrightness = 0;

// TM1637 GPIO NUMBERS CONFIG
#define leftDisplayCLK 13 // D7
#define leftDisplayDIO 12 // D6
#define midDisplayCLK 14  // D5
#define midDisplayDIO 2   // D4
#define rightDisplayCLK 0 // D3
#define rightDisplayDIO 4 // D2

// PREDEFINED MESSAGES & SYMBOLS
const uint8_t SEG_CONN[] = { SEG_A | SEG_F | SEG_E | SEG_D, SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_A | SEG_B | SEG_C | SEG_F | SEG_E, SEG_A | SEG_B | SEG_C | SEG_F | SEG_E };
const uint8_t SEG_FAIL[] = { SEG_A | SEG_F | SEG_G | SEG_E, SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F, SEG_B | SEG_C, SEG_F | SEG_D | SEG_E };
const uint8_t SEG_DATA[] = { SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F, SEG_F | SEG_E | SEG_G | SEG_D, SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F };
const uint8_t SEG_NET[] = { SEG_A | SEG_B | SEG_C | SEG_F | SEG_E, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G, SEG_F | SEG_E | SEG_G | SEG_D, SEG_E };
const uint8_t SEG_DONE[] = { SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_A | SEG_B | SEG_C | SEG_F | SEG_E, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G };
const uint8_t SEG_LOAD[] = { SEG_F | SEG_D | SEG_E, SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F, SEG_B | SEG_C | SEG_D | SEG_E | SEG_G };
const uint8_t SEG_JSON[] = { SEG_B | SEG_C | SEG_D, SEG_A | SEG_F | SEG_G | SEG_C | SEG_D, SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_A | SEG_B | SEG_C | SEG_F | SEG_E };
const uint8_t SEG_DEGREE[] = { SEG_A | SEG_B | SEG_G | SEG_F };
const uint8_t SEG_PERCENT[] = { SEG_A | SEG_B | SEG_G | SEG_F, SEG_G | SEG_C | SEG_D | SEG_E };
const uint8_t SEG_WIND[] = { SEG_A | SEG_G | SEG_D };
const uint8_t SEG_PRESSURE_LOW[] = { SEG_D | SEG_E };
const uint8_t SEG_PRESSURE_HIGH[] = { SEG_A | SEG_F };

const uint8_t SEG_UPDATE[] = { SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_A | SEG_B | SEG_G | SEG_F | SEG_E, SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, SEG_F | SEG_E | SEG_G | SEG_D };
const uint8_t SEG_INIT[] = { SEG_B | SEG_C, SEG_A | SEG_B | SEG_C | SEG_F | SEG_E, SEG_B | SEG_C, SEG_F | SEG_E | SEG_G | SEG_D };
const uint8_t SEG_END[] = { SEG_A | SEG_D | SEG_E | SEG_F | SEG_G, SEG_A | SEG_B | SEG_C | SEG_F | SEG_E, SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, SEG_E };

const uint8_t SEG_TRANSFER[] = { SEG_F | SEG_E | SEG_G | SEG_D, SEG_E | SEG_G, SEG_A | SEG_F | SEG_G | SEG_C | SEG_D, SEG_A | SEG_F | SEG_G | SEG_E, SEG_B | SEG_C };
const uint8_t SEG_AUTH[] = { SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F, SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_F | SEG_E | SEG_G | SEG_D, SEG_B | SEG_C | SEG_F | SEG_E | SEG_G };
const uint8_t SEG_SKETCH[] = { SEG_A | SEG_F };
const uint8_t SEG_FILE[] = { SEG_A | SEG_F | SEG_G | SEG_E, SEG_B | SEG_C, SEG_F | SEG_D | SEG_E, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G };
const uint8_t SEG_TYPE[] = { SEG_F | SEG_E | SEG_G | SEG_D, SEG_F | SEG_G | SEG_B | SEG_C | SEG_D, SEG_A | SEG_B | SEG_G | SEG_F | SEG_E, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G};

TM1637Display displayLeft(leftDisplayCLK, leftDisplayDIO);
TM1637Display displayMiddle(midDisplayCLK, midDisplayDIO);
TM1637Display displayRight(rightDisplayCLK, rightDisplayDIO);

// ON MESSAGES RECEIVED
void callback(char* topic, byte* payload, unsigned int length) {

  // TIME FUNCTION CALLED. SHOULD BE CALLED AGAIN IN DISPLAYTIME*3
  lastMillis = millis();

  // CONVERTING JSON
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    displayPhrase(SEG_JSON, SEG_DATA, SEG_FAIL);  // WRONG JSON | UNKNOWN DATA
  }

  int hours = doc["Time"][0];
  int minutes = doc["Time"][1];
  int ambientLight = doc["Time"][2];
  int playroomTemp = doc["Playroom"][0];
  int playroomHum = doc["Playroom"][1];
  int outdoorTemp = doc["Outdoor"][0];
  int outdoorHum = doc["Outdoor"][1];
  int atmPressure = doc["Outdoor"][2];
  int windSpeed = doc["Outdoor"][3];

  // CLEAR DISPLAYS
  displayLeft.clear();
  displayMiddle.clear();
  displayRight.clear();

  // BRIGHTNESS BASED ON INCOMING DATA
  if (ambientLight < 10) {
    setBrightnessAll(0);
  }
  else if (ambientLight > 30) {
    setBrightnessAll(7);
  }
  else {
    setBrightnessAll(4);
  }

  /* BRIGHTNESS BASED ON TIME
    if (hours <= 8 || hours >= 23) { setBrightnessAll(0); }
      else if (hours >= 12 && hours <= 16) { setBrightnessAll(7); }
           else { setBrightnessAll(4); } */

  // DISPLAY PLAYROOM TEMP & HUMIDITY
  displayLeft.setSegments(SEG_DEGREE, 1, 3);
  displayLeft.showNumberDec(playroomTemp, false, 2, 1);
  displayMiddle.showNumberDecEx(hours * 100 + minutes, 0b01000000, true, 4, 0);
  displayRight.setSegments(SEG_PERCENT, 2, 2);
  displayRight.showNumberDec(playroomHum, false, 2, 0);
  delay(displaySeconds * 1000);

  // DISPLAY OUTDOOR TEMP & HUMIDITY
  displayLeft.clear();
  displayRight.clear();
  displayLeft.setSegments(SEG_DEGREE, 1, 3);
  displayLeft.showNumberDec(outdoorTemp, false, 2, 1);
  displayRight.setSegments(SEG_PERCENT, 2, 2);
  displayRight.showNumberDec(outdoorHum, false, 2, 0);
  delay(displaySeconds * 1000);

  // DISPLAY OUTDOOR WIND $ PRESSURE
  displayLeft.clear();
  displayRight.clear();
  displayLeft.setSegments(SEG_WIND, 1, 0);
  // PRINT WINDSYMBOL
  if (windSpeed > 5) {
    displayLeft.setSegments(SEG_WIND, 1, 1);
  }
  displayLeft.showNumberDec(windSpeed, false, 2, 2);
  // PRINT HI OR LOW PRESSURE SYMBOL
  if (atmPressure > 750) {
    displayRight.setSegments(SEG_PRESSURE_HIGH, 1, 0);
  }
  else if (atmPressure < 750) {
    displayRight.setSegments(SEG_PRESSURE_LOW, 1, 0);
  }
  displayRight.showNumberDec(atmPressure, false, 3, 1);
  delay(displaySeconds * 1000);
}

// SETUP
WiFiClient wifiConnection;
PubSubClient client(mqttServer, 1883, callback, wifiConnection);

void setup() {
  Serial.begin(115200);
  WiFi.begin(wifiNetwork, wifiPassword);
  //WiFi.softAPdisconnect(true);
  displayLeft.clear();
  displayMiddle.clear();
  displayRight.clear();
  setBrightnessAll(0);
  connect();
  // OTA
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("MQTTClock");
  ArduinoOTA.onStart([]() {
    if (ArduinoOTA.getCommand() == U_FLASH) {
      displayPhrase(SEG_UPDATE, SEG_TYPE, SEG_SKETCH);
    } else { // U_FS
      displayPhrase(SEG_UPDATE, SEG_TYPE, SEG_FILE);
    }
  });
  ArduinoOTA.onEnd([]() {
    displayPhrase(SEG_UPDATE, SEG_DATA, SEG_END);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    displayPhrase(SEG_UPDATE, SEG_UPDATE, SEG_UPDATE);
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      displayPhrase(SEG_UPDATE, SEG_AUTH, SEG_FAIL);
    } else if (error == OTA_BEGIN_ERROR) {
      displayPhrase(SEG_UPDATE, SEG_INIT, SEG_FAIL);
    } else if (error == OTA_CONNECT_ERROR) {
      displayPhrase(SEG_UPDATE, SEG_CONN, SEG_FAIL);
    } else if (error == OTA_RECEIVE_ERROR) {
      displayPhrase(SEG_UPDATE, SEG_TRANSFER, SEG_FAIL);
    } else if (error == OTA_END_ERROR) {
      displayPhrase(SEG_UPDATE, SEG_END, SEG_FAIL);
    }
  });
  ArduinoOTA.begin();
}

// WRITE TO ALL THREE DISPLAYS
void displayPhrase(const uint8_t* leftText, const uint8_t* middleText, const uint8_t* rightText)
{
  displayLeft.clear();
  displayMiddle.clear();
  displayRight.clear();
  displayLeft.setSegments(leftText, 4, 0);
  displayMiddle.setSegments(middleText, 4, 0);
  displayRight.setSegments(rightText, 4, 0);
}

void setBrightnessAll(int brightness)
{
  displayLeft.setBrightness(brightness); displayMiddle.setBrightness(brightness); displayRight.setBrightness(brightness);
}

// CONNECTION

void connect() {
  setBrightnessAll(7);
  displayPhrase(SEG_NET, SEG_CONN, SEG_FAIL);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  displayPhrase(SEG_NET, SEG_CONN, SEG_DONE);
  delay(1000);
  displayPhrase(SEG_DATA, SEG_CONN, SEG_FAIL);
  while (!client.connect(mqttClientName, mqttUser, mqttPassword)) {
    delay(1000);
  }
  displayPhrase(SEG_DATA, SEG_CONN, SEG_DONE);
  delay(1500);
  displayPhrase(SEG_JSON, SEG_DATA, SEG_LOAD);
  client.subscribe(mqttInTopic);
  client.publish(mqttOutTopic, "online");
  delay(2000);
  displayPhrase(SEG_UPDATE, SEG_START, SEG_END);
  delay(5000);
  displayPhrase(SEG_UPDATE, SEG_TYPE, SEG_FILE);
  delay(5000);
  displayPhrase(SEG_INIT, SEG_FAIL, SEG_TRANSFER);
  delay(5000);
}

// LOOP

void loop()
{
  if (!client.connected()) {
    displayPhrase(SEG_DATA, SEG_CONN, SEG_FAIL);
    delay(1000);
    connect();
  }
  if ((millis() - lastMillis) > (3 * displaySeconds * 1000 + 1000)) {
    client.subscribe(mqttInTopic);
    client.publish(mqttOutTopic, "resubscribing");
  }
  ArduinoOTA.handle();
  client.loop();
}
