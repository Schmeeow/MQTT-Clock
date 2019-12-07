#include <ESP8266WiFi.h> // Базовая библиотека микроконтроллера
#include <ArduinoJson.h> // Библиотека для работы с JSON
#include <TM1637Display.h> // Библиотека для работы с дисплеями
#include <PubSubClient.h> // В библиотеке PubSubClient.h необходимо увеличить значения MQTT_KEEPALIVE до 120 и MQTT_MAX_PACKET_SIZE до 256
#include <ESP8266mDNS.h> // Нужна для прошивки Over-The-Air
#include <WiFiUdp.h> // Нужна для прошивки Over-The-Air
#include <ArduinoOTA.h> // Нужна для прошивки Over-The-Air

// ОСНОВНЫЕ НАСТРОЙКИ
const char* wifiNetwork = "YourWiFiSSID"; // Название сети WiFi
const char* wifiPassword = "YourWiFiPassword"; // Пароль сети WiFi
IPAddress   mqttServer(192, 168, 1, 1); // IP-адрес MQTT-брокера
const char* clientName = "Smart Clock v0.99"; // Имя клиента
const char* mqttUser = "YourMQTTUser"; // Имя пользователя для MQTT-брокера
const char* mqttPassword = "YourMQTTPassword"; // Пароль для MQTT-брокера
const char* mqttInTopic = "republish/json"; // Топик для получения данных
const int   OTAUpdatePort = 8266; // Номер порта для прошивки Over-The-Air
const char* OTAUserName = "Smart Clock v0.99"; // Название клиента для прошивки Over-The-Air
const char* OTAUserPassword = "YourOTAPassword"; // Номер порта для прошивки Over-The-Air

// НАСТРОЙКИ ОТОБРАЖЕНИЯ
const int dataDisplayTime = 10000; // Сколько показывать основные данные, мс
const int statusDisplayTime = 500; // Сколько показывать сообщения статуса, мс
const int defaulBrightness = 0; // Яркость дисплеев по умолчанию (0~7)

// НОМЕРА GPIO КОНТАКТОВ ДИСПЛЕЕВ TM1637
#define leftDisplayCLK 2 // Левый дисплей CLK // D4
#define leftDisplayDIO 0 // Левый дисплей DIO // D3
#define midDisplayCLK 4  // Средний дисплей CLK // D2
#define midDisplayDIO 5   // Средний дисплей DIO // D1
#define rightDisplayCLK 16 // Правый дисплей CLK // D0
#define rightDisplayDIO 14 // Правый дисплей DIO // D5

//  НОМЕРА GPIO КОНТАКТОВ RGB-СВЕТОДИОДА
#define redPin 15 // Красный // D8
#define greenPin 13 // Зеленый  // D5
#define bluePin 12 // Синий // D6

// НАБОРЫ СИМВОЛОВ ДЛЯ ДИСПЛЕЕВ
const uint8_t SEG_CONN[] = { SEG_A | SEG_F | SEG_E | SEG_D, SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_A | SEG_B | SEG_C | SEG_F | SEG_E, SEG_A | SEG_B | SEG_C | SEG_F | SEG_E };
const uint8_t SEG_FAIL[] = { SEG_A | SEG_F | SEG_G | SEG_E, SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F, SEG_B | SEG_C, SEG_F | SEG_D | SEG_E };
const uint8_t SEG_DATA[] = { SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F, SEG_F | SEG_E | SEG_G | SEG_D, SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F };
const uint8_t SEG_NETW[] = { SEG_A | SEG_B | SEG_C | SEG_F | SEG_E, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G, SEG_F | SEG_E | SEG_G | SEG_D, SEG_F | SEG_B | SEG_D };
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
const uint8_t SEG_SAVE[] = { SEG_A | SEG_F | SEG_G | SEG_C | SEG_D, SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F, SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G };
const uint8_t SEG_AUTH[] = { SEG_A | SEG_B | SEG_C | SEG_G | SEG_E | SEG_F, SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_F | SEG_E | SEG_G | SEG_D, SEG_B | SEG_C | SEG_F | SEG_E | SEG_G };
const uint8_t SEG_FILE[] = { SEG_A | SEG_F | SEG_G | SEG_E, SEG_B | SEG_C, SEG_F | SEG_D | SEG_E, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G };
const uint8_t SEG_TYPE[] = { SEG_F | SEG_E | SEG_G | SEG_D, SEG_F | SEG_G | SEG_B | SEG_C | SEG_D, SEG_A | SEG_B | SEG_G | SEG_F | SEG_E, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G};
const uint8_t SEG_CODE[] = { SEG_A | SEG_F | SEG_E | SEG_D, SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, SEG_A | SEG_D | SEG_E | SEG_F | SEG_G};
const uint8_t SEG_LINE[] = { SEG_G, SEG_G, SEG_G, SEG_G};
const uint8_t SEG_BOOT[] = { SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, SEG_F | SEG_E | SEG_G | SEG_D};
const uint8_t SEG_WIFI[] = { SEG_F | SEG_B | SEG_D, SEG_B | SEG_C,  SEG_A | SEG_F | SEG_G | SEG_E, SEG_B | SEG_C};

TM1637Display displayLeft(leftDisplayCLK, leftDisplayDIO); // Объект для левого дисплея
TM1637Display displayMiddle(midDisplayCLK, midDisplayDIO); // Объект для среднего дисплея
TM1637Display displayRight(rightDisplayCLK, rightDisplayDIO); // Объект для правого дисплея

void callback(char* topic, byte* payload, unsigned int length) // Функция, вызываемая при получении сообщения от MQTT-брокера
{
  
  StaticJsonDocument<256> doc; // Объект, которому будет присвоено значение сообщения
  DeserializationError error = deserializeJson(doc, payload, length); // Десериализация
  if (error) { displayPhrase(SEG_JSON, SEG_DATA, SEG_FAIL); } // Функция, которая будет выполнена, если произошла ошибка

  int hours = doc["System"][0]; // Часы
  int minutes = doc["System"][1]; // Минуты
  int ambientLight = doc["System"][2]; // Фоновая освещенность
  int playroomTemp = doc["Playroom"][0]; // Температура в детской
  int playroomHum = doc["Playroom"][1]; // Влажность в детской
  int outdoorTemp = doc["Outdoor"][0]; // Температура на улице
  int outdoorHum = doc["Outdoor"][1]; // Влажность на улице
  int atmPressure = doc["Outdoor"][2]; // Атмосферное давление
  int windSpeed = doc["Outdoor"][3]; // Скорость ветра
 
  displayLeft.clear(); displayMiddle.clear(); displayRight.clear(); // Очистка дисплеев

  if (ambientLight < 10) { setBrightnessAll(0); } else if (ambientLight > 20) { setBrightnessAll(7); } else { setBrightnessAll(4); } // Настройка яркости дисплеев по фоновой освещенности
  // if (hours <= 8 || hours >= 23) { setBrightnessAll(0); } else if (hours >= 12 && hours <= 16) { setBrightnessAll(7); } else { setBrightnessAll(4); } // Настройка яркости дисплеев по времени

  displayMiddle.showNumberDecEx(hours * 100 + minutes, 0b01000000, true, 4, 0); // Вывод времени на центральный дисплей

  // Показываем температуру и влажность в детской
  setRGB(0,10,0); // Зеленый цвет на светодиоде
  displayLeft.setSegments(SEG_DEGREE, 1, 3); // Знак градуса
  displayLeft.showNumberDec(playroomTemp, false, 2, 1); // Значение температуры  
  displayRight.setSegments(SEG_PERCENT, 2, 2); // Знак процента
  displayRight.showNumberDec(playroomHum, false, 2, 0); // Значение влажности
  delay(dataDisplayTime); // Ждем

  // Показываем температуру и влажность на улице
  setRGB(0,0,10); // Синий цвет на светодиоде
  displayLeft.clear(); displayRight.clear(); // Очищаем левый и правый дисплеи
  displayLeft.setSegments(SEG_DEGREE, 1, 3); // Знак градуса
  displayLeft.showNumberDec(outdoorTemp, false, 2, 1); // Значение температуры 
  displayRight.setSegments(SEG_PERCENT, 2, 2); // Знак процента
  displayRight.showNumberDec(outdoorHum, false, 2, 0); // Значение влажности
  delay(dataDisplayTime); // Ждем

  // Показываем скорость ветра и атмосферное давление
  setRGB(0,20,10); // Синий и зеленый цвета на светодиоде
  displayLeft.clear(); displayRight.clear(); // Очищаем левый и правый дисплеи
  displayLeft.setSegments(SEG_WIND, 1, 0); // Выводим значок для ветра
  if (windSpeed > 5) { displayLeft.setSegments(SEG_WIND, 1, 1); } // Если ветер больше 5 м/с, выводим еще один значок
  displayLeft.showNumberDec(windSpeed, false, 2, 2); // Значение скорости ветра
  if (atmPressure > 750) { displayRight.setSegments(SEG_PRESSURE_HIGH, 1, 0); } else if (atmPressure < 750) { displayRight.setSegments(SEG_PRESSURE_LOW, 1, 0); } // Уголок вверх или вниз перед давлением (больше или меньше 750мм)
  displayRight.showNumberDec(atmPressure, false, 3, 1); // Значение давления
  delay(dataDisplayTime); // Ждем

  // Повторяем еще раз, т.к. данные обновляются раз в минуту
  displayLeft.clear(); displayRight.clear(); // Очищаем левый и правый дисплеи
  
  // Показываем температуру и влажность в детской
  setRGB(0,10,0); // Зеленый цвет на светодиоде
  displayLeft.setSegments(SEG_DEGREE, 1, 3); // Знак градуса
  displayLeft.showNumberDec(playroomTemp, false, 2, 1); // Значение температуры  
  displayRight.setSegments(SEG_PERCENT, 2, 2); // Знак процента
  displayRight.showNumberDec(playroomHum, false, 2, 0); // Значение влажности
  delay(dataDisplayTime); // Ждем

  // Показываем температуру и влажность на улице
  setRGB(0,0,10); // Синий цвет на светодиоде
  displayLeft.clear(); displayRight.clear(); // Очищаем левый и правый дисплеи
  displayLeft.setSegments(SEG_DEGREE, 1, 3); // Знак градуса
  displayLeft.showNumberDec(outdoorTemp, false, 2, 1); // Значение температуры 
  displayRight.setSegments(SEG_PERCENT, 2, 2); // Знак процента
  displayRight.showNumberDec(outdoorHum, false, 2, 0); // Значение влажности
  delay(dataDisplayTime); // Ждем

  // Показываем скорость ветра и атмосферное давление
  setRGB(0,20,10); // Синий и зеленый цвета на светодиоде
  displayLeft.clear(); displayRight.clear(); // Очищаем левый и правый дисплеи
  displayLeft.setSegments(SEG_WIND, 1, 0); // Выводим значок для ветра
  if (windSpeed > 5) { displayLeft.setSegments(SEG_WIND, 1, 1); } // Если ветер больше 5 м/с, выводим еще один значок
  displayLeft.showNumberDec(windSpeed, false, 2, 2); // Значение скорости ветра
  if (atmPressure > 750) { displayRight.setSegments(SEG_PRESSURE_HIGH, 1, 0); } else if (atmPressure < 750) { displayRight.setSegments(SEG_PRESSURE_LOW, 1, 0); } // Уголок вверх или вниз перед давлением (больше или меньше 750мм)
  displayRight.showNumberDec(atmPressure, false, 3, 1); // Значение давления

  /*
   ВНИМАНИЕ! ДЛИТЕЛЬНОСТЬ ЦИКЛА НЕ ДОЛЖНА ПРЕВЫШАТЬ ОДНУ МИНУТУ (ПОЭТОМУ ПОСЛЕДНЕГО delay НЕТ)
   */
}

// Инициализация клиентов
WiFiClient wifiConnection; // WiFi 
PubSubClient client(mqttServer, 1883, callback, wifiConnection); // MQTT

// Инициализация
void setup() {
  setRGB(5,5,5); // Все три цвета на светодиоде
  setBrightnessAll(defaulBrightness); // Яркость дисплеев по умолчанию
  WiFi.begin(wifiNetwork, wifiPassword); // Пытаемся подключиться к WiFi
  WiFi.softAPdisconnect(true);  // Отключаем встроенную точку доступа  
  // Далее идут qункции для обновления прошивки Over-The-Air
  //ArduinoOTA.setPort(OTAUpdatePort); // Раскомментируйте, если нужно установить порт
  //ArduinoOTA.setHostname(OTAUserName); // Раскомментируйте, если нужно установить имя клиента
  //ArduinoOTA.setPassword(OTAUserPassword); // Раскомментируйте, если нужно установить пароль для обновления
  ArduinoOTA.onStart([]() { setBrightnessAll(defaulBrightness); displayPhrase(SEG_LINE, SEG_UPDATE, SEG_LINE); delay(statusDisplayTime); displayPhrase(SEG_CODE, SEG_LOAD, SEG_INIT); delay(statusDisplayTime); if (ArduinoOTA.getCommand() == U_FLASH) { displayPhrase(SEG_UPDATE, SEG_TYPE, SEG_CODE); } else { displayPhrase(SEG_UPDATE, SEG_TYPE, SEG_FILE); } delay(statusDisplayTime);  });
  ArduinoOTA.onEnd([]() { displayPhrase(SEG_CODE, SEG_LOAD, SEG_DONE); delay(statusDisplayTime); displayPhrase(SEG_CODE, SEG_SAVE, SEG_DONE); delay(statusDisplayTime*2); ESP.restart(); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { displayLeft.setSegments(SEG_LOAD, 4, 0); displayMiddle.setSegments(SEG_PERCENT, 2, 2); displayMiddle.showNumberDec((progress / (total / 100)), true, 2, 0); displayRight.setSegments(SEG_DONE, 4, 0); });
  ArduinoOTA.onError([](ota_error_t error) { if (error == OTA_AUTH_ERROR) { displayPhrase(SEG_UPDATE, SEG_AUTH, SEG_FAIL); delay(statusDisplayTime*2); } else if (error == OTA_BEGIN_ERROR) { displayPhrase(SEG_UPDATE, SEG_INIT, SEG_FAIL); delay(statusDisplayTime*2); } else if (error == OTA_CONNECT_ERROR) { displayPhrase(SEG_UPDATE, SEG_CONN, SEG_FAIL); delay(statusDisplayTime*2); } else if (error == OTA_RECEIVE_ERROR) { displayPhrase(SEG_UPDATE, SEG_LOAD, SEG_FAIL); delay(statusDisplayTime*2); } else if (error == OTA_END_ERROR) { displayPhrase(SEG_UPDATE, SEG_SAVE, SEG_FAIL); delay(statusDisplayTime*2); }});
  ArduinoOTA.begin();
}

void displayPhrase(const uint8_t* leftText, const uint8_t* middleText, const uint8_t* rightText) // Функция для вывода на все три дисплея сразу
{ displayLeft.clear(); displayMiddle.clear(); displayRight.clear(); displayLeft.setSegments(leftText, 4, 0); displayMiddle.setSegments(middleText, 4, 0); displayRight.setSegments(rightText, 4, 0); }

void setBrightnessAll(int brightness) // Функция для установки яркости для всех трех дисплеев сразу
{ displayLeft.clear(); displayMiddle.clear(); displayRight.clear(); displayLeft.setBrightness(brightness); displayMiddle.setBrightness(brightness); displayRight.setBrightness(brightness); }

void setRGB(int valueRed, int valueGreen, int valueBlue) // Функция для включения RGB-светодиода
{ analogWrite(redPin, valueRed); analogWrite(greenPin, valueGreen); analogWrite(bluePin, valueBlue); }

void connect() // Установка соединения с WiFi и MQTT-брокером
{ 
  setBrightnessAll(defaulBrightness); // Яркость дисплеев по умолчанию
  setRGB(10,0,0); // Красный цвет на светодиоде 
  displayPhrase(SEG_LINE, SEG_BOOT, SEG_LINE); // Выводим статус 
  delay(statusDisplayTime); // Ждем
  displayPhrase(SEG_NETW, SEG_CONN, SEG_INIT); // Выводим статус 
  while (WiFi.status() != WL_CONNECTED) { delay(statusDisplayTime); displayPhrase(SEG_NETW, SEG_CONN, SEG_FAIL); } // Повторяем, пока нет соединения с WiFi 
  displayPhrase(SEG_NETW, SEG_CONN, SEG_DONE); // Выводим статус 
  delay(statusDisplayTime); // Ждем
  displayPhrase(SEG_DATA, SEG_CONN, SEG_INIT); // Выводим статус 
  while (!client.connect(clientName, mqttUser, mqttPassword)) { delay(statusDisplayTime); displayPhrase(SEG_DATA, SEG_CONN, SEG_FAIL); } // Повторяем, пока нет соединения с MQTT
  displayPhrase(SEG_DATA, SEG_CONN, SEG_DONE); // Выводим статус 
  delay(statusDisplayTime); // Ждем
  displayPhrase(SEG_JSON, SEG_DATA, SEG_LOAD); // Выводим статус 
  setRGB(0,0,10); // Синий цвет на светодиоде
  client.subscribe(mqttInTopic); // Подписываемся на топик
}

// Закольцованная функция
void loop() 
{  
  ArduinoOTA.handle(); // Обработчик прошивки Over-The-Air
  if (!client.connected()) { connect(); } // Если клиент не подключен, то устанавливаем соединение с WiFi и MQTT-брокером
  client.loop();
}
