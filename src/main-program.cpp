#include <Arduino.h>
#include <OneButton.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <FS.h>
#include <AsyncTCP.h>
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

/**
 * Pin Layout
 * 
 * MRFC522 RFID READER SPI
 * VCC  --> 3.3V
 * GND  --> GND
 * SCK  --> 18
 * MISO --> 19
 * MOSI --> 23
 * SS   --> 5
 * RST  --> 27
 * 
 * Keyless Solenoid (Relay) --> 25
 * Buzzer --> 26
 * 
 * User button (Pulled Up)  --> 33
 * Power detect (Pulled Up) --> 32
 * 
*/

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

#include <array>

#define SS_PIN 5
#define RST_PIN 27

const char* TAG = "keyless rfid";

const char* apSsid = "esp32-ssid";
const char* apPass = "esp32-pass";

IPAddress apIp(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

MFRC522 cardReader(SS_PIN, RST_PIN);

OneButton userButton(33);
OneButton powerDetect(32);

AsyncWebServer server(80);

enum STATE : uint8_t {
  IDLE,
  RUNNING,
  RESET
};

int internalLed = 2;
int pinSolenoid = 25;
int pinBuzzer = 26;
int rfidReadingLed = 13;
int rfidAccessLed = 16;

uint8_t testArr1[] = {1, 2, 3};
uint8_t testArr2[] = {1, 2, 3};
uint8_t testArr3[] = {1, 2, 4};

const uint8_t nUid[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xa};

typedef std::array<std::array<uint8_t, 6>, 2> mac_reference_list;
typedef std::array<std::array<uint8_t, 10>, 3> ktp_reference_list;

mac_reference_list macList = {
  {
    {0xde, 0x37, 0xf6, 0xfd, 0x2e, 0x6b}, //xiaomi poco m4 pro
    {0xf8, 0x5e, 0xa0, 0x0a, 0xd7, 0x9b}  //asus sundaya
  }
};

ktp_reference_list ktpList = {
  {
    {0x04, 0x83, 0x47, 0x62, 0xe6, 0x5b, 0x80, 0x00, 0x00, 0x00}, //gozali e-ktp
    {0x04, 0x36, 0x0f, 0xda, 0xff, 0x24, 0x80, 0x00, 0x00, 0x00}, //thomas e-ktp
  }
};

uint8_t macTest[] = {0xde, 0x37, 0xf6, 0xfd, 0x2e, 0x6b};
uint8_t macTestDiff[] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x17};

uint8_t numOfRecognizedMac = 0;

bool isPowerReceived = false;
bool isButtonClicked = false;
bool isButtonLongClicked = false;
bool isActivatedByCard = false;
bool isMacRecognized = false;
bool isRegisteredKtpList = false;
bool isClickByWeb = false;
bool isStartByWeb = false;
bool isForceStop = false;

int solenoidSignal = 0;

uint8_t currentState = IDLE;

unsigned lastRead = 0;

void testMfrc522()
{
  if (cardReader.PICC_IsNewCardPresent()) { // new tag is available
    if (cardReader.PICC_ReadCardSerial()) { // NUID has been readed
      MFRC522::PICC_Type piccType = cardReader.PICC_GetType(cardReader.uid.sak);
      Serial.print("cardReader/NFC Tag Type: ");
      Serial.println(cardReader.PICC_GetTypeName(piccType));

      // print UID in Serial Monitor in the hex format
      Serial.print("UID:");
      for (int i = 0; i < cardReader.uid.size; i++) {
        Serial.print(cardReader.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(cardReader.uid.uidByte[i], HEX);
      }
      Serial.println();

      cardReader.PICC_HaltA(); // halt PICC
      cardReader.PCD_StopCrypto1(); // stop encryption on PCD
    }
  }
}

bool readRfid(MFRC522 &reader, MFRC522::Uid &uid)
{
  if (!reader.PICC_IsNewCardPresent())
  {
    return 0;
  }

  if (!reader.PICC_ReadCardSerial())
  {
    return 0;
  }
  
  uid.size = reader.uid.size;
  uid.sak = reader.uid.sak;
  memcpy(uid.uidByte, reader.uid.uidByte, reader.uid.size);
  ESP_LOGI(TAG, "uid : ");
  ESP_LOG_BUFFER_HEX(TAG, uid.uidByte, uid.size);
  ESP_LOGI(TAG, "size : %d\n", reader.uid.size);
  return 1;
}

bool isInMacList(const mac_reference_list &macList, const uint8_t* macToCheck)
{
  for (size_t i = 0; i < macList.size(); i++)
  {
    bool isEqual = false;
    for (size_t j = 0; j < macList[i].size(); j++)
    {
      if (macToCheck[j] == macList[i][j])
      {
        isEqual = true;
      }
      else {
        isEqual = false;
        break;
      }
    }
    if (isEqual)
    {
      return 1;
    }
  }
  return 0;
}

bool isInKtpList(const ktp_reference_list &ktpList, const MFRC522::Uid &uidToCheck)
{
  for (size_t i = 0; i < ktpList.size(); i++)
  {
    bool isEqual = false;
    for (size_t j = 0; j < uidToCheck.size; j++)
    {
      if (uidToCheck.uidByte[j] == ktpList[i][j])
      {
        isEqual = true;
      }
      else {
        isEqual = false;
        break;
      }
    }
    if (isEqual)
    {
      return 1;
    }
  }
  return 0;
}

bool compareArray(const uint8_t *arr1, const uint8_t *arr2, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    if (arr1[i] != arr2[i])
    {
      return 0;
    }
  }
  return 1;
}

void apClientConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  ESP_LOGI(TAG, "client connected");
  ESP_LOG_BUFFER_HEXDUMP(TAG, info.wifi_ap_staconnected.mac, 6, ESP_LOG_INFO);
  
  if (isInMacList(macList, info.wifi_ap_staconnected.mac))
  {
    ESP_LOGI(TAG, "mac recognized");
    numOfRecognizedMac++;
  }
}

void apClientDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  ESP_LOGI(TAG, "client disconnected");
  ESP_LOG_BUFFER_HEXDUMP(TAG, info.wifi_ap_stadisconnected.mac, 6, ESP_LOG_INFO);
  if (isInMacList(macList, info.wifi_ap_staconnected.mac))
  {
    ESP_LOGI(TAG, "mac recognized");
    numOfRecognizedMac--;
  }
}

void onClick()
{
  isButtonClicked = true;
  ESP_LOGI(TAG, "button clicked");
}

void onLongClickPress()
{
  isButtonLongClicked = true;
  ESP_LOGI(TAG, "button long click pressed");
}

void onLongClickRelease()
{
  isButtonLongClicked = false;
  ESP_LOGI(TAG, "button long click release");
}

void powerDetectOnLongPressStart()
{
  isPowerReceived = true;
  ESP_LOGI(TAG, "power on");
}

void powerDetectOnLongPressStop()
{
  isPowerReceived = false;
  ESP_LOGI(TAG, "power off");
}

String buildJsonResponse(int code)
{
    JsonDocument doc;
    String response;
    doc["status"] = code;
    serializeJson(doc, response);
    return response;
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  esp_log_level_set(TAG, ESP_LOG_INFO);
  pinMode(pinSolenoid, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(internalLed, OUTPUT);
  pinMode(rfidReadingLed, OUTPUT);
  pinMode(rfidAccessLed, OUTPUT);
  digitalWrite(pinSolenoid, HIGH);
  digitalWrite(pinBuzzer, HIGH);
  digitalWrite(rfidReadingLed, LOW);
  digitalWrite(rfidAccessLed, LOW);
  SPI.begin(SCK, MISO, MOSI, SS_PIN);
  cardReader.PCD_Init();
  cardReader.PCD_DumpVersionToSerial();
  userButton.setDebounceMs(20);
  userButton.setClickMs(50);
  userButton.setPressMs(500);
  userButton.attachClick(onClick);
  userButton.attachLongPressStart(onLongClickPress);
  userButton.attachLongPressStop(onLongClickRelease);
  powerDetect.setDebounceMs(20);
  powerDetect.setClickMs(50);
  powerDetect.setPressMs(200);
  powerDetect.attachLongPressStart(powerDetectOnLongPressStart);
  powerDetect.attachLongPressStop(powerDetectOnLongPressStop);
  WiFi.onEvent(apClientConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
  WiFi.onEvent(apClientDisconnected, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAPConfig(apIp, apGateway, apSubnet);
  WiFi.softAP(apSsid, apPass);
  
  server.on("/api/get-data", HTTP_GET, [] (AsyncWebServerRequest *request) {
    JsonDocument doc;

    doc["key_status"] = (int)isPowerReceived; //active_low, but the flag already processed on callback
    doc["connected_device"] = WiFi.softAPgetStationNum();
    doc["recognized_device"] = numOfRecognizedMac;
    doc["solenoid_status"] = (int)!solenoidSignal; //pin is active_low, turn off meaning zero, need to invert it
    doc["force_stop"] = (int)isForceStop;

    String output;

    serializeJson(doc, output);
    request->send(200, "application/json", output);
  });

  AsyncCallbackJsonWebHandler *clickHandler = new AsyncCallbackJsonWebHandler("/api/click", [](AsyncWebServerRequest *request, JsonVariant &json)
  {
    int status = 400;
    if (!json.containsKey("click"))
    {
      request->send(status, "application/json", buildJsonResponse(status));
      return;
    }

    JsonVariant click = json["click"];
    isClickByWeb = click.as<bool>();
    status = 200;
    request->send(status, "application/json", buildJsonResponse(status));
  } );

  AsyncCallbackJsonWebHandler *startHandler = new AsyncCallbackJsonWebHandler("/api/start", [](AsyncWebServerRequest *request, JsonVariant &json)
  {
    int status = 400;
    if (!json.containsKey("start"))
    {
      request->send(status, "application/json", buildJsonResponse(status));
      return;
    }

    JsonVariant start = json["start"];
    isStartByWeb = start.as<bool>();
    status = 200;
    request->send(status, "application/json", buildJsonResponse(status));
  } );

  AsyncCallbackJsonWebHandler *forceStopHandler = new AsyncCallbackJsonWebHandler("/api/force-stop", [](AsyncWebServerRequest *request, JsonVariant &json)
  {
    int status = 400;
    if (!json.containsKey("force_stop"))
    {
      request->send(status, "application/json", buildJsonResponse(status));
      return;
    }

    JsonVariant forceStop = json["force_stop"];
    isForceStop = forceStop.as<bool>();
    status = 200;
    request->send(status, "application/json", buildJsonResponse(status));
  } );

  server.addHandler(clickHandler);
  server.addHandler(startHandler);
  server.addHandler(forceStopHandler);
  server.begin();

  ESP_LOGI(TAG, "START EPS32");
  
  if (compareArray(testArr1, testArr2, 3))
  {
    ESP_LOGI(TAG, "equal array");
  }

  if (!compareArray(testArr1, testArr3, 3))
  {
    ESP_LOGI(TAG, "different array");
  }

  if (isInMacList(macList, macTest))
  {
    ESP_LOGI(TAG, "mac match");
  }

  if (!isInMacList(macList, macTestDiff))
  {
    ESP_LOGI(TAG, "mac did not match");
  }

  lastRead = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  solenoidSignal = digitalRead(pinSolenoid);
  MFRC522::Uid uid;
  if (readRfid(cardReader, uid))
  {
    ESP_LOGI(TAG, "rfid was read");
    digitalWrite(rfidReadingLed, !digitalRead(rfidReadingLed));
    isRegisteredKtpList = false;
    if(isInKtpList(ktpList, uid) )
    {
      ESP_LOGI(TAG, "is in list");
      isRegisteredKtpList = true;
      lastRead = millis();
    }
  }
  else
  {
    digitalWrite(rfidReadingLed, LOW);
    if (isRegisteredKtpList)
    {
      if (millis() - lastRead > 1000)
      {
        isActivatedByCard = !isActivatedByCard;
        isRegisteredKtpList = false;
        lastRead = millis();
      }
    }
  }

  // if (WiFi.softAPgetStationNum() || isActivatedByCard)
  if ((numOfRecognizedMac || isActivatedByCard) && !isForceStop)
  {
    digitalWrite(internalLed, HIGH);
    digitalWrite(rfidAccessLed, HIGH);
    userButton.tick();
    powerDetect.tick();
    if (currentState != RUNNING)
    {
      if (isPowerReceived)
      {
        digitalWrite(pinSolenoid, LOW);
        currentState = RUNNING;
      }
    }

    switch (currentState)
    {
    case IDLE:
      ESP_LOGI(TAG, "idle state");
      if (isButtonClicked || isClickByWeb)
      {
        digitalWrite(pinSolenoid, !solenoidSignal);
        isButtonClicked = false;
        isClickByWeb = false;
      }
      if (isPowerReceived)
      {
        currentState = RUNNING;
      }
      isStartByWeb = false;
      break;
    case RUNNING:
      ESP_LOGI(TAG, "running state");
      if (isButtonClicked || isClickByWeb)
      {
        isButtonClicked = false;
        isClickByWeb = false;
      }

      if (isButtonLongClicked || isStartByWeb)
      {
        digitalWrite(pinBuzzer, LOW);
        isStartByWeb = false;
      }
      if (!isPowerReceived)
      {
        currentState = RESET;
      }
      break;
    case RESET:
      ESP_LOGI(TAG, "reset state");
      if (isButtonClicked || isClickByWeb)
      {
        isButtonClicked = false;
        isClickByWeb = false;
      }
      digitalWrite(pinBuzzer, HIGH);
      isActivatedByCard = false;
      currentState = IDLE;
      break;

    default:
      break;
    }
  }
  else
  {
    digitalWrite(pinBuzzer, HIGH);
    digitalWrite(pinSolenoid, HIGH);
    digitalWrite(internalLed, LOW);
    digitalWrite(rfidAccessLed, LOW);
    if(isButtonClicked || isClickByWeb)
    {
      isButtonClicked = false;
      isClickByWeb = false;
    }
    currentState = IDLE;
  }
  delay(10);
}
