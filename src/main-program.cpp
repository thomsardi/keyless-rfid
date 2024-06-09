#include <Arduino.h>
#include <OneButton.h>
#include <MFRC522.h>
#include <WiFi.h>

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

enum STATE : uint8_t {
  IDLE,
  RUNNING,
  RESET
};

int internalLed = 2;
int pinSolenoid = 25;
int pinBuzzer = 26;

uint8_t testArr1[] = {1, 2, 3};
uint8_t testArr2[] = {1, 2, 3};
uint8_t testArr3[] = {1, 2, 4};

const uint8_t nUid[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xa};

typedef std::array<std::array<uint8_t, 6>, 2> mac_reference_list;

mac_reference_list macList = {
  {
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06},
    {0x11, 0x12, 0x13, 0x14, 0x15, 0x16}
  }
};

uint8_t macTest[] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};
uint8_t macTestDiff[] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x17};

uint8_t numOfRecognizedMac = 0;

bool isPowerReceived = false;
bool isButtonClicked = false;
bool isButtonLongClicked = false;
bool isActivatedByCard = false;
bool isMacRecognized = false;
uint8_t currentState = IDLE;

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
    numOfRecognizedMac++;
  }
}

void apClientDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  ESP_LOGI(TAG, "client disconnected");
  ESP_LOG_BUFFER_HEXDUMP(TAG, info.wifi_ap_stadisconnected.mac, 6, ESP_LOG_INFO);
  if (isInMacList(macList, info.wifi_ap_staconnected.mac))
  {
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  esp_log_level_set(TAG, ESP_LOG_INFO);
  pinMode(pinSolenoid, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(internalLed, OUTPUT);
  digitalWrite(pinSolenoid, HIGH);
  digitalWrite(pinBuzzer, HIGH);
  SPI.begin(SCK, MISO, MOSI, SS_PIN);
  cardReader.PCD_Init();
  cardReader.PCD_DumpVersionToSerial();
  userButton.setDebounceMs(20);
  userButton.setClickMs(50);
  userButton.setPressMs(200);
  userButton.attachClick(onClick);
  userButton.attachLongPressStart(onLongClickPress);
  userButton.attachLongPressStop(onLongClickRelease);
  powerDetect.setDebounceMs(20);
  powerDetect.setClickMs(50);
  powerDetect.setPressMs(200);
  powerDetect.attachLongPressStart(powerDetectOnLongPressStart);
  powerDetect.attachLongPressStop(powerDetectOnLongPressStop);
  WiFi.softAPConfig(apIp, apGateway, apSubnet);
  WiFi.softAP(apSsid, apPass);
  WiFi.onEvent(apClientConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
  WiFi.onEvent(apClientDisconnected, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
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

}

void loop() {
  // put your main code here, to run repeatedly:
  MFRC522::Uid uid;
  if (readRfid(cardReader, uid))
  {
    ESP_LOGI(TAG, "rfid was read");
    isActivatedByCard = false;
    if (compareArray(uid.uidByte, nUid, uid.size))
    {
      isActivatedByCard = true;
    }
  }
  else
  {
    ESP_LOGI(TAG, "no card");
  }

  // if (WiFi.softAPgetStationNum() || isActivatedByCard)
  if (numOfRecognizedMac || isActivatedByCard)
  {
    digitalWrite(internalLed, HIGH);
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
      if (isButtonClicked)
      {
        digitalWrite(pinSolenoid, !digitalRead(pinSolenoid));
        isButtonClicked = false;
      }
      if (isPowerReceived)
      {
        currentState = RUNNING;
      }
      break;
    case RUNNING:
      ESP_LOGI(TAG, "running state");
      if (isButtonClicked)
      {
        isButtonClicked = false;
      }

      if (isButtonLongClicked)
      {
        digitalWrite(pinBuzzer, LOW);
      }
      if (!isPowerReceived)
      {
        currentState = RESET;
      }
      break;
    case RESET:
      ESP_LOGI(TAG, "reset state");
      if (isButtonClicked)
      {
        isButtonClicked = false;
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
    if(isButtonClicked)
    {
      isButtonClicked = false;
    }
    currentState = IDLE;
  }
}
