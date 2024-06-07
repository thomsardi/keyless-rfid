#include <Arduino.h>
#include <OneButton.h>
#include <WiFi.h>

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

const char* TAG = "keyless rfid";

const char* apSsid = "esp32-ssid";
const char* apPass = "esp32-pass";

IPAddress apIp(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

OneButton userButton(33);
OneButton powerDetect(32);

enum STATE : uint8_t {
  IDLE,
  RUNNING,
  RESET
};

int pinSolenoid = 27;
int pinBuzzer = 26;

bool isPowerReceived = false;
bool isButtonClicked = false;
bool isButtonLongClicked = false;
uint8_t currentState = IDLE;

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
  ESP_LOGI(TAG, "START EPS32");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (WiFi.softAPgetStationNum())
  {
    userButton.tick();
    powerDetect.tick();
    if (currentState != RUNNING)
    {
      if (isPowerReceived)
      {
        digitalWrite(pinSolenoid, HIGH);
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
        digitalWrite(pinBuzzer, HIGH);
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
      digitalWrite(pinBuzzer, LOW);
      currentState = IDLE;
      break;

    default:
      break;
    }
  }
  else
  {
    digitalWrite(pinBuzzer, LOW);
    digitalWrite(pinSolenoid, LOW);
    if(isButtonClicked)
    {
      isButtonClicked = false;
    }
    currentState = IDLE;
  }
}
