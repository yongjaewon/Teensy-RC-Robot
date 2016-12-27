#include <FUTABA_SBUS.h>
#include <Streaming.h>
#include <RoboClaw.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int buzzer = 12;
unsigned long buzzerPreviousMillis = 0;
bool buzzerOn = false;

const int buttonIn = 6;

const int switchUp = 1904;
const int switchMiddle = 1024;
const int switchDown = 144;

const int gimbalHigh = 1680;
const int gimbalCenter = 1023;
const int gimbalLow = 366;

unsigned long batteryAlarmPreviousMillis = 0;
unsigned long batteryResetPreviousMillis = 0;
unsigned long batteryStatusDelay = 5000;
const int batteryAlarmVoltage = 66;

unsigned long updateBatteryVoltagePreviousMillis = 0;
unsigned long updateBatteryVoltageInterval = 500;
int batteryVoltage;

unsigned long displayChannelsPreviousMillis = 0;
unsigned long displayVoltagePreviousMillis = 0;
unsigned long displaySettingsPreviousMillis = 0;

unsigned long displayChannelsInterval = 33;
unsigned long displayVoltageInterval = 500;
unsigned long displaySettingsInterval = 33;

unsigned long cycleDisplayPreviousMillis = 0;
unsigned long cycleDisplayInterval = 1000;

int displayState = 1; //1: channels, 2: settings, 3: voltage, every other number: voltage

bool radioInitialized = false;
bool roboclawConnected = false;

bool sBusOutOfRange = false;

unsigned long startupMillis;
int startupSoundState = 0;

FUTABA_SBUS sBus;

int normalizedChannels[8];

#define ch1 normalizedChannels[0]
#define ch2 normalizedChannels[1]
#define ch3 normalizedChannels[2]
#define ch4 normalizedChannels[3]
#define ch5 normalizedChannels[4]
#define ch6 normalizedChannels[5]
#define ch7 normalizedChannels[6]
#define ch8 normalizedChannels[7]

// hardware SPI on Teensy 3.x: MOSI pin 11, SCK pin 13 plus the pins defined below
#define OLED_DC     9
#define OLED_CS     10
#define OLED_RESET  14
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

RoboClaw roboclaw(&Serial3, 10000);
#define RC1 0x80
#define RC2 0x81

int driveValue;

void setup(){
  pinMode(buzzer, OUTPUT);
  pinMode(buttonIn, INPUT_PULLUP);
  
  sBus.begin();
  Serial.begin(115200);
  roboclaw.begin(38400);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display.setRotation(2);
  initiallizeChannels();
  startupMillis = millis();
}

void loop(){
    startupSound();
    readSBus();
    normalizeChannels();
    drive();
    updateBatteryVoltage();
    updateIndicators();
    updateDisplay();
}

void readSBus() {
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    radioInitialized = true;
  }
}

void drive() {
  if (!sBusOutOfRange) {
    driveValue = round(ch2 * ch3 / 255);
    if (ch2 >= 0) {
      roboclaw.ForwardM1(RC2, driveValue);
    } else {
      roboclaw.BackwardM1(RC2, -driveValue);
    }
  } else {
    stopAllMotors();
  }
}

void stopAllMotors() {
  roboclaw.ForwardM1(RC1, 0);
  roboclaw.ForwardM2(RC1, 0);
  roboclaw.ForwardM1(RC2, 0);
  roboclaw.ForwardM2(RC2, 0);
}

void startupSound() {
  if (startupSoundState != 3) {
    unsigned long millisSinceStartup = millis() - startupMillis;
    if (millisSinceStartup < 100 && startupSoundState != 1) {
      tone(buzzer, 698);
      startupSoundState = 1;
    } else if (millisSinceStartup >= 100 && millisSinceStartup < 200 && startupSoundState != 2) {
      tone(buzzer, 932);
      startupSoundState = 2;
    } else if (millisSinceStartup >= 200) {
      noTone(buzzer);
      startupSoundState = 3;
    }
  }
}

void setBuzzer(int buzzerFrequency, unsigned long buzzerInterval) {
  if (buzzerFrequency > 0) {
    if (buzzerInterval == 0) {
      tone(buzzer, buzzerFrequency);
    } else {
      unsigned long currentMillis = millis();
      if (currentMillis - buzzerPreviousMillis >= buzzerInterval) {
        buzzerPreviousMillis = currentMillis;
        if (buzzerOn) {
          noTone(buzzer);
          buzzerOn = false;
        } else {
          tone(buzzer, buzzerFrequency);
          buzzerOn = true;
        }
      }
    }
  } else if (startupSoundState == 3) {
    noTone(buzzer);
  }
}

void updateIndicators() {
  if (batteryStatus() || !roboclawConnected) {
    setBuzzer(0, 0);
  } else {
    setBuzzer(1260, 200);
  }
}

void updateBatteryVoltage() {
  unsigned long currentMillis = millis();
  if (currentMillis - updateBatteryVoltagePreviousMillis >= updateBatteryVoltageInterval) {
    updateBatteryVoltagePreviousMillis = currentMillis;
    batteryVoltage = roboclaw.ReadMainBatteryVoltage(RC1, &roboclawConnected);
  }
}

bool radioStatus() {
  if (radioInitialized && sBus.failsafe_status != 3) {
    return true;
  } else {
    return false;
  }
}

bool batteryStatus() {
  unsigned long currentMillis = millis();
  if (!roboclawConnected) {
    return false;
  } else if (batteryVoltage <= batteryAlarmVoltage) {
    if (currentMillis - batteryAlarmPreviousMillis >= batteryStatusDelay) {
      batteryResetPreviousMillis = currentMillis;
      return false;
    } else {
      return true;
    }
  } else {
    if (currentMillis < batteryStatusDelay || currentMillis - batteryResetPreviousMillis >= batteryStatusDelay) {
      batteryAlarmPreviousMillis = currentMillis;
      return true;
    } else {
      return false;
    }
  }
}
