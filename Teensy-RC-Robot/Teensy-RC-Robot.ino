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
unsigned long displayVoltageInterval = 1000;
unsigned long displaySettingsInterval = 33;

int displayState = 0;

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

void setup(){
  pinMode(buzzer, OUTPUT);
  
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

void initiallizeChannels() {
  sBus.channels[0] = gimbalCenter;
  sBus.channels[1] = gimbalCenter;
  sBus.channels[2] = gimbalCenter;
  sBus.channels[3] = gimbalCenter;
  sBus.channels[4] = switchMiddle;
  sBus.channels[5] = switchUp;
  sBus.channels[6] = switchMiddle;
  sBus.channels[7] = switchMiddle;
}

void readSBus() {
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    radioInitialized = true;
  }
}

void normalizeChannels() {
  for (int i = 0; i < 8; i++) {
    normalizedChannels[i] = sBus.channels[i];
  }

  for (int i = 0; i < 4; i++) {
    if (normalizedChannels[i] < gimbalLow || normalizedChannels[i] > gimbalHigh) {
      sBusOutOfRange = true;
    }
  }
  
  //reverse channels 2 and 3
  ch2 = 2046 - ch2;
  ch3 = 2046 - ch3;

  //swap channel 3 with 4
  int temp = ch3;
  ch3 = ch4;
  ch4 = temp;

  //zero the center
  ch1 = ch1 - gimbalCenter;
  ch2 = ch2 - gimbalCenter;
  ch3 = ch3 - gimbalCenter;
  ch4 = ch4 - gimbalCenter;

  //downrate to 8-bit
  double downrateConstant = 5.173228;
  ch1 = (int)(ch1 / downrateConstant);
  ch2 = (int)(ch2 / downrateConstant);
  ch3 = (int)(ch3 / downrateConstant);
  ch4 = (int)(ch4 / downrateConstant);

  applyDeadband();
}

void applyDeadband() {
  for (int i = 0; i < 4; i++) {
    if (normalizedChannels[i] < 2 && normalizedChannels[i] > -2) {
      normalizedChannels[i] = 0;
    }
  }
}

void drive() {
  if (!sBusOutOfRange) {
    if (ch2 >= 0) {
      roboclaw.ForwardM1(RC2, ch2);
    } else {
      roboclaw.BackwardM1(RC2, -ch2);
    }
  } else {
    sBusOutOfRange = false;
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
  if (getBatteryStatus() || !roboclawConnected) {
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

bool getRadioStatus() {
  if (radioInitialized && sBus.failsafe_status != 3) {
    return true;
  } else {
    return false;
  }
}

bool getBatteryStatus() {
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

void updateDisplay() {
  unsigned long currentMillis = millis();
  if (ch6 == switchUp) {
    if (roboclawConnected) {
      if ((currentMillis - displayChannelsPreviousMillis >= displayChannelsInterval) || displayState != 1) {
        displayChannelsPreviousMillis = currentMillis;
        displayChannels();
      }
    } else {
      displayChannels();
    }
    displayState = 1;
  } else if (ch6 == switchMiddle) {
    if ((currentMillis - displaySettingsPreviousMillis >= displaySettingsInterval) || displayState != 2) {
      displaySettingsPreviousMillis = currentMillis;
      displaySettings();
    }
    displayState = 2;
  } else {
    if ((currentMillis - displayVoltagePreviousMillis >= displayVoltageInterval) || displayState != 3) {
      displayVoltagePreviousMillis = currentMillis;
      displayVoltage();
    }
    displayState = 3;
  }
}

void displayChannels() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  for (int i = 0; i < 8; i = i + 2) {
    for (int j = 0; j < 2; j++) {
      display.setCursor(67 * j, 8 * i);
      display.setTextSize(1);
      display.print(i + j + 1);
      display.print(":");
      display.setTextSize(2);
      display.print(normalizedChannels[i + j]);
    }
  }
  display.display();
}

void displayVoltage() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(20, 2);
  display.print("Battery Voltage");
  display.setTextSize(5);
  if (batteryVoltage <= 100) {
    display.setCursor(22, 23);
  } else {
    display.setCursor(7, 23);
  }
  display.print(batteryVoltage / 10.0, 1);
  display.display();
}

void displaySettings() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Alarm Voltage: ");
  display.print(batteryAlarmVoltage / 10.0, 1);
  display.println("V");
  display.print("Current Voltage: ");
  display.print(batteryVoltage / 10.0, 1);
  display.println("V");
  display.print("RoboClaw Connected: ");
  display.println(roboclawConnected);
  display.print("Radio Initialized: ");
  display.println(radioInitialized);
  display.print("Failsafe Status: ");
  display.println(sBus.failsafe_status);
  display.print("sBus Out of Range: ");
  display.println(sBusOutOfRange);
  display.display();
}
