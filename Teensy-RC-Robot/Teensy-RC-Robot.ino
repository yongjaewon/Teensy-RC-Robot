#include <FUTABA_SBUS.h>
#include <Streaming.h>
#include <RoboClaw.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int teensyLED = 13;

const int buzzer = 12;
unsigned long buzzerPreviousMillis = 0;
unsigned long buzzerInterval = 200;
bool buzzerOn = false;

const int redPin = 21;
const int greenPin = 23;
const int bluePin = 22;
unsigned long rgbPreviousMillis = 0;
bool rgbOn = false;

unsigned long batteryAlarmPreviousMillis = 0;
unsigned long batteryResetPreviousMillis = 0;
unsigned long batteryStatusDelay = 5000;
const int batteryAlarmVoltage = 66;

unsigned long displayChannelsPreviousMillis = 0;
unsigned long displayVoltagePreviousMillis = 0;
unsigned long displaySettingsPreviousMillis = 0;

unsigned long displayChannelsRefreshInterval = 50;
unsigned long displayVoltageRefreshInterval = 1000;
unsigned long displaySettingsRefreshInterval = 50;

bool radioInitialized = false;
bool roboclawConnected = false;

int batteryVoltage;

const int failsafeChannels[] = {1023, 1023, 1023, 1023, 1024, 1904, 1024, 1024};

FUTABA_SBUS sBus;

// hardware SPI on Teensy 3.x: MOSI pin 11, SCK pin 13 plus the pins defined below
#define OLED_DC     9
#define OLED_CS     10
#define OLED_RESET  14
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

RoboClaw roboclaw(&Serial3, 10000);
#define RC1 0x80
#define RC2 0x81

void setup(){
  pinMode(teensyLED, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  sBus.begin();
  Serial.begin(115200);
  roboclaw.begin(38400);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  failSafe();
  drive();
  delay(500);
}

void loop(){
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    radioInitialized = true;
    sBus.UpdateServos();
    sBus.UpdateChannels();
    sBus.toChannels = 0;
  }

  if (!radioStatus()) {
    failSafe();
  }
  
  drive();
  updateIndicators();
  updateDisplay();
}

void updateDisplay() {
  unsigned long currentMillis = millis();
  if (sBus.channels[5] == 144) { //display 1
    if (currentMillis - displayChannelsPreviousMillis >= displayChannelsRefreshInterval) {
      displayChannelsPreviousMillis = currentMillis;
      displayChannels();
    }
  } else if (sBus.channels[5] == 1904) { //display 2
    if (currentMillis - displayVoltagePreviousMillis >= displayVoltageRefreshInterval) {
      displayVoltagePreviousMillis = currentMillis;
      displayVoltage();
    }
  } else {
    if (currentMillis - displaySettingsPreviousMillis >= displaySettingsRefreshInterval) {
      displaySettingsPreviousMillis = currentMillis;
      displaySettings();
    }
  }
}

void failSafe() {
  for (int i = 0; i < 8; i++) {
    sBus.channels[i] = failsafeChannels[i];
  }
}

void buzz(bool condition) {
  if (condition) {
    unsigned long currentMillis = millis();
    if (currentMillis - buzzerPreviousMillis >= buzzerInterval) {
      buzzerPreviousMillis = currentMillis;
      if (buzzerOn) {
        noTone(buzzer);
        buzzerOn = false;
      } else {
        tone(buzzer, 1260);
        buzzerOn = true;
      }
    }
  } else if (sBus.channels[7] == 144){
    unsigned long currentMillis = millis();
    if (currentMillis - buzzerPreviousMillis >= buzzerInterval) {
      buzzerPreviousMillis = currentMillis;
      if (buzzerOn) {
        noTone(buzzer);
        buzzerOn = false;
      } else {
        tone(buzzer, sBus.channels[6]);
        buzzerOn = true;
      }
    }
  } else {
    noTone(buzzer);
  }
}

void updateIndicators() {
  if (batteryStatus()) {
    buzz(false);
    if (radioStatus()) {
      setRGB(0, 255, 0, 0);
    } else {
      setRGB(0, 255, 0, 500);
    }
  } else if (!roboclawConnected) {
    buzz(false);
    if (radioStatus()) {
      setRGB(40, 255, 0, 0);
    } else {
      setRGB(40, 255, 0, 500);
    }
  } else {
    buzz(true);
    if (radioStatus()) {
      setRGB(255, 0, 0, 0);
    } else {
      setRGB(255, 0, 0, 500);
    }
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
  batteryVoltage = roboclaw.ReadMainBatteryVoltage(RC1, &roboclawConnected);
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

void drive() {
  int drive = (int)((sBus.channels[1] - 1023) / 5.17);
  if (drive >= 0) {
    roboclaw.ForwardM1(RC2, drive);
  } else {
    roboclaw.BackwardM1(RC2, -drive);
  }
}

void setRGB(int red, int green, int blue, unsigned long blinkInterval)
{
  unsigned long currentMillis = millis();
  
  if (blinkInterval == 0) {
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);
  } else if (currentMillis - rgbPreviousMillis >= blinkInterval) {
    rgbPreviousMillis = currentMillis;
    if (rgbOn) {
      analogWrite(redPin, 0);
      analogWrite(greenPin, 0);
      analogWrite(bluePin, 0);
      rgbOn = false;
    } else {
      analogWrite(redPin, red);
      analogWrite(greenPin, green);
      analogWrite(bluePin, blue);
      rgbOn = true;
    }
  }
}

void displayChannels() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  for (int i = 0; i < 8; i = i + 2) {
    display.setCursor(0, 8 * i);
    display.setTextSize(1);
    display.print(i + 1);
    display.print(":");
    display.setTextSize(2);
    display.print(sBus.channels[i]);
    display.setCursor(67, 8 * i);
    display.setTextSize(1);
    display.print(i + 2);
    display.print(":");
    display.setTextSize(2);
    display.print(sBus.channels[i + 1]);
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
  display.print("Battery Alarm: ");
  display.print(batteryAlarmVoltage / 10.0, 1);
  display.println("V");
  display.print("Radio Initialized: ");
  display.println(radioInitialized);
  display.print("RoboClaw Connected: ");
  display.println(roboclawConnected);
  display.print("Failsafe Status: ");
  display.println(sBus.failsafe_status);
  for (int i = 0; i < 8; i = i + 2) {
    display.setCursor(0, 33 + 4 * i);    display.print("Ch");
    display.print(i + 1);
    display.print(": ");
    display.print(failsafeChannels[i]);
    display.setCursor(64, 33 + 4 * i);
    display.print("Ch");
    display.print(i + 2);
    display.print(": ");
    display.print(failsafeChannels[i + 1]);
  }
  display.display();
}
