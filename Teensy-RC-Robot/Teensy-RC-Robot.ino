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

const int gimbalCenter = 1024;

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

int channels[8];

#define ch1 channels[0]
#define ch2 channels[1]
#define ch3 channels[2]
#define ch4 channels[3]
#define ch5 channels[4]
#define ch6 channels[5]
#define ch7 channels[6]
#define ch8 channels[7]

int aCounter = 0;

void setup(){
  pinMode(buzzer, OUTPUT);
  
  sBus.begin();
  Serial.begin(115200);
  roboclaw.begin(38400);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  initiallizeChannels();
  startupSound();
}

void loop(){
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    radioInitialized = true;
    //sBus outputs some weird data at the beginning. this counter ignores the first 10 packets.
    if (aCounter > 10) {
      sBus.UpdateServos();
      sBus.UpdateChannels();
      sBus.toChannels = 0;
      normalizeChannels();
    } else {
      aCounter++;
    }
  }

  drive();
  updateBatteryVoltage();
  updateIndicators();
  updateDisplay();
}

void drive() {
  if (ch2 >= 0) {
    roboclaw.ForwardM1(RC2, ch2);
  } else {
    roboclaw.BackwardM1(RC2, -ch2);
  }
}

void startupSound() {
  tone(buzzer, 698);
  delay(100);
  tone(buzzer, 932);
  delay(100);
  noTone(buzzer);
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

void normalizeChannels() {
  for (int i = 0; i < 8; i++) {
    channels[i] = sBus.channels[i];
  }
  
  //reverse channels 1 and 4
  ch1 = 2048 - ch1;
  ch4 = 2048 - ch4;

  //swap channels 3 and 4
  int temp = ch3;
  ch3 = ch4;
  ch4 = temp;

  //for channels 1 through 4 (gimbals)
  for (int i = 0; i < 4; i++) {
    //zero the center
    channels[i] -= 1024;

    //downrate to 8-bit
    channels[i] = (int)(channels[i] / 5.165);

    //apply deadband
    if (channels[i] >= -2 && channels[i] <= 2) {
      channels[i] = 0;
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
  } else {
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
  batteryVoltage = roboclaw.ReadMainBatteryVoltage(RC1, &roboclawConnected);
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
    if (currentMillis - displayChannelsPreviousMillis >= displayChannelsRefreshInterval) {
      displayChannelsPreviousMillis = currentMillis;
      displayChannels();
    }
  } else if (ch6 == switchMiddle) {
    if (currentMillis - displaySettingsPreviousMillis >= displaySettingsRefreshInterval) {
      displaySettingsPreviousMillis = currentMillis;
      displaySettings();
    }
  } else {
    if (currentMillis - displayVoltagePreviousMillis >= displayVoltageRefreshInterval) {
      displayVoltagePreviousMillis = currentMillis;
      displayVoltage();
    }
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
      display.print(channels[i + j]);
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
