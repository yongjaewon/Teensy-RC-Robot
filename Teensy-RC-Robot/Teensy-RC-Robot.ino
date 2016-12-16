#include <FUTABA_SBUS.h>
#include <Streaming.h>
#include <RoboClaw.h>

const int teensyLED = 13;

const int redPin = 20;
const int greenPin = 22;
const int bluePin = 21;

const int buzzer = 23;

unsigned long previousMillis = 0;
bool rgbOn = false;

bool radioInitialized = false;
bool radioConnected = false;

int batteryVoltage;
bool roboclawConnected = false;
const int minCellVoltage = 33;
const int numOfCells = 2;

FUTABA_SBUS sBus;

RoboClaw roboclaw(&Serial2,10000);
#define address 0x80

void setup(){
  pinMode(teensyLED, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  sBus.begin();
  Serial.begin(115200);
  roboclaw.begin(38400);
}

void loop(){
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    radioInitialized = true;
    sBus.UpdateServos();
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    drive();
  }
  
  printAllsBusStatus();
  updateRGB();
}

void failSafe() {
  roboclaw.ForwardM1(address, 0);
}

void updateRGB() {
  radioConnected = radioStatus();
  if (!batteryStatus()) {
    noTone(buzzer);
    if (radioConnected && roboclawConnected) {
      setRGB(0, 255, 0, 0);
    } else if (radioConnected && !roboclawConnected) {
      setRGB(255, 255, 0, 0); 
    } else if (!radioConnected && roboclawConnected) {
      setRGB(0, 255, 0, 500);
    } else {
      setRGB(255, 255, 0, 500);
    }
  }
  else {
    //tone(buzzer, 440);
    if (radioConnected) {
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
  batteryVoltage = roboclaw.ReadMainBatteryVoltage(address, &roboclawConnected);
  if (!roboclawConnected || batteryVoltage > (minCellVoltage * numOfCells)) {
    return true;
  } else {
    return false;
  }
}

void drive() {
  if (!radioConnected) {
    roboclaw.ForwardM1(address, 0);
  } else {
    int drive = (int)((sBus.channels[1] - 1023) / 5.17);
    if (drive >= 0) {
      roboclaw.ForwardM1(address, drive);
    } else {
      roboclaw.BackwardM1(address, -drive);
    }
  }
}

void setRGB(int red, int green, int blue, unsigned long blinkInterval)
{
  unsigned long currentMillis = millis();
  
  if (blinkInterval == 0) {
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);
  } else if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;
    if (rgbOn == false) {
      analogWrite(redPin, red);
      analogWrite(greenPin, green);
      analogWrite(bluePin, blue);
      rgbOn = true;
    } else {
      analogWrite(redPin, 0);
      analogWrite(greenPin, 0);
      analogWrite(bluePin, 0);
      rgbOn = false;
    }
  }
}

void printAllsBusStatus() {
  uint8_t i;
    for (i=0; i<8; i++) {
      Serial.printf("Ch %d : %d \n", i + 1, sBus.channels[i]);
    }
  Serial.print("RoboClaw Status: ");
  if (roboclawConnected) {
    Serial.println("Connected");
    Serial.printf("Battery Voltage: %.1f V \n", batteryVoltage / 10.0);
  } else {
    Serial.println("Disconnected");
    Serial.println("Battery Voltage: Not Available");
  }
  if (radioConnected) {
    Serial.println("Radio Status: Connected \n\n");
  } else {
    Serial.println("Radio Status: Disconnected \n\n");
  }
}
