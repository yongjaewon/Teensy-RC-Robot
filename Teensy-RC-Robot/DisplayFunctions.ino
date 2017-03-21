void updateDisplay() {
  if (digitalRead(buttonIn) == LOW) {
    cycleDisplay();
  } else if (radioStatus()) {
    radioControlDisplay();
  } else {
    refreshCurrentDisplay();
  }
}

void refreshCurrentDisplay() {
  if (displayState == 1) {
    displayChannels();
  } else if (displayState == 2) {
    displaySettings();
  } else if (displayState == 3) {
    displayVoltage();
  }
}

void cycleDisplay() {
  unsigned long currentMillis = millis();
  if (currentMillis - cycleDisplayPreviousMillis >= cycleDisplayInterval) {
    cycleDisplayPreviousMillis = currentMillis;
    if (displayState == 1) {
      displaySettings();
    } else if (displayState == 2) {
      displayVoltage();
    } else {
      displayChannels();
    }
  } else {
   refreshCurrentDisplay();
  }
}

void radioControlDisplay() {
  if (ch6 == switchUp) {
    displayChannels();
  } else if (ch6 == switchMiddle) {
    displaySettings();
  } else {
    displayVoltage();
  }
}

void displayChannels() {
  unsigned long currentMillis = millis();
  if (roboclawConnected) {
    if ((currentMillis - displayChannelsPreviousMillis >= displayChannelsInterval) || displayState != 1) {
      displayChannelsPreviousMillis = currentMillis;
      printChannels();
    }
  } else {
    printChannels();
  }
  displayState = 1;
}

void displaySettings() {
  unsigned long currentMillis = millis();
  if ((currentMillis - displaySettingsPreviousMillis >= displaySettingsInterval) || displayState != 2) {
    displaySettingsPreviousMillis = currentMillis;
    printSettings();
  }
  displayState = 2;
}

void displayVoltage() {
  unsigned long currentMillis = millis();
  if ((currentMillis - displayVoltagePreviousMillis >= displayVoltageInterval) || displayState != 3) {
    displayVoltagePreviousMillis = currentMillis;
    printVoltage();
  }
  displayState = 3;
}

void printChannels() {
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

void printVoltage() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(20, 2);
  display.print("Battery Voltage");
  display.setTextSize(5);
  if (roboclawConnected) {
    if (batteryVoltage < 100) {
      display.setCursor(22, 23);
    } else {
      display.setCursor(7, 23);
    }
    display.print(batteryVoltage / 10.0, 1);
  } else {
    display.setCursor(22, 23);
    display.print("USB");
  }
  display.display();
}

void printSettings() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  
  display.setCursor(0, 0);
  display.print("VAlrm ");
  if (batteryAlarmVoltage < 100) {
    display.print(" ");
  }
  display.print(batteryAlarmVoltage / 10.0, 1);
  
  display.setCursor(67, 0);
  display.print("VBatt ");
  if (batteryVoltage < 100) {
    display.print(" ");
  }
  display.print(batteryVoltage / 10.0, 1);

  display.setCursor(0, 9);
  display.print("RoboClaw ");
  display.print(roboclawConnected);

  display.setCursor(67, 9);
  display.print("sBusOOR  ");
  display.print(sBusOutOfRange);

  display.setCursor(0, 18);
  display.print("RdioInit ");
  display.print(radioInitialized);

  display.setCursor(67, 18);
  display.print("Failsafe ");
  display.print(sBus.failsafe_status);

  display.setCursor(0, 27);
  display.print(driveValue);
  
  display.display();
}
