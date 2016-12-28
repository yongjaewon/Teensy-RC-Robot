void initiallizeChannels() {
  sBus.channels[0] = gimbalCenter;
  sBus.channels[1] = gimbalCenter;
  sBus.channels[2] = gimbalCenter;
  sBus.channels[3] = gimbalCenter;
  sBus.channels[4] = switchMiddle;
  sBus.channels[5] = switchMiddle;
  sBus.channels[6] = switchMiddle;
  sBus.channels[7] = switchMiddle;
}

void normalizeChannels() {
  sBusOutOfRange = false;
  for (int i = 0; i < 4; i++) {
    //142 and 1904 are the extremes reachable by gimbals using trims.
    //if not in range 142-1904, data is erratic.
    if (sBus.channels[i] < 142 || normalizedChannels[i] > 1904) {
      sBusOutOfRange = true;
    }
  }

  //copy channels data onto normalizedChannels[]
  for (int i = 0; i < 8; i++) {
    normalizedChannels[i] = sBus.channels[i];
  }
  
  //reverse channels 2 and 3
  ch2 = 2046 - ch2;
  ch3 = 2046 - ch3;

  //zero the center
  ch1 = ch1 - gimbalCenter;
  ch2 = ch2 - gimbalCenter;
  ch4 = ch4 - gimbalCenter;

  //zero the low position
  ch3 -= gimbalLow;
  
  //downrate to 8-bit (-127 to 127 : not exactly. real-8bit is -128 to 127)
  ch1 = round(ch1 / 5.156862745);
  ch2 = round(ch2 / 5.156862745);
  ch4 = round(ch4 / 5.156862745);

  //downrate to 7-bit (0 to 127)
  ch3 = round(ch3 / 10.35433071);

  applyDeadbands();
  applyEndPoints();
}

void applyDeadbands() {
  int deadbandValue = 4;
  if (ch1 <= deadbandValue && ch1 >= -deadbandValue) {
    ch1 = 0;
  }
  
  if (ch2 <= deadbandValue && ch2 >= -deadbandValue) {
    ch2 = 0;
  }
  
  if (ch3 <= deadbandValue && ch3 >= -deadbandValue) {
    ch3 = 0;
  }

  if (ch4 <= deadbandValue && ch4 >= -deadbandValue) {
    ch4 = 0;
  }
}

void applyEndPoints() {
  if (ch1 < -127) {
    ch1 = -127;
  } else if (ch1 > 127) {
    ch1 = 127;
  }

  if (ch2 < -127) {
    ch2 = -127;
  } else if (ch2 > 127) {
    ch2 = 127;
  }

  if (ch3 < 0) {
    ch3 = 0;
  } else if (ch3 > 127) {
    ch3 = 127;
  }

  if (ch4 < -127) {
    ch4 = 127;
  } else if (ch4 > 127) {
    ch4 = 127;
  }
}
