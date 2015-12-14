// SparkFun Serial LCD code

void selectLineOne() {  // puts the cursor at line 0 char 0.
  myLCD.write(0xFE); // command flag
  myLCD.write(128); // position
  delay(10);
}
void selectLineTwo() { // puts the cursor at line 1 char 0.
  myLCD.write(0xFE); // command flag
  myLCD.write(192); // position
  delay(10);
}

void goTo(int position) { // position = line 0: 0-15, line 1: 16-31, 31+ defaults back to 0
  if (position < 16) {
    myLCD.write(0xFE); //command flag
    myLCD.write((position + 128));
  } else if (position < 32) {
    myLCD.write(0xFE); //command flag
    myLCD.write((position + 64 + 128));
  } else {
    goTo(0);
  }
  
  delay(10);
}

void clearLCD() {
  myLCD.write(0xFE); // command flag
  myLCD.write(0x01); // clear command
  delay(10);
}

void backlightOn() { // turns on the backlight
  myLCD.write(0x7C); // command flag for backlight stuff
  myLCD.write(157); // light level for on
  delay(10);
}

void backlightOff() { // turns off the backlight
  myLCD.write(0x7C); // command flag for backlight stuff
  myLCD.write(128); // light level for off
  delay(10);
}

void serCommand() { // a general function to call the command flag for issuing all other commands   
  myLCD.write(0xFE);
}

