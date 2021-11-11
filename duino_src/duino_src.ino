/**
  Arduino code:
  For single programmable LED
  receive message on serial USB and change LED

*/


#include "ard_JU.h"

#include "Adafruit_NeoPixel.h"   // from https://github.com/adafruit/Adafruit_NeoPixel

Adafruit_NeoPixel strip = Adafruit_NeoPixel(nLEDS, ledPin, NEO_GRB + NEO_KHZ800);

void setup() {
  pinMode(ledPin, OUTPUT);
  strip.begin();
  //  myStripShow(); // Initialize all pixels to 'off'
  // Serial.begin(9600);
  // Serial.begin(19200);
  Serial.begin(115200);

  idleCol =  strip.Color(idleColR, idleColG, idleColB);
}

/**
   Main Arduino loop.
*/
void loop() {

  while (Serial.available() == 0) {
    strip.setPixelColor(0, 1 * brightness, 0, 0);
    strip.show();
    idleC = 0;


    for (i = 0; i < 65530; i++) {
      checkSerialInput();
    }

    strip.setPixelColor(0, 0, 1 * brightness, 0);
    strip.show();


    for (i = 0; i < 65530; i++) {
      checkSerialInput();

    }

    strip.setPixelColor(0, 0, 0, 1 * brightness);
    strip.show();


    for (i = 0; i < 65530; i++) {
      checkSerialInput();
    }


  }





  //  if (Serial.available() > 0) {
  //    // read the incoming byte:
  //    char c = Serial.read();
  //
  //    // say what you got:
  //    Serial.print("I received: ");
  //    Serial.println(c);
  //  }
}



void checkSerialInput() {



  //  https://forum.arduino.cc/t/split-string-by-delimiters/373124/3

//  or here:
//  https://forum.arduino.cc/t/splitting-up-a-string-with-a-delimiter-and-storing-each-cut-in-an-array/628574

  // input message is delimited with semi colon and termniated with newline
  // must begin with a 0

  //  idle_flag = 0;
  //  strip.clear();
  serial_in = Serial.read();

  if (serial_in == -1) {
    //    empty
    return;

  }

  // read in semi-colon delimited message
  serialResponse  = Serial.readStringUntil('\r\n');


  // Convert from String Object to String.
  char buf[sizeof(sz)];
  serialResponse.toCharArray(buf, sizeof(buf));
  char *p = buf;
  char *str;
  i=0;
  while ((str = strtok_r(p, ";", &p)) != NULL){ // delimiter is the semicolon
    Serial.println(str);
    buf[i] = str;
    i=i+1;
  }
  i=i-1;

  if (serial_in == '0') {

    // code 0 = solid
//    Serial.println(serial_in);
    idleColR = buf[0];
    idleColG = buf[1];
    idleColB = buf[2];
    Serial.println("colours");
    Serial.println(buf[0]);
    Serial.println(idleColG);
    Serial.println(idleColB);
    
  }

//  mode 1= solid flash
//  colour A
//  flash interval


  //  messSize = strlen(serial_in);

  //  Serial.println(messSize);

  // say what you got:
  if (messSize > 2) {

    //     Serial.println(messSize);
    ////    Serial.println(serial_in);
    ////    Serial.println(messSize);
    //
    ////    Serial.flush();

  }

  //  if (serial_in == '0') {
  //    SMode = 0;
  //    Serial.println("mode 0");
  //    Serial.flush();
  //    Serial.println("a");
  //
  //    idleC = 128;
  //    idleColR = 0;
  //    idleColG = 0;
  //    idleColB = 128;
  //
  //    strip.setPixelColor(idleC, idleColR, idleColG, idleColB);  // show blue light moving along
  //    Serial.println("b");
  //
  //
  //    breakFlag = 0;
  //    idleC = 0;
  //
  //  }
  //
  //  else if (serial_in == '1') {
  //    SMode = 1;
  //    Serial.println("mode 1");
  //    Serial.flush();
  //
  //    colorWipe(strip.Color(0, 0 , maxC ), 25); // g
  //    Serial.print("breakFlag = ");
  //    Serial.println(breakFlag);
  //
  //
  //    if (breakFlag == 1) {
  //      colorWipe(strip.Color(0, 0 , 0), 50); // off
  //    }
  //    else {
  //      wipeReverse = !wipeReverse;
  //    }
  //    breakFlag = 0;
  //    idleC = 0;
  //
  //  }



}

void myStripShow() {
  if (Serial.available() > 0) {
    checkSerialInput();
    breakFlag = 1;
  }
  //  strip.show();
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {

  breakFlag = 0;
  if (wipeReverse)
  {
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      //      tone(buzzer, i ); // Send 1KHz sound signal...
      myStripShow();
      if (breakFlag == 1) {
        break;
      }
      else {
        delay(wait);
      }
    }
  }
  else
  {
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(strip.numPixels() - i, c);
      //      tone(buzzer, i XI); // Send 1KHz sound signal...

      myStripShow();
      if (breakFlag == 1) {
        break;
      }
      else {
        delay(wait);
      }
    }
  }

  wipeReverse = !wipeReverse;


  //  noTone(buzzer);     // Stop sound...


}


// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));

      //     tone(buzzer, i + j); // Send xm sound signal...

    }
    myStripShow();
    delay(wait);
  }

}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
