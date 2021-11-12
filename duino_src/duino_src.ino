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

  idleCol =  strip.Color(idleColG, idleColR, idleColB);
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

  // expect a strong like 0,1,123,456,789*

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

  serialResponse = Serial.readStringUntil('\r\n');
  //  Serial.println(serialResponse);

  // Convert from String Object to String.
  char buf[sizeof(sz)];
  serialResponse.toCharArray(buf, sizeof(buf));
  char *p = buf;
  char *str;
  i = 1;
  while ((str = strtok_r(p, ";", &p)) != NULL) { // delimiter is the semicolon
    Serial.println(str);
    strings[i] = str;
    i = i + 1;
  }
  i = i - 1;

  //  Serial.println("i:");
  //  Serial.println(i);

  for (j = 1; j < i + 1; j++) {
    //      Serial.println("j:");
    data[j] = String(strings[j]).toInt();
    //    Serial.print("a:");
    //    Serial.println(strings[j] );
    //    Serial.print("b:");
    //    Serial.println(data[j] );
    //    Serial.println();
  }

  if (serial_in == '0') {
    mode0();
  }

   if (serial_in == '1') {
    mode1();
  }

  return;

}

void mode0() {
  Serial.println("mode 0");
  // solid
  while (Serial.available() == 0) {

    idleC = 0;

    idleColR = data[1];
    idleColG = data[2];
    idleColB = data[3];

//    Serial.print("idleColG:");
//    Serial.println(idleColG);
//     Serial.print("idleColR:");
//    Serial.println(idleColR);
//     Serial.print("idleColB:");
//    Serial.println(idleColB);

    strip.setPixelColor(0, idleColG, idleColR, idleColB);
    strip.show();

    for (i = 0; i < 65530; i++) {
      checkSerialInput();
    }

  }

}

void mode1() {
  Serial.println("mode 2");
  // flash
  while (Serial.available() == 0) {

    idleC = 0;

    idleColR = data[1];
    idleColG = data[2];
    idleColB = data[3];
    flashRate = data[4];

//    Serial.print("idleColG:");
//    Serial.println(idleColG);
//     Serial.print("idleColR:");
//    Serial.println(idleColR);
//     Serial.print("idleColB:");
//    Serial.println(idleColB);

    strip.setPixelColor(0, idleColG, idleColR, idleColB);
    strip.show();

    for (i = 0; i < flashRate; i++) {
      checkSerialInput();
    }

    strip.setPixelColor(0, 0, 0, 0);
    strip.show();

    for (i = 0; i < flashRate; i++) {
      checkSerialInput();
    }

  }

}


//  if (serial_in == '0') {
//
//    // code 0 = solid
//    //    Serial.println(serial_in);
//    idleColR = buf[0];
//    idleColG = buf[1];
//    idleColB = buf[2];
//    Serial.println("colours");
//    Serial.println(buf[0]);
//    Serial.println(idleColG);
//    Serial.println(idleColB);
//
//  }

//  mode 1= solid flash
//  colour A
//  flash interval


//  messSize = strlen(serial_in);

//  Serial.println(messSize);


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
