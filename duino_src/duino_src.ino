/**
  Arduino code:
  For single programmable LED
  receive message on serial USB and change LED

  serial messages should be semi-colon delimited
  eg 0;20;40;10
  first value is mode
  mode 0 = solid [g r b]
  mode 1 = flash [g r b onTime offTime]
  mode 2 = random [not quite working]
  mode 3 = rainbow fade,  second argument should be 25 for fast fade
  mode 9 = set brightness (right shift amount of GRB)

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

  //  idleCol =  strip.Color(ColG, ColR, ColB);
}

/**
   Main Arduino loop.
*/
void loop() {

  while (Serial.available() == 0) {
    strip.setPixelColor(0, 255 >> brightness, 0, 0);
    strip.show();

    for (i = 0; i < 65530; i++) {
      checkSerialInput();
    }
    strip.setPixelColor(0, 0, 255 >> brightness, 0);
    strip.show();

    for (i = 0; i < 65530; i++) {
      checkSerialInput();
    }

    strip.setPixelColor(0, 0, 0, 255 >> brightness);
    strip.show();

    for (i = 0; i < 65530; i++) {
      checkSerialInput();
    }
  }

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
    //    Serial.println(str);
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
    Mode = 0;
    mode0();
  }

  if (serial_in == '1') {
    Mode = 1;
    mode1();
  }

  if (serial_in == '2') {
    Mode = 2;
    mode2();
  }

  if (serial_in == '3') {
    Mode = 3;
    mode3();
  }

  if (serial_in == '9') {
    Mode = 9;
    mode9();
  }

  return;

}

void mode0() {
  Serial.println("mode 0");
  // solid

    ColR = data[1] ;
    ColG = data[2] ;
    ColB = data[3] ;

  while (Serial.available() == 0) {

    strip.setPixelColor(0, ColG>> brightness, ColR>> brightness, ColB>> brightness);
    strip.show();

    for (i = 0; i < 65530; i++) {
      checkSerialInput();
    }

  }

}

void mode1() {
  Serial.println("mode 1");
  // flash

    ColR = data[1] ;
    ColG = data[2] ;
    ColB = data[3];

    onTime = data[4];
    offTime = data[5];
    
  while (Serial.available() == 0) {

//    Serial.println(brightness);
//    Serial.println("");


    //    Serial.print("ColG:");
    //    Serial.println(ColG);
    //     Serial.print("ColR:");
    //    Serial.println(ColR);
    //     Serial.print("ColB:");
    //    Serial.println(ColB);

    strip.setPixelColor(0, ColG>> brightness, ColR>> brightness, ColB>> brightness);
    strip.show();

    for (i = 0; i < onTime; i++) {
      for (j = 0; j < onTime; j++) {
        checkSerialInput();
      }
    }

    strip.setPixelColor(0, 0, 0, 0);
    strip.show();

    for (i = 0; i < offTime; i++) {
      for (j = 0; j < offTime; j++) {
        checkSerialInput();
      }
    }

  }

}


void mode2() {
  Serial.println("mode 2");
  // flash random
    onTime = data[1];
    offTime = data[2];
    
  while (Serial.available() == 0) {

    for (k = 1; k < 97; k++) {
      ColR = (k & 1) << 16;
      ColG = (k & 2) << 8;
      ColB = k & 4 << 4;
      strip.setPixelColor(0, ColR >> brightness, ColG >> brightness, ColB >> brightness);
      strip.show();
      for (i = 0; i < onTime; i++) {
        for (j = 0; j < onTime; j++) {
          checkSerialInput();
        }
      }
      strip.setPixelColor(0, 0, 0, 0);
      strip.show();
      for (i = 0; i < offTime; i++) {
        for (j = 0; j < offTime; j++) {
          checkSerialInput();
        }
      }
    }
  }
}

void mode3() {
  Serial.println("mode 3");
  // fade rainbow
  // second argument should be 25 for fast fade

  onTime = data[1];
  while (Serial.available() == 0) {
    
    for (int k = 0; k < 360; k++)
    {
      trueHSV(0, k);

      for (i = 0; i < onTime; i++) {
        for (j = 0; j < onTime; j++) {
          checkSerialInput();
        }
      }
    }
  }

}

void mode9() {
  Serial.println("mode 9: adjust brightness");

  brightness = data[1];

  Serial.println(brightness);
  //   Serial.println(Mode);
  Serial.println();

  if (Mode == 0) {
    Serial.println(Mode);
    Mode = 0;
    mode0();
  }

  if (Mode == 1) {
    Mode = 1;
    mode1();
  }

  if (Mode == 2) {
    Mode = 2;
    mode2();
  }

  if (Mode == 3) {
    Mode = 3;
    mode3();
  }

  return;

}


// the real HSV rainbow
void trueHSV(byte LED, int angle)
{
  byte red, green, blue;

  if (angle < 60) {
    red = 255;
    green = HSVlights[angle];
    blue = 0;
  } else if (angle < 120) {
    red = HSVlights[120 - angle];
    green = 255;
    blue = 0;
  } else if (angle < 180) {
    red = 0, green = 255;
    blue = HSVlights[angle - 120];
  } else if (angle < 240) {
    red = 0, green = HSVlights[240 - angle];
    blue = 255;
  } else if (angle < 300) {
    red = HSVlights[angle - 240], green = 0;
    blue = 255;
  } else
  {
    red = 255, green = 0;
    blue = HSVlights[360 - angle];
  }
  setRGBpoint(red, green, blue);
}

void setRGBpoint(uint8_t red, uint8_t green, uint8_t blue)
{
  // this code is for common anode LEDs. If you use common cathode ones,
  // remove the '255-' bits.
  //  analogWrite(outputPins[LED*3], 255-red);
  //  analogWrite(outputPins[LED*3+1], 255-green);
  //  analogWrite(outputPins[LED*3+2], 255-blue);

  strip.setPixelColor(0, green >> brightness, red >> brightness, blue >> brightness);
  strip.show();
}
