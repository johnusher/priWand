#ifndef _ARD_JU_ACH_
#define _ARD_JU_ACH_


char serial_in;
int Mode = 0;



String serialResponse = "";
char sz[] = "1; 2; 3;4;5;6.414;7";

char binary[3] = {0}; //This is where the binary representation will be stored
byte someValue = 1; // we loop over 3 bits

//char array[] = "0;123;456;789";

int data[10];

char *strings[10];
char *ptr = NULL;

//int ind1; // , locations
//int ind2;
//int ind3;
//int ind4;

String readString; //main captured String 
String code;
String data1;

int idleC = 0;	// counter for led position
int colC = 0;	// counter for led colour


uint8_t ColR = 64; 	// colour of idle
uint8_t ColG = 32; 
uint8_t ColB = 49; 

uint8_t brightness = 4;   // amount we right shift by

int onTime = 5000; 
int offTime = 5000; 

uint8_t maxC = 128;   // max colour value

bool idle_flag = 1; 

const byte nLEDS = 1;
const byte ledPin = 6;

int breakFlag = 0; 

uint16_t i, j,k;	

// for rainbow effect:
const uint8_t HSVlights[61] = 
{0, 4, 8, 13, 17, 21, 25, 30, 34, 38, 42, 47, 51, 55, 59, 64, 68, 72, 76,
81, 85, 89, 93, 98, 102, 106, 110, 115, 119, 123, 127, 132, 136, 140, 144,
149, 153, 157, 161, 166, 170, 174, 178, 183, 187, 191, 195, 200, 204, 208,
212, 217, 221, 225, 229, 234, 238, 242, 246, 251, 255};




#endif
