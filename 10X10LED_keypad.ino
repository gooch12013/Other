#include <Keypad.h>
#include "FastLED.h"
#define NUM_LEDS 100
#define DATA_PIN 2
// #define CLOCK_PIN 13
CRGB leds[NUM_LEDS];






const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {3, 4, 5, 6}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {7, 8, 9,10};

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
byte ledPin = 13; 

boolean blink = false;
boolean ledPin_state;

void setup(){
    Serial.begin(9600);
    pinMode(ledPin, OUTPUT);              // Sets the digital pin as output.
    digitalWrite(ledPin, HIGH);           // Turn the LED on.
    ledPin_state = digitalRead(ledPin);   // Store initial LED state. HIGH when LED is on.
    keypad.addEventListener(keypadEvent); // Add an event listener for this keypad
    FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);


}

void loop(){
    char key = keypad.getKey();

    if (key) {
        Serial.println(key);
    }
    if (blink){
        digitalWrite(ledPin,!digitalRead(ledPin));    // Change the ledPin from Hi2Lo or Lo2Hi.
        delay(100);
    }
}

// Taking care of some special events.
void keypadEvent(KeypadEvent key){
    switch (keypad.getState()){
    case PRESSED:
        if (key == '1') {
            ledson(); 
        }
        break;
        if (key == '#') {
            alloff(); 
            
            
        }
        break;



    case RELEASED:
        if (key == '*') {
            
        }
        break;

    case HOLD:
        if (key == '*') {
            
        }
        break;
    }
}


void alloff (){
leds[0] = CRGB(0, 0, 0);
leds[1] = CRGB(0, 0, 0);
leds[2] = CRGB(0, 0, 0);
leds[3] = CRGB(0, 0, 0);
leds[4] = CRGB(0, 0, 0);
leds[5] = CRGB(0, 0, 0);
leds[6] = CRGB(0, 0, 0);
leds[7] = CRGB(0, 0, 0);
leds[8] = CRGB(0, 0, 0);
leds[9] = CRGB(0, 0, 0);
leds[10] = CRGB(0, 0, 0);
leds[11] = CRGB(0, 0, 0);
leds[12] = CRGB(0, 0, 0);
leds[13] = CRGB(0, 0, 0);
leds[14] = CRGB(0, 0, 0);
leds[15] = CRGB(0, 0, 0);
leds[16] = CRGB(0, 0, 0);
leds[17] = CRGB(0, 0, 0);
leds[18] = CRGB(0, 0, 0);
leds[19] = CRGB(0, 0, 0);
leds[20] = CRGB(0, 0, 0);
leds[21] = CRGB(0, 0, 0);
leds[22] = CRGB(0, 0, 0);
leds[23] = CRGB(0, 0, 0);
leds[24] = CRGB(0, 0, 0);
leds[25] = CRGB(0, 0, 0);
leds[26] = CRGB(0, 0, 0);
leds[27] = CRGB(0, 0, 0);
leds[28] = CRGB(0, 0, 0);
leds[29] = CRGB(0, 0, 0);
leds[30] = CRGB(0, 0, 0);
leds[31] = CRGB(0, 0, 0);
leds[32] = CRGB(0, 0, 0);
leds[33] = CRGB(0, 0, 0);
leds[34] = CRGB(0, 0, 0);
leds[35] = CRGB(0, 0, 0);
leds[36] = CRGB(0, 0, 0);
leds[37] = CRGB(0, 0, 0);
leds[38] = CRGB(0, 0, 0);
leds[39] = CRGB(0, 0, 0);
leds[40] = CRGB(0, 0, 0);
leds[41] = CRGB(0, 0, 0);
leds[42] = CRGB(0, 0, 0);
leds[43] = CRGB(0, 0, 0);
leds[44] = CRGB(0, 0, 0);
leds[45] = CRGB(0, 0, 0);
leds[46] = CRGB(0, 0, 0);
leds[47] = CRGB(0, 0, 0);
leds[48] = CRGB(0, 0, 0);
leds[49] = CRGB(0, 0, 0);
leds[50] = CRGB(0, 0, 0);
leds[51] = CRGB(0, 0, 0);
leds[52] = CRGB(0, 0, 0);
leds[53] = CRGB(0, 0, 0);
leds[54] = CRGB(0, 0, 0);
leds[55] = CRGB(0, 0, 0);
leds[56] = CRGB(0, 0, 0);
leds[57] = CRGB(0, 0, 0);
leds[58] = CRGB(0, 0, 0);
leds[59] = CRGB(0, 0, 0);
leds[60] = CRGB(0, 0, 0);
leds[61] = CRGB(0, 0, 0);
leds[62] = CRGB(0, 0, 0);
leds[63] = CRGB(0, 0, 0);
leds[64] = CRGB(0, 0, 0);
leds[65] = CRGB(0, 0, 0);
leds[66] = CRGB(0, 0, 0);
leds[67] = CRGB(0, 0, 0);
leds[68] = CRGB(0, 0, 0);
leds[69] = CRGB(0, 0, 0);
leds[70] = CRGB(0, 0, 0);
leds[71] = CRGB(0, 0, 0);
leds[72] = CRGB(0, 0, 0);
leds[73] = CRGB(0, 0, 0);
leds[74] = CRGB(0, 0, 0);
leds[75] = CRGB(0, 0, 0);
leds[76] = CRGB(0, 0, 0);
leds[77] = CRGB(0, 0, 0);
leds[78] = CRGB(0, 0, 0);
leds[79] = CRGB(0, 0, 0);
leds[80] = CRGB(0, 0, 0);
leds[81] = CRGB(0, 0, 0);
leds[82] = CRGB(0, 0, 0);
leds[83] = CRGB(0, 0, 0);
leds[84] = CRGB(0, 0, 0);
leds[85] = CRGB(0, 0, 0);
leds[86] = CRGB(0, 0, 0);
leds[87] = CRGB(0, 0, 0);
leds[88] = CRGB(0, 0, 0);
leds[89] = CRGB(0, 0, 0);
leds[90] = CRGB(0, 0, 0);
leds[91] = CRGB(0, 0, 0);
leds[92] = CRGB(0, 0, 0);
leds[93] = CRGB(0, 0, 0);
leds[94] = CRGB(0, 0, 0);
leds[95] = CRGB(0, 0, 0);
leds[96] = CRGB(0, 0, 0);
leds[97] = CRGB(0, 0, 0);
leds[98] = CRGB(0, 0, 0);
leds[99] = CRGB(0, 0, 0);
FastLED.show();
  
  
  
  
  }


void ledson (){
int x=1;
 while(x==1){ 
 Serial.println("LED ON");

leds[0] = CRGB(0, 0, 0);
leds[1] = CRGB(0, 0, 0);
leds[2] = CRGB(0, 0, 0);
leds[3] = CRGB(0, 0, 0);
leds[4] = CRGB(0, 0, 0);
leds[5] = CRGB(0, 0, 0);
leds[6] = CRGB(0, 0, 0);
leds[7] = CRGB(0, 0, 0);
leds[8] = CRGB(0, 0, 0);
leds[9] = CRGB(0, 0, 0);
leds[10] = CRGB(0, 0, 0);
leds[11] = CRGB(0, 0, 0);
leds[12] = CRGB(0, 0, 0);
leds[13] = CRGB(0, 0, 0);
leds[14] = CRGB(0, 0, 0);
leds[15] = CRGB(0, 0, 0);
leds[16] = CRGB(0, 0, 0);
leds[17] = CRGB(0, 0, 0);
leds[18] = CRGB(0, 0, 0);
leds[19] = CRGB(0, 0, 0);
leds[20] = CRGB(0, 0, 0);
leds[21] = CRGB(0, 0, 0);
leds[22] = CRGB(0, 0, 0);
leds[23] = CRGB(0, 0, 0);
leds[24] = CRGB(0, 0, 0);
leds[25] = CRGB(0, 0, 0);
leds[26] = CRGB(0, 0, 0);
leds[27] = CRGB(0, 0, 0);
leds[28] = CRGB(0, 0, 0);
leds[29] = CRGB(0, 0, 0);
leds[30] = CRGB(0, 0, 0);
leds[31] = CRGB(0, 0, 0);
leds[32] = CRGB(0, 0, 0);
leds[33] = CRGB(0, 0, 0);
leds[34] = CRGB(0, 0, 0);
leds[35] = CRGB(0, 0, 0);
leds[36] = CRGB(0, 0, 0);
leds[37] = CRGB(0, 0, 0);
leds[38] = CRGB(0, 0, 0);
leds[39] = CRGB(0, 0, 0);
leds[40] = CRGB(0, 0, 0);
leds[41] = CRGB(0, 0, 0);
leds[42] = CRGB(0, 0, 0);
leds[43] = CRGB(0, 0, 0);
leds[44] = CRGB(0, 0, 0);
leds[45] = CRGB(0, 0, 0);
leds[46] = CRGB(0, 0, 0);
leds[47] = CRGB(0, 0, 0);
leds[48] = CRGB(0, 0, 0);
leds[49] = CRGB(0, 0, 0);
leds[50] = CRGB(0, 0, 0);
leds[51] = CRGB(0, 0, 0);
leds[52] = CRGB(0, 0, 0);
leds[53] = CRGB(0, 0, 0);
leds[54] = CRGB(0, 0, 0);
leds[55] = CRGB(0, 0, 0);
leds[56] = CRGB(0, 0, 0);
leds[57] = CRGB(0, 0, 0);
leds[58] = CRGB(0, 0, 0);
leds[59] = CRGB(0, 0, 0);
leds[60] = CRGB(0, 0, 0);
leds[61] = CRGB(0, 0, 0);
leds[62] = CRGB(0, 0, 0);
leds[63] = CRGB(0, 0, 0);
leds[64] = CRGB(0, 0, 0);
leds[65] = CRGB(0, 0, 0);
leds[66] = CRGB(0, 0, 0);
leds[67] = CRGB(0, 0, 0);
leds[68] = CRGB(0, 0, 0);
leds[69] = CRGB(0, 0, 0);
leds[70] = CRGB(0, 0, 0);
leds[71] = CRGB(0, 0, 0);
leds[72] = CRGB(0, 0, 0);
leds[73] = CRGB(0, 0, 0);
leds[74] = CRGB(0, 0, 0);
leds[75] = CRGB(0, 0, 0);
leds[76] = CRGB(0, 0, 0);
leds[77] = CRGB(0, 0, 0);
leds[78] = CRGB(0, 0, 0);
leds[79] = CRGB(0, 0, 0);
leds[80] = CRGB(0, 0, 0);
leds[81] = CRGB(0, 0, 0);
leds[82] = CRGB(0, 0, 0);
leds[83] = CRGB(0, 0, 0);
leds[84] = CRGB(0, 0, 0);
leds[85] = CRGB(0, 0, 0);
leds[86] = CRGB(0, 0, 0);
leds[87] = CRGB(0, 0, 0);
leds[88] = CRGB(0, 0, 0);
leds[89] = CRGB(0, 0, 0);
leds[90] = CRGB(0, 0, 0);
leds[91] = CRGB(0, 0, 0);
leds[92] = CRGB(0, 0, 0);
leds[93] = CRGB(0, 0, 0);
leds[94] = CRGB(0, 0, 0);
leds[95] = CRGB(0, 0, 0);
leds[96] = CRGB(0, 0, 0);
leds[97] = CRGB(0, 0, 0);
leds[98] = CRGB(0, 0, 0);
leds[99] = CRGB(0, 0, 0);
FastLED.show();

delay(2000);
 
leds[0] = CRGB(0, 0, 255);
leds[1] = CRGB(0, 0, 255);
leds[2] = CRGB(0, 0, 255);
leds[3] = CRGB(0, 0, 255);
leds[4] = CRGB(0, 0, 255);
leds[5] = CRGB(0, 0, 255);
leds[6] = CRGB(0, 0, 255);
leds[7] = CRGB(0, 0, 255);
leds[8] = CRGB(0, 0, 255);
leds[9] = CRGB(0, 0, 255);
leds[10] = CRGB(0, 0, 255);
leds[11] = CRGB(0, 255, 0);
leds[12] = CRGB(0, 255, 0);
leds[13] = CRGB(0, 255, 0);
leds[14] = CRGB(0, 255, 0);
leds[15] = CRGB(0, 255, 0);
leds[16] = CRGB(0, 255, 0);
leds[17] = CRGB(0, 255, 0);
leds[18] = CRGB(0, 255, 0);
leds[19] = CRGB(0, 0, 255);
leds[20] = CRGB(0, 0, 255);
leds[21] = CRGB(0, 255, 0);
leds[22] = CRGB(255, 0, 255);
leds[23] = CRGB(255, 0, 255);
leds[24] = CRGB(255, 0, 255);
leds[25] = CRGB(255, 0, 255);
leds[26] = CRGB(255, 0, 255);
leds[27] = CRGB(255, 0, 255);
leds[28] = CRGB(0, 255, 0);
leds[29] = CRGB(0, 0, 255);
leds[30] = CRGB(0, 0, 255);
leds[31] = CRGB(0, 255, 0);
leds[32] = CRGB(255, 0, 255);
leds[33] = CRGB(255, 0, 111);
leds[34] = CRGB(255, 0, 111);
leds[35] = CRGB(255, 0, 111);
leds[36] = CRGB(255, 0, 111);
leds[37] = CRGB(255, 0, 255);
leds[38] = CRGB(0, 255, 0);
leds[39] = CRGB(0, 0, 255);
leds[40] = CRGB(0, 0, 255);
leds[41] = CRGB(0, 255, 0);
leds[42] = CRGB(255, 0, 255);
leds[43] = CRGB(255, 0, 111);
leds[44] = CRGB(0, 0, 255);
leds[45] = CRGB(0, 0, 255);
leds[46] = CRGB(255, 0, 111);
leds[47] = CRGB(255, 0, 255);
leds[48] = CRGB(0, 255, 0);
leds[49] = CRGB(0, 0, 255);
leds[50] = CRGB(0, 0, 255);
leds[51] = CRGB(0, 255, 0);
leds[52] = CRGB(255, 0, 255);
leds[53] = CRGB(255, 0, 111);
leds[54] = CRGB(0, 0, 255);
leds[55] = CRGB(0, 0, 255);
leds[56] = CRGB(255, 0, 111);
leds[57] = CRGB(255, 0, 255);
leds[58] = CRGB(0, 255, 0);
leds[59] = CRGB(0, 0, 255);
leds[60] = CRGB(0, 0, 255);
leds[61] = CRGB(0, 255, 0);
leds[62] = CRGB(255, 0, 255);
leds[63] = CRGB(255, 0, 111);
leds[64] = CRGB(255, 0, 111);
leds[65] = CRGB(255, 0, 111);
leds[66] = CRGB(255, 0, 111);
leds[67] = CRGB(255, 0, 255);
leds[68] = CRGB(0, 255, 0);
leds[69] = CRGB(0, 0, 255);
leds[70] = CRGB(0, 0, 255);
leds[71] = CRGB(0, 255, 0);
leds[72] = CRGB(255, 0, 255);
leds[73] = CRGB(255, 0, 255);
leds[74] = CRGB(255, 0, 255);
leds[75] = CRGB(255, 0, 255);
leds[76] = CRGB(255, 0, 255);
leds[77] = CRGB(255, 0, 255);
leds[78] = CRGB(0, 255, 0);
leds[79] = CRGB(0, 0, 255);
leds[80] = CRGB(0, 0, 255);
leds[81] = CRGB(0, 255, 0);
leds[82] = CRGB(0, 255, 0);
leds[83] = CRGB(0, 255, 0);
leds[84] = CRGB(0, 255, 0);
leds[85] = CRGB(0, 255, 0);
leds[86] = CRGB(0, 255, 0);
leds[87] = CRGB(0, 255, 0);
leds[88] = CRGB(0, 255, 0);
leds[89] = CRGB(0, 0, 255);
leds[90] = CRGB(0, 0, 255);
leds[91] = CRGB(0, 0, 255);
leds[92] = CRGB(0, 0, 255);
leds[93] = CRGB(0, 0, 255);
leds[94] = CRGB(0, 0, 255);
leds[95] = CRGB(0, 0, 255);
leds[96] = CRGB(0, 0, 255);
leds[97] = CRGB(0, 0, 255);
leds[98] = CRGB(0, 0, 255);
leds[99] = CRGB(0, 0, 255);
FastLED.show(); 
delay(1000);
 }
 x= 1;
 }
