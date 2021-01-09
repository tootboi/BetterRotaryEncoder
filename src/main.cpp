#include <Arduino.h>
#include <HID-Project.h>

#define CLK A2
#define DATA A0

void digitalDebounce();
void tableDecode();

void setup() {
  Serial.begin(9600);

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DATA, INPUT_PULLUP);

  Consumer.begin(); //For writing media keys.
}

void loop() {
  //digitalDebounce();
  tableDecode();
}

void digitalDebounce() {
  static uint16_t state = 0, counter = 0;

  //this line allows for debounce without the use of delay() function.
    //it works by left bit shifting the state while encoder pin is pulled down (rotating),
    //untill state eventually becomes 0xf000.
  state = (state << 1) | digitalRead(CLK) | 0xe000;

  if(state == 0xf000) {
    // Serial.print("CLK: ");
    // Serial.println(CLK);

    state=0x0000;
    if(digitalRead(DATA)) {
      counter++;
      Consumer.write(MEDIA_VOLUME_UP);
    } else {
      counter--;
      Consumer.write(MEDIA_VOLUME_DOWN);
    }
    Serial.println(counter);
  }
}

//for noisy rotary encoders
void tableDecode() {
  static uint8_t prevStates = 0;
  static uint8_t validation = 0;    //for validating if last two prevStates are valid
  static int8_t encoderLUT[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

  prevStates <<= 2;
  if(digitalRead(DATA)) prevStates |= 0x02;
  if(digitalRead(CLK)) prevStates |= 0x01;
  prevStates &= 0xf;    //this removes bits beyond 4 bits, that were created by the bit shift

  int8_t decodedVal = encoderLUT[prevStates];
  if(decodedVal) {
    validation <<= 4;
    validation |= prevStates;
    if(validation==23) {    //BINARY 00010111
      Serial.println("CW");
      Consumer.write(MEDIA_VOLUME_UP);
    } else if(validation==43) {   //BINARY 00101011
      Serial.println("CCW");
      Consumer.write(MEDIA_VOLUME_DOWN);
    }
  }
}