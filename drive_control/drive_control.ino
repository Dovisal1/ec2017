#include <SPI.h>
#include "RF24.h"

byte addresses[][7] = {"3drive", "3cntrl"};
RF24 radio(7, 8);

void setup() {
  // put your setup code here, to run once:
  // Driver Set up
  TCCR1A = 0xA2;
  TCCR1B = 0x12;
  TCCR1C = 0x00;
  ICR1 = 20000;
  OCR1A = 1500; //right
  OCR1B = 1500; //left
  DDRB = 0xff;

  // Radio Communication Setup
  radio.begin();
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();
}

void loop() {
  // put your main code here, to run repeatedly:
  float pr[2];

  if (radio.available()) {
      while(radio.available()) {
        radio.read(pr, sizeof(pr));
      }
  
    float rspeed, lspeed, diff;
    int sign;
  
    if (pr[0] > 0)
      sign = 1;
    else
      sign = -1;
  
    int turn;
    if (pr[1] > 0)
      turn = 1;
    else
      turn = -1;
  
    if (pr[1] > 20 || pr[1] < -20) {//turn
      diff = map(pr[1], turn * 20, turn * 50, 1, 1.3) * pr[0];
      lspeed = range_value(pr[0] - turn * sign * diff);
      rspeed = range_value(pr[0] + turn * sign * diff);
      
        
      OCR1A = map(rspeed, -75, 75, 1000, 2000);
      OCR1B = map(lspeed, -75, 75, 1000, 2000);
    } else {
      OCR1A = OCR1B = map(pr[0], -75, 75, 1000, 2000);
    }
   }
}

float range_value(float num) {
  if (num > 75)
      return 75;
  else if (num < -75)
      return -75;
  else
      return num;
}
