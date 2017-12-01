#include <SPI.h>
#include "RF24.h"

byte addresses[][7] = {"3drive", "3cntrl"};
RF24 radio(7, 8);

#define DEBUG 0
#define Dprint(s) if (DEBUG) Serial.print(s)
#define Dprintln(s) if (DEBUG) Serial.println(s)

void setup() {
  // put your setup code here, to run once:
  // Driver Set up
  TCCR1A = 0xA2;
  TCCR1B = 0x12;
  TCCR1C = 0x00;
  ICR1 = 20000; // top
  OCR1A = 1500; //right
  OCR1B = 1500; //left
  
  DDRB = 0xff;

  if (DEBUG) {
    Serial.begin(9600);
    while(!Serial)
      ;
  }

  // Radio Communication Setup
  radio.begin();
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();
}

unsigned long last_rx = 0;

float pmax = 55;
float rmax = 55;

#define TURN_THRESHOLD 14
#define TURN_CURVE 4
#define THROTTLE_THRESHOLD 8
#define THROTTLE_MAX 55
#define TIMEOUT 250

void loop() {
  // put your main code here, to run repeatedly:
  float p, r, pr[2]; // pitch roll values

  float cap;
  int speed;
  float lpercent, rpercent;
  int diff, rdiff, ldiff;
  
  if (radio.available()) {

      while(radio.available()) {
        radio.read(pr, sizeof(pr));
      }
      
      p = pr[0];
      r = pr[1];

      speed = abs(p);
      
      if (speed > pmax)
        pmax = speed;
      if (abs(r) > rmax)
        rmax = abs(r);

      /* turning logic */
      if (r >= TURN_THRESHOLD) { //left turn
        cap = logmap(r, TURN_THRESHOLD, rmax, 0.0, 100.0, TURN_CURVE);
        rpercent = (150.0 - cap/2) / 100.0;
        //rpercent = 1;
        lpercent = (100.0 - cap) / 100.0;
      } else if (r <= -TURN_THRESHOLD) { //right turn
        cap = logmap(r, -TURN_THRESHOLD, -rmax, 0.0, 100.0, TURN_CURVE);
        lpercent = (150.0 - cap/2) / 100.0;
        //lpercent = 1;
        rpercent = (100.0 - cap) / 100.0;
      } else {
        lpercent = rpercent = 1;
      }

      if (speed > THROTTLE_MAX)
        diff = 500;
      else if (speed > THROTTLE_THRESHOLD)
        diff = logmap(speed, THROTTLE_THRESHOLD, THROTTLE_MAX, 50, 500, 0.8);
      else
        diff = map(speed, 0, THROTTLE_THRESHOLD, 0, 50);

      rdiff = diff * rpercent;
      ldiff = diff * lpercent;
      
      if (rdiff > 500) {
          ldiff -= (rdiff - 500);
          rdiff = 500;
      } else if (ldiff > 500) {
        rdiff -= (ldiff - 500);
        ldiff = 500;
      }
      
      if (p >= 0) {
        OCR1A = 1500 + rdiff;
        OCR1B = 1500 + ldiff;
      } else {
        OCR1A = 1500 - rdiff;
        OCR1B = 1500 - ldiff;
      }

      if (DEBUG) {
        char fmt[] = "p = %s; r = %s; diff = %d; OCR1A = %d; OCR1B = %d\n";
        char buf[256];
        char rbuf[16], pbuf[16];
        dtostrf(p, 4, 2, pbuf);
        dtostrf(r, 4, 2, rbuf);
        sprintf(buf, fmt, pbuf, rbuf, diff, OCR1A, OCR1B);
        Serial.println(buf);
      }

      last_rx = millis();
  } else if (millis() - last_rx > TIMEOUT) {
    OCR1A = OCR1B = 1500; // stop motors
    last_rx = millis();
  }
}

float logmap(float x, float imn, float imx, float omn, float omx, float curve) {
  return omn + (omx - omn) * pow((x - imn) / (imx - imn), curve);
}

