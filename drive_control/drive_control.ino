
#include <SPI.h>
#include "RF24.h"

byte addresses[][7] = {"3drive", "3cntrl"};
RF24 radio(7, 8);

#define DEBUG 0
#define Dprint(s) if (DEBUG) Serial.print(s)
#define Dprintln(s) if (DEBUG) Serial.println(s)

void setup() {

  // PWM out setup
  TCCR1A = 0xA2;
  TCCR1B = 0x12;
  TCCR1C = 0x00;
  ICR1 = 20000; // top
  OCR1A = 1500; //right
  OCR1B = 1500; //left
  
  DDRB = 0x06;

  #if DEBUG == 1
    Serial.begin(9600);
    while(!Serial)
      ;
  #endif

  // Radio Communication Setup
  radio.begin();
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();
}

unsigned long last_rx = 0;

float pmax = 55;
float rmax = 55;

#define TURN_THRESHOLD 15
#define TURN_CURVE 3.5
#define THROTTLE_THRESHOLD 10
#define THROTTLE_MAX 55
#define THROTTLE_CURVE 0.75
#define TIMEOUT 500

void loop() {
  
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
      if (r > TURN_THRESHOLD) { //left turn
        cap = polymap(r, TURN_THRESHOLD, rmax, 0.0, 100.0, TURN_CURVE);
        rpercent = (300.0 - cap) / 200.0;
        lpercent = (100.0 - cap) / 100.0;
      } else if (r < -TURN_THRESHOLD) { //right turn
        cap = polymap(r, -TURN_THRESHOLD, -rmax, 0.0, 100.0, TURN_CURVE);
        lpercent = (300.0 - cap) / 200.0;
        rpercent = (100.0 - cap) / 100.0;
      } else {
        lpercent = rpercent = 1.0;
      }

      /* speed mapping */
      if (speed > THROTTLE_MAX)
        diff = 500;
      else if (speed > THROTTLE_THRESHOLD)
        diff = polymap(speed, THROTTLE_THRESHOLD, THROTTLE_MAX, 50, 500, THROTTLE_CURVE);
      else
        diff = map(speed, 0, THROTTLE_THRESHOLD, 0, 50);

      /* apportioning speed to each wheel based on turn */
      rdiff = diff * rpercent;
      ldiff = diff * lpercent;

      /* adjusting for speed limit on each wheel */
      if (rdiff > 500) {
          ldiff -= (rdiff - 500); //offset the speed loss
          rdiff = 500;
      } else if (ldiff > 500) {
        rdiff -= (ldiff - 500);
        ldiff = 500;
      }

      /* invert the pulse differential from the mean if going in reverse */
      if (p < 0) {
        rdiff = -rdiff;
        ldiff = -ldiff;
      }

      /* update the pwm motor output */
      OCR1A = 1500 + rdiff;
      OCR1B = 1500 + ldiff;

      #if DEBUG == 1
        char fmt[] = "p = %s; r = %s; diff = %d; OCR1A = %d; OCR1B = %d\n";
        char buf[256];
        char rbuf[16], pbuf[16];
        dtostrf(p, 4, 2, pbuf);
        dtostrf(r, 4, 2, rbuf);
        sprintf(buf, fmt, pbuf, rbuf, diff, OCR1A, OCR1B);
        Serial.println(buf);
      #endif

      last_rx = millis();
  } else if (millis() - last_rx > TIMEOUT) {
    OCR1A = OCR1B = 1500; // stop motors
    last_rx = millis();
  }
}

static inline float polymap(float x, float imn, float imx, float omn, float omx, float curve) {
    return omn + (omx - omn) * pow((x - imn) / (imx - imn), curve);
}

