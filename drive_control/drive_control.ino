
#include <SPI.h>
#include "RF24.h"

RF24 radio(7, 8);
byte addresses[][7] = {"3drive", "3cntrl"};

// set DEBUG to 1 for serial status updates when testing
#define DEBUG 0
#define Dprint(s) if (DEBUG) Serial.print(s)
#define Dprintln(s) if (DEBUG) Serial.println(s)

void setup()
{
  /** PWM motor driver setup
   *
   * Phase Correct PWM mode with ICR as top
   * clock prescaler = 8
   * ICR1 (top) = 20000
   * so f = 16Mhz / 8 / 20000 / 2 = 50Hz
   * OCR1A and OCR1B control the duty cycle on PB1 and PB2
   * 
   * each clock tick is 1us, so OCR1A and OCR1B should be
   * on the range 1000 to 2000 for 1ms to 2ms pulses
   * 1000: max reverse
   * 1500: stop
   * 2000: max forward
   * (in practice we found that the motors were not so precise,
   * and going a little above 2000 or below 1000 produces faster
   * speeds)
   */

  TCCR1A = 0xA2;
  TCCR1B = 0x12;
  TCCR1C = 0x00;
  ICR1 = 20000; // top

  // initialize the motors to off
  OCR1A = 1500; // right motor
  OCR1B = 1500; // left motor
  
  // enable the PWM output on PB1 and PB2
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

// dynamic range 
float pmax = 55;
float rmax = 55;

// parameters for the 
#define TURN_THRESHOLD      15
#define TURN_CURVE          4.0
#define THROTTLE_THRESHOLD  10
#define THROTTLE_MAX        55
#define THROTTLE_STOP       75
#define THROTTLE_TOP        550
#define THROTTLE_CURVE      0.75
#define TIMEOUT             500


void loop()
{  
  float p, r, pr[2]; // pitch roll values

  float cap;
  int speed;
  float lpercent, rpercent;
  int diff, rdiff, ldiff;
  
  if (radio.available()) {

    while(radio.available()) {
      radio.read(pr, sizeof(pr));
    }

    p = pr[0]; // pitch (throttle)
    r = pr[1]; // roll  (steering)

    speed = abs(p);

    if (speed > pmax)
      pmax = speed;
    if (abs(r) > rmax)
      rmax = abs(r);

    /** speed mapping
     * There are 3 regions:
     *  - if the pitch is above a top threshold the maximum speed is assigned.
     *  - if the pitch is below a bottom threshold the speed is zero.
     *  - otherwise, the speed is non-linearly mapped to somewhere in between.
     */
    if (speed > THROTTLE_MAX)
      diff = THROTTLE_TOP;
    else if (speed > THROTTLE_THRESHOLD)
      diff = polymap(speed, THROTTLE_THRESHOLD, THROTTLE_MAX, THROTTLE_STOP, THROTTLE_TOP, THROTTLE_CURVE);
    else
      diff = map(speed, 0, THROTTLE_THRESHOLD, 0, THROTTLE_STOP);

    /** turning logic
     * if the roll is above the threshold, in either direction,
     * a turn is detected
     *
     * The roll is converted non-linearly to a number from 0 to 100.
     * This number is then used to create a percentage for each wheel
     *  - the outer wheel is sped up, anywhere from 100% to 150% of original speed.
     *  - the inner wheel is slowed down, anywhere from 0% to 100% of original speed
     */
    if (r > TURN_THRESHOLD)
    { //left turn
      cap = polymap(r, TURN_THRESHOLD, rmax, 0.0, 100.0, TURN_CURVE);
      rpercent = (300.0 - cap) / 200.0;
      lpercent = (100.0 - cap) / 100.0;
    }
    else if (r < -TURN_THRESHOLD)
    { //right turn
      cap = polymap(r, -TURN_THRESHOLD, -rmax, 0.0, 100.0, TURN_CURVE);
      lpercent = (300.0 - cap) / 200.0;
      rpercent = (100.0 - cap) / 100.0;
    }
    else
    { // no turn
      // equal weight for each wheel
      lpercent = rpercent = 1.0;
    }

    // apportioning speed to each wheel based on turn
    rdiff = diff * rpercent;
    ldiff = diff * lpercent;

    // adjusting for speed limit on each wheel in case we went over
    if (rdiff > THROTTLE_TOP) {
      ldiff -= (rdiff - THROTTLE_TOP);
      rdiff = THROTTLE_TOP;
    } else if (ldiff > THROTTLE_TOP) {
      rdiff -= (ldiff - THROTTLE_TOP);
      ldiff = THROTTLE_TOP;
    }

    // invert the pulse differential from the mean if going in reverse
    if (p < 0) {
      rdiff = -rdiff;
      ldiff = -ldiff;
    }

    // update the motors
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

    // record the transmission time
    last_rx = millis();
  }
  // if no transmission received recently then stop moving
  else if (millis() - last_rx > TIMEOUT)
  {
    OCR1A = OCR1B = 1500; // stop motors
    last_rx = millis();
  }
}

// provides a non-linear mapping (if curve != 1)
static inline float polymap(float x, float imn, float imx, float omn, float omx, float curve) {
  return omn + (omx - omn) * pow((x - imn) / (imx - imn), curve);
}

