void setup() {
  // put your setup code here, to run once:
  TCCR1A = 0xA2;
  TCCR1B = 0x12;
  TCCR1C = 0x00;
  ICR1 = 20000;
  OCR1A = 1850; //right
  OCR1B = 1725; //left

  DDRB = 0xff;
}

void loop() {
  // put your main code here, to run repeatedly:

}
