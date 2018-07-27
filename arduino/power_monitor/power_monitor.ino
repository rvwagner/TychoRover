#include "wiring_private.h"
#include "pins_arduino.h"

uint8_t analog_reference = DEFAULT;
float refreshRate = 10; // Rate is in Hz
float vsd[6];
float current[6];
void setup() {
  
  int gnd0 = analogReadSlow(2);
  int vcc0 = analogReadSlow(3);
  int gnd1 = analogReadSlow(2);

  Serial.begin(9600);
  Serial.print("Ground-0 = ");
  Serial.println(gnd0, DEC);
  Serial.print("Vcc-0 = ");
  Serial.println(vcc0, DEC);
  Serial.print("Ground-1 = ");
  Serial.println(gnd1, DEC);

}

void loop() {
  unsigned long startTime = millis(); // Get the start time for refresh rate calc
  // Read ground, a0, and a1, and generate adc-voltage slopes and y-intercepts
  int gnd = analogReadSlow(2);  // Read ground to get x0 for first curve
  int a0 = analogReadSlow(0); // 2.048V
  int a1 = analogReadSlow(1); // 4.096V
  float m0 = (2.048/(a0-gnd)); // First slope from 0 to 2.048V
  float m1 = (2.048/(a1-a0)); // Second slope from 2.048 to 4.096V
  float b0 = (-m0*gnd); // Y-int for first slope, no +y because y is zero
  float b1 = (-m1*a0)+2.048; // Y-int for second slope

  // Read a4-a9 and compare to adc-voltage slopes and voltage divider ratios
  for (int i = 0; i <= 5; i++)
  {
    float v = analogReadSlow(i+4);
    if (v <= a0)
    {
      v = (m0*v)+b0;
    }
    else
    {
      v = (m1*v)+b1;
    }
    vsd[i] = v;
  }
  vsd[0] = vsd[0]/0.0910;
  vsd[1] = vsd[1]/0.0911;
  vsd[2] = vsd[2]/0.0909;
  vsd[3] = vsd[3]/0.0911;
  vsd[4] = vsd[4]/0.0910;
  vsd[5] = vsd[5]/0.0910;
  
  //read a10-a15 and calculate / return current value
  for (int i = 0; i <= 5; i++)
  {
    float vout = analogReadSlow(i+10);
    float vcc = vsd[4];
    if (vout <= a0)
    {
      vout = (m0*vout)+b0;
    }
    else
    {
      vout = (m1*vout)+b1;
    }
    current[i] = 73.3*(vout/vcc)-36.7; // Equation from Pololu: i = 73.3A * (vout/vcc) - 36.7A
  }
  printValues();
  if (millis()-startTime < 1000/refreshRate)
  {
    delay((1000/refreshRate)-(millis()-startTime));
  }
}

void printValues()
{
  for (int i = 0; i <= 5; i++)
  {
     Serial.print("Pin ");
     Serial.print(i+4, DEC);
     Serial.print(" Voltage is: ");
     Serial.print(vsd[i], DEC);
     Serial.println(" Volts.");
  }
  for (int i = 0; i <= 5; i++)
  {
     Serial.print("Pin ");
     Serial.print(i+10, DEC);
     Serial.print(" Current is: ");
     Serial.print(current[i], DEC);
     Serial.println(" Amps.");
  }
}

int analogReadSlow(uint8_t pin)
{
  uint8_t low, high;

#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
  if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#endif
  pin = analogPinToChannel(pin);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
  if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
  if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#else
  if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif

#if defined(ADCSRB) && defined(MUX5)
  // the MUX5 bit of ADCSRB selects whether we're reading from channels
  // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif

  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
  // to 0 (the default).
#if defined(ADMUX)
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = (analog_reference << 4) | (pin & 0x07);
#else
  ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif
#endif

  // without a delay, we seem to read from the wrong channel
  //delay(1);

  // delay 2 microseconds
  delayMicroseconds(2);

#if defined(ADCSRA) && defined(ADCL)
  // start the conversion
  sbi(ADCSRA, ADSC);

  // ADSC is cleared when the conversion finishes
  while (bit_is_set(ADCSRA, ADSC));

  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low  = ADCL;
  high = ADCH;
#else
  // we dont have an ADC, return 0
  low  = 0;
  high = 0;
#endif

  // combine the two bytes
  return (high << 8) | low;
}
