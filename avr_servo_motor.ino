#define F_CPU 16000000UL
#define POT_DDR DDRC
#define POT_IP PINC //for reading potentiometer output
#define POT_OP PORTC

#define LED_DDR DDRD
#define LED_IP PIND //for adjusting LED output
#define LED_OP PORTD //for adjusting LED output

#include<avr/io.h>
#include<util/delay.h>
#include<stdint.h>
#include<avr/interrupt.h>


volatile long unsigned elapsed_ct = 0;


ISR(TIMER0_OVF_vect) {
  /*Ticking clock every overflow*/
  elapsed_ct ++; //1 count is roughly 1 milliseconds with current scalling
}

static inline void initADC0(void) {
  ADMUX |= (1 << REFS0); //Reference voltage on AVCC
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // ADC clock prescaler 1/8 (basically a numeric 3)
  ADCSRA |= (1 << ADEN); //Enable ADC
}

static inline void initTimer0(void) {
  TCCR0B |= (1 << CS01) | (1 << CS00) ; // /64 clock scaler 
  TIMSK0 |= (1 << TOIE0); //enable overflow interrupt

  //Clock for LED
  TCCR2A |= (1 << WGM20) | (1 << WGM21); // Fast PWM mode
  TCCR2B |= (1 << CS22) | (1 << CS20); // PMW Freq = F_CPU/ 128/ 256
  TCCR2A |= (1 << COM2B1); //non-inverting mode, output to OC2B = PD3
  

  TCNT0 = 0; //init time counter to 0
}

static inline void initTimer1Servo(void) {
  //Enable fast PWM, bit setting spreads across 2 registers
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1<<WGM13);

  TCCR1B |= (1 << CS11); // /8 scaler, count 2 ticks per microsecond
  ICR1 = 40000; //make TOP value 20ms
  TCCR1A |= (1 << COM1A1); //output to OC1A = PB1 
  DDRB |= (1 << PB1); //set pin 1 to output
}

int main(void) {

  const uint8_t servoPin = (1 << PD5);
  uint8_t LEDPin= (1 << PD3);
  uint8_t LED13 = (1 << PB5);
  uint8_t potPin = (1 << PC0);
  uint8_t brightness = 0;
  const uint16_t blinkInterval = 200;
  
  DDRB = LED13; //set built-in LED as output

  initADC0();
  initTimer0();
  initTimer1Servo();
  uint32_t blinkTimer = elapsed_ct;
  sei();

  while(1) {
    ADCSRA |= (1 << ADSC); //start ADC conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint16_t adcVal = ADC; //read in ADC val, from 0-1023

    //Code to make sure LED goes from completely off
    if (adcVal <= 100) LED_DDR &= ~LEDPin;
    else LED_DDR = LEDPin; 
    
    uint8_t LEDVal = (ADC >> 2);  
    OCR2B = LEDVal; //set toggle rate 
    OCR1A = map(adcVal, 0, 1023, 1400, 4400); //change angle of motor by sending a pulse between 1ms~2ms long
    if (adcVal > (1023/2)) {
      if (elapsed_ct - blinkTimer >= blinkInterval) {
        PORTB ^= LED13; // flip LED13 to blink
        blinkTimer = elapsed_ct;
      }
    }
    else PORTB &= ~LED13;
  }

  return 0;
}
