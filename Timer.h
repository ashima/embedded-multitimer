#include <wiring_private.h>

#ifndef TIMER_H
#define TIMER_H

class Timer {
private:
  volatile uint8_t *_TCCRnA;
  volatile uint8_t *_TCCRnB;
  volatile uint16_t *_TCNTn;
  volatile uint8_t *_TIMSKn;
  volatile uint8_t *_TIFRn;
public:
	uint16_t pulses_hiword;
	uint32_t last_pulses;
	uint32_t high_time;
	uint32_t last_high_time;
	uint8_t mode;
	enum TimingMode { None, Pulses, Capture };
	
	Timer(volatile uint8_t *TCCRnA, volatile uint8_t *TCCRnB,
        volatile uint16_t *TCNTn, volatile uint8_t *TIMSKn,
        volatile uint8_t *TIFRn) 
        : pulses_hiword(0), mode(None) {
    _TCCRnA = TCCRnA;
    _TCCRnB = TCCRnB;
    _TCNTn = TCNTn;
    _TIMSKn = TIMSKn;
    _TIFRn = TIFRn;
	}
	
	void init_for_pulses() {
    *_TCCRnB = 0;          // disable the timers while they are being configured
    *_TCCRnA = 0;          // WGMn3:0 = 0 means we count from 0 -> 0xFFFF, 0 -> 0xFFFF, ...
    *_TCNTn = 0;           // reset counters
    *_TIMSKn = (1<<TOIE5); // enable pulses_hiword interrupts only
    *_TIFRn = 0;           // clear any pending interrupts
    
    last_pulses = pulses_hiword = 0;
    
    // the Timer now counts up
    // clock on falling edge of external Tn signal if CSn0 = 0, rising edge if CSn0 = 1
    *_TCCRnB = (1<<CS50) | (1<<CS51) | (1<<CS52);
    
    mode = Pulses;
	}
	
	void init_for_capture() {
    *_TCCRnB = 0;          // disable the timers while they are being configured
    *_TCCRnA = 0;          // WGMn3:0 = 0 means we count from 0 -> 0xFFFF, 0 -> 0xFFFF, ...
    *_TIMSKn = (1<<ICIE5); // enable input high_time interrupts only
    *_TIFRn = 0;           // clear any pending interrupts

    high_time = last_high_time = 0;

    // the Timer now counts up
    // clock on falling edge of external Tn signal if CSn0 = 0, rising edge if CSn0 = 1
    *_TCCRnB = (1<<ICES5) | (1<<CS51);  // prescaler of /8
    
    mode = Capture;
	}
	
	void stop() {
	  *_TIMSKn = 0;
	  *_TCCRnB = 0;
	}
};

Timer Timer1 = Timer(&TCCR1A, &TCCR1B, &TCNT1, &TIMSK1, &TIFR1);
ISR(TIMER1_OVF_vect) { Timer1.pulses_hiword++; }
Timer Timer3 = Timer(&TCCR3A, &TCCR3B, &TCNT3, &TIMSK3, &TIFR3);
ISR(TIMER3_OVF_vect) { Timer3.pulses_hiword++; }
Timer Timer4 = Timer(&TCCR4A, &TCCR4B, &TCNT4, &TIMSK4, &TIFR4);
ISR(TIMER4_OVF_vect) { Timer4.pulses_hiword++; }
Timer Timer5 = Timer(&TCCR5A, &TCCR5B, &TCNT5, &TIMSK5, &TIFR5);
ISR(TIMER5_OVF_vect) { Timer5.pulses_hiword++; }

ISR(TIMER1_CAPT_vect) {
  static uint16_t up = 0;
  
  // if we are detecting rising edges
  if (TCCR1B & (1<<ICES1)) {
    up = ICR1;
    cbi(TCCR1B, ICES1);
  }
  else {
    int32_t diff = ICR1 - up;
    if (diff < 0)
      diff += 1<<16;
    Timer1.high_time += diff;
    sbi(TCCR1B, ICES1);
  }
}
ISR(TIMER3_CAPT_vect) {
  static uint16_t up = 0;
  
  // if we are detecting rising edges
  if (TCCR3B & (1<<ICES3)) {
    up = ICR3;
    cbi(TCCR3B, ICES3);
  }
  else {
    int32_t diff = ICR3 - up;
    if (diff < 0)
      diff += 1<<16;
    Timer3.high_time += diff;
    sbi(TCCR3B, ICES3);
  }
}
ISR(TIMER4_CAPT_vect) {
  static uint16_t up = 0;
  
  // if we are detecting rising edges
  if (TCCR4B & (1<<ICES4)) {
    up = ICR4;
    cbi(TCCR4B, ICES4);
  }
  else {
    int32_t diff = ICR4 - up;
    if (diff < 0)
      diff += 1<<16;
    Timer4.high_time += diff;
    sbi(TCCR4B, ICES4);
  }
}
ISR(TIMER5_CAPT_vect) {
  static uint16_t up = 0;
  
  // if we are detecting rising edges
  if (TCCR5B & (1<<ICES5)) {
    up = ICR5;
    cbi(TCCR5B, ICES5);
  }
  else {
    int32_t diff = ICR5 - up;
    if (diff < 0)
      diff += 1<<16;
    Timer5.high_time += diff;
    sbi(TCCR5B, ICES5);
  }
}
#endif