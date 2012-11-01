#include "WProgram.h"
#include "Timer.h"
#include "TWIMaster.h"
//#include <avr/sfr_defs.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define PWM_FREQUENCY 300   // in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

#define LEDPIN 13
#define LED2PIN 4
#define LED3PIN 31

#define USB_BAUD 57600
#define USB_SERIAL Serial

//#define TWI

#define PORT_TWI PORTD
#define PIN_TWI PIND
#define DDR_TWI DDRD
#define SCL PD0
#define SDA PD1


void print_timer(Timer &t, uint16_t TCNTn);

Timer *timers[] = {&Timer1, &Timer3, &Timer4, &Timer5};
enum ADC_Instruments { Thermocouple = 0, Current = 1, Voltage = 2 };
// use pins ADC0, 1, ..., ADC{ADC_Count}
const char ADC_Count = 3;
// how much of each successive measurement should be included
const float ADC_exponential_filter_factor = 0.01;
volatile float _adc[ADC_Count];
volatile char _curADC = 0;

inline void start_ADC_conversion()
{
  ADMUX = (1<<REFS1) | // 1.1V voltage reference
          (1<<ADLAR) | // MSB of value is MSB of ADCH
          _curADC;     // for values 0-7, read the corresponding ADC pin
  
  // initiate the next measurement
  sbi(ADCSRA, ADSC);
}

void initialize_ADC()
{
  // 10-bit resolution "requires an input clock frequency between 50 kHz and 200 kHz";
  // 16 MHz / 128 == 125 kHz
  ADCSRA = (1<<ADEN) |     // enable ADC
           (1<<ADIE) |     // enable interrupts
           (0b111<<ADPS0); // ADC clk = XTAL clk / 128

  // disable digital buffers on ADC pins we're using, assuming we start from ADC0
  unsigned char didr0 = 0;
  
  for (char i = 0; i < ADC_Count; i++)
    didr0 = (didr0 << 1) | 1;
    
  DIDR0 = didr0;
  
  _curADC = 0;
  start_ADC_conversion();  
}

// cycle through the ADCs we are measuring
ISR(ADC_vect) {
  // the ADC>>6 assumes:
  // 1. ADLAR is set in ADMUX
  // 2. note that if differential measurements are made, we can expect only 
  //    8-bit resolution (1x and 10x) or 7-bit resolution (200x), meaning we'd
  //    only need to read from ADCH
  _adc[_curADC] = ADC_exponential_filter_factor  * (float)((unsigned int)ADC>>6) +
             (1 - ADC_exponential_filter_factor) * _adc[_curADC];
  _curADC = (_curADC + 1) % ADC_Count;

  start_ADC_conversion();  
} 

void setup()
{
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  pinMode(LED2PIN, OUTPUT);
  digitalWrite(LED2PIN, LOW);
  pinMode(LED3PIN, OUTPUT);
  digitalWrite(LED3PIN, LOW);

  // debugging
  PORTA = 0;
  DDRA = 0xFF;

  USB_SERIAL.begin(USB_BAUD);
  USB_SERIAL.println("MultiTimer");

//  sbi(DDRB, PB5);
//  sbi(DDRB, PB6);
//  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
//  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);                 // Prescaler set to 8, that gives us a resolution of 0.5us
//  ICR1 = PWM_COUNTER_PERIOD;                               // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.
//  OCR1A = 1000 * 2;
//  OCR1B = 1000 * 2;

//  cbi(PORTD, PD0);
//  cbi(PORTD, PD1);
//  sbi(DDRD, PD0);
//  sbi(DDRD, PD1);
  
  i2c_master_initialize();

  // SDA/SCL pullups
  sbi(PORT_TWI, SCL);
  sbi(PORT_TWI, SDA);

  initialize_ADC();  
  
  Timer1.init_for_capture();
  Timer3.init_for_capture();
  Timer4.init_for_pulses();
  Timer5.init_for_capture();
  
  DDRA = 0xFF;
}

void printHex(HardwareSerial &ser, uint8_t n)
{
  if (n < 0x10)
    ser.write('0');
  
  ser.print(n, HEX);
}

char read_b(HardwareSerial &ser)
{
  while (!ser.available())
  {}
  
  return ser.read();
}

bool is_digit(char c) {
  return '0' <= c && c <= '9';
}

void print_timer(Timer &t, uint16_t TCNTn) {
  switch (t.mode) {
  case Timer::Pulses: {
    cli();
    uint16_t pulses_hiword = t.pulses_hiword;
    sei();
    uint32_t pulses = ((uint32_t)pulses_hiword << 16) + TCNTn;
    USB_SERIAL.print(pulses - t.last_pulses);
    t.last_pulses = pulses;
    break;
    }
  case Timer::Capture: {
    cli();
    uint32_t high_time = t.high_time;
    sei();
    USB_SERIAL.print((high_time - t.last_high_time) / 2); // clock prescalar is /8; we're at 16MHz
    t.last_high_time = high_time;
    break;
    }
  case Timer::None:
    break;
  }
  USB_SERIAL.print(" ");
}

char hexc2int(char c)
{
  if ('0' <= c && c <= '9')
    return c - '0';
  else if ('a' <= c && c <= 'f')
    return c - 'a' + 10;
  else if ('A' <= c && c <= 'F')
    return c - 'A' + 10;
  else
    return -1;
}

// consumes all hexadecimal characters and the first non-hexadecimal character, returning the
// 16-bit value defined by the last >= 4 characters read
uint16_t read_hex16_b(HardwareSerial &ser)
{
  uint16_t v = 0;

  // the while (true) version makes it easier to eat the very next serial character, which is
  // desirable for simple debugging purposes
  //for (uint8_t i = 4; i > 0; i--)
  while (true)
  {
    uint8_t c = read_b(ser);
    char n = hexc2int(c);
    
    if (n >= 0)
      v = (v * 16) + n;
    else
      break;
  }
  
  return v;    
}

uint16_t read_u16_b(HardwareSerial &ser)
{
  uint16_t v = 0;

  while (true)
  {
    while (!ser.available())
      ;
    
    uint8_t c = ser.read();
    
    if ('0' <= c && c <= '9')
      v = (v * 10) + (c - '0');
    else
      break;
  }
  
  return v;    
}

// consumes all hexadecimal characters and the first non-hexadecimal character, returning the
// 8-bit value defined by the last >= 2 characters read
uint8_t read_hex8_b(HardwareSerial &ser)
{
  uint8_t v = 0;

  // the while (true) version makes it easier to eat the very next serial character, which is
  // desirable for simple debugging purposes
  //for (uint8_t i = 2; i > 0; i--)
  while (true)
  {
    uint8_t c = read_b(ser);
    char n = hexc2int(c);
    
    if (n >= 0)
      v = (v * 16) + n;
    else
      break;
  }
  
  return v;    
}

//uint32_t _last_n3 = 0;
//uint32_t _last_n4 = 0;
//uint32_t _last_n5 = 0;

const float _alpha = 25.0 / (400 * 6 * 6 / 60) / 2.0 * 4; 
const int _fullScale = 1024;
const float _vref = /*1.1*/ 1.084;
const float _resistanceCurrent = /*284.5*/ 300.8 / _fullScale * _vref / 2; // last /2 is 1/2 coil tap
//const float _voltageDivision = 23.79;
const float _voltageConversion = /*0.0251136*/ 0.02501;

#define TWI_8BIT(v) ((uint8_t)((v) >> 3))

uint16_t _twi = 0;
bool _new_twi_mode = true;
static const uint8_t TWI_ADDR_BASE = 0x50;
uint8_t _twi_addr = TWI_ADDR_BASE; //(TWI_ADDR_BASE >> 1) + 1;
const uint8_t MAXCOUNT = 8;
volatile state_t *_p = NOSTATE;
volatile uint8_t i2c_calls = 0;

#define TWI_STATE_OK(state) ((state & (1<<STATE_SUCCESS_BIT)) != 0)

void print_twi_error() {
  USB_SERIAL.print("TWI ERROR -> ");
  printHex(USB_SERIAL, _twi_addr);
  USB_SERIAL.print(": ");
  printHex(USB_SERIAL, _p->state);
  USB_SERIAL.print(" : ");
  printHex(USB_SERIAL, PIND & 0x3);
  USB_SERIAL.println();
}

void twi_donefunc(state_t *p) {
  sbi(PORTA, PA3);
  _p = p;
  i2c_calls++;
  cbi(PORTA, PA3);
  
//  if (!TWI_STATE_OK(p->state))
//    print_twi_error();
}

bool wait_for_twi(char* p, uint8_t count) {
  i2c_calls = 0;

  twiQ.enqueue_w(_twi_addr, p, count, twi_donefunc);
  
  // blink Arduino LED
  uint16_t loops = 0;
  while (i2c_calls == 0 && ++loops != 0)
    sbi(PINB, PB7);
  
  if (loops == 0) {
    i2c_master_initialize();
    _p->state = 1<<(STATE_SUCCESS_BIT+1);
  }
  
  return TWI_STATE_OK(_p->state);
}

bool wait_for_twi_r(char* p, uint8_t count) {
  i2c_calls = 0;

  twiQ.enqueue_r(_twi_addr, p, count, twi_donefunc);
  
  // blink Arduino LED
  uint16_t loops = 0;
  while (i2c_calls == 0 && ++loops != 0)
    sbi(PINB, PB7);
  
  if (loops == 0) {
    i2c_master_initialize();
    _p->state = 1<<(STATE_SUCCESS_BIT+1);
  }
  
  return TWI_STATE_OK(_p->state);
}

bool twi_write_ib(uint16_t mem_addr, uint8_t* write, uint8_t count)
{
  count = count < MAXCOUNT ? count : MAXCOUNT;

  char p[MAXCOUNT+2];
  p[0] = mem_addr & 0xFF;
  p[1] = mem_addr >> 8;
  
  for (int i = 0; i < count; i++)
    p[i+2] = write[i];
  
  wait_for_twi(p, count+2);
}

bool twi_write_b(uint8_t* write, uint8_t count)
{
  count = count < MAXCOUNT ? count : MAXCOUNT;

  char p[MAXCOUNT];

  for (int i = 0; i < count; i++)
    p[i] = write[i];
  
  wait_for_twi(p, count);
}

uint8_t twi_read_ib_u8(uint16_t mem_addr)
{
  i2c_calls = 0;

  char p1[] = { mem_addr & 0xFF, mem_addr >> 8 };
  char p2[1];
  
  if (!wait_for_twi(p1, sizeof(p1)) ||
      !wait_for_twi_r(p2, sizeof(p2)))
    return 0;
  
  return p2[0];    
}

bool change_direction(bool reverse) {
  if (!_new_twi_mode) {
    uint8_t p[] = { TWI_8BIT(_twi), reverse ? 1<<7 : 0 };
    return twi_write_b(p, sizeof(p));
  } else {
    uint8_t p[] = { reverse ? 1<<7 : 0 };
    return twi_write_ib(0x0011, p, sizeof(p));
  }
}

bool send_twi(uint16_t v) {
  if (!_new_twi_mode) {
    uint8_t p[] = { TWI_8BIT(v) };
    return twi_write_b(p, sizeof(p));
  } else {
    uint8_t p[] = { v & 0x7, TWI_8BIT(v) };
    return twi_write_ib(0x0006, p, sizeof(p)) &&
           twi_write_ib(0xFFF8, NULL, 0);
  }
}

void loop() {
  static uint32_t _last_n3_delta = 0;
  static uint32_t _last_n4_delta = 0;

  static unsigned long _last;
  static unsigned long _print;
  static unsigned long _lastDelta;
  static unsigned long _delta = 0;
  static uint8_t _iter = 0;
  static bool _multicast = false;
  static uint16_t _matchPseudoRPM = 0;
  static bool _firstMatchPseudoRPM = true;
  static bool firstRun = true;
  
  if (firstRun) {
    _last = _print = _lastDelta = micros();
    firstRun = false;
  }
  
  if (USB_SERIAL.available()) {
    char c = USB_SERIAL.peek();
    
    if (is_digit(c)) {
      uint16_t n = read_u16_b(USB_SERIAL);
      
      if (1 <= n && n <= 10) {
        _twi = n == 1
          ? 1
          : n * 25*8;
      
        USB_SERIAL.print("Setting TWI = ");
        USB_SERIAL.println(_twi);
      } else if (11 <= n && n <= 1000) {
        _twi = n * 2;
        USB_SERIAL.print("Setting TWI = ");
        USB_SERIAL.println(_twi);
      } else if (1000 < n && n <= 20000) {
        USB_SERIAL.print("Matching B pseudo-rpm to ");
        USB_SERIAL.println(n);
        _matchPseudoRPM = n;
        _firstMatchPseudoRPM = true;
      } else {
        USB_SERIAL.println("Stopping motor.");
        _matchPseudoRPM = 0;
        _twi = 0;
      }
    } else if (c == 'r' || c == 'f') {
      USB_SERIAL.read();
      USB_SERIAL.println(c == 'r'
        ? "reverse rotation (CCW)"
        : "forward rotation (CW)");
      
      if (!change_direction(c == 'r'))
        print_twi_error();
    } else if (c == 'o' || c == 'n') {
      USB_SERIAL.read();
      USB_SERIAL.print("changing to ");
      USB_SERIAL.print(c == 'n' ? "new" : "old");
      USB_SERIAL.println(" TWI mode");

      _new_twi_mode = c == 'n';
    } else if (c == '*') {
      USB_SERIAL.read();
      _multicast = !_multicast;
      USB_SERIAL.print("multicast = ");
      USB_SERIAL.println(_multicast ? "true" : "false");
    } else if (c == 'a') {
      USB_SERIAL.read();
      
      _twi_addr = read_hex8_b(USB_SERIAL);
      USB_SERIAL.print("changing TWI addr to 0x");
      USB_SERIAL.println(_twi_addr, HEX);
    } else if (c == 's') {
      USB_SERIAL.read();
      USB_SERIAL.println("scanning 0x50-0x57");
      
      uint8_t save = _twi_addr;
      bool twi_was_valid = TWI_STATE_OK(_p->state);
      
      for (_twi_addr = TWI_ADDR_BASE; _twi_addr < TWI_ADDR_BASE + 8; _twi_addr++) {
        bool valid = send_twi(_twi);

        USB_SERIAL.print("addr 0x");
        printHex(USB_SERIAL, _twi_addr);
        USB_SERIAL.println(valid ? " valid" : "");

        if (valid && !twi_was_valid) {
          save = _twi_addr;
          twi_was_valid = true;
        }
      }
      
      _twi_addr = save;
    } else if (c == 'b') {
      USB_SERIAL.read();
      USB_SERIAL.println("beeping 0x50-0x57");
      
      uint8_t save = _twi_addr;
      
      uint16_t beep = 0xFFF9;
      
      for (_twi_addr = TWI_ADDR_BASE; _twi_addr < TWI_ADDR_BASE + 8; _twi_addr++) {
        bool valid = twi_write_ib(beep, NULL, 0);

        USB_SERIAL.print("addr 0x");
        printHex(USB_SERIAL, _twi_addr);
        USB_SERIAL.println(valid ? " valid" : "");
        
        if (valid && beep < 0xFFFC)
          beep++;
        
        delayMicroseconds(100);
      }
      
      _twi_addr = save;
    } else if (c == '\r') {
      USB_SERIAL.read();
    } else if (c == 't') {
      USB_SERIAL.read();
      
      char c = read_b(USB_SERIAL);
      
      USB_SERIAL.println(c);
      
      sbi(PORTA, PA4);
      //delay(1);
      
      uint8_t ddr_save = DDR_TWI;

      switch (c) {
      case 'x':
        TWCR = 1<< TWSTO;
        TWCR = 0;
        
        DDR_TWI |= (1<<SCL) | (1<<SDA);
        
        cbi(PORT_TWI, SCL);
        cbi(PORT_TWI, SDA);
        
        for (uint8_t i = 0; i < 30; i++) {
          delayMicroseconds(20);
          sbi(PIN_TWI, SCL);
        }

        delayMicroseconds(20);
        DDR_TWI = ddr_save;        
        //break;

      case '\r':
        TWCR = 0;
        i2c_master_initialize();
        break;
      case '0':
        TWDR = 0;
        break;
      case 'F':
        TWDR = 0xFF;
        break;
      case 'i':
        i2c_master_initialize();
        break;
      case 'C':
        TWCR = 0;
        i2c_master_initialize();
        break;
      }
      
      cbi(PORTA, PA4);
    } else if (c == 'x') {
      USB_SERIAL.read();
      PORTA = 0xFF;
      delay(1);
      PORTA = 0x00;
    } else {
      USB_SERIAL.print("reconfig");
      for (uint8_t i = 0; USB_SERIAL.available() && i < sizeof(timers); i++) {
        switch (USB_SERIAL.read()) {
        case 'p':
          timers[i]->init_for_pulses();
          break;
        case 'c':
          timers[i]->init_for_capture();
          break;
        default:
          timers[i]->stop();
        }
      }    
    }
  }

  unsigned long t = micros();  

  #ifdef TWI
  if (t - _last < 40000)
    return;

  _last = t;
  
  sbi(PORTA, PA2);
  if (_multicast) {
    _iter++;
    uint8_t save = _twi_addr;
    uint8_t toggle = 1<<PA4;

    for (_twi_addr = 0x50; _twi_addr <= 0x52; _twi_addr++) {
      PINA = toggle;
      if (!send_twi(_twi) && t - _print >= 1000000)
        print_twi_error();
      PINA = toggle;
      toggle <<= 1;
      
      if ((_iter & 0x0F) == 0) {
        if (twi_read_ib_u8(0x0088) == 0) {
          USB_SERIAL.print("got_started = 0 on 0x");
          printHex(USB_SERIAL, _twi_addr);
          USB_SERIAL.println("; attempting 0-send");
          
          for (int i = 0; i < 20; i++)
            send_twi(0);
          for (int i = 0; i < 20; i++)
            send_twi(_twi);
        }
      }
    }
    
    _twi_addr = save;
  } else {
    if (!send_twi(_twi) && t - _print >= 1000000)
      print_twi_error();
  }
  cbi(PORTA, PA2);
  #endif


  if (t - _lastDelta >= 200000)
  {
    _lastDelta += 200000; // 200ms

    cli();
    uint16_t t3 = TCNT3;
    uint16_t t4 = TCNT4;
    sei();
    
    uint32_t n3 = ((uint32_t)Timer3.pulses_hiword << 16) + t3;
    uint32_t n4 = ((uint32_t)Timer4.pulses_hiword << 16) + t4;
    
  
    if (_matchPseudoRPM > 0 && !_firstMatchPseudoRPM)
    {
      uint32_t d3 = _matchPseudoRPM > 0 
        ? _matchPseudoRPM / 5
        : n3 - _last_n3_delta;
      uint32_t d4 = n4 - _last_n4_delta;
      
      float adj = _alpha * (int16_t)(d3 - d4)*2;
      
      if (adj > 60)
        adj = 60;
      else if (adj < -60)
        adj = -60;
      else if (0.4 <= adj && adj < 1.0)
        adj = 1;
      else if (-1 < adj && adj <= -0.4)
        adj = -1;
      
      _twi += adj;
    }
    
    _firstMatchPseudoRPM = false;
    
    _last_n3_delta = n3;
    _last_n4_delta = n4;
  }
    

  if (t - _print >= 1000000)
  {
    _print += 1000000; // 1s
  
    cli();
    uint16_t t1 = TCNT1;
    uint16_t t3 = TCNT3;
    uint16_t t4 = TCNT4;
    uint16_t t5 = TCNT5;
    sei();
    
    float _vCurrent = _adc[Current] / _fullScale * _vref;
    
    USB_SERIAL.print(micros());
    USB_SERIAL.print("  ");
    print_timer(Timer1, t1);
    print_timer(Timer3, t3);
    print_timer(Timer4, t4);
    print_timer(Timer5, t5);
//    for (uint8_t i = 0; i < ADC_Count; i++)
//    {
//      USB_SERIAL.print(_adc[i]);
//      USB_SERIAL.print(" ");
//    }
//    USB_SERIAL.print(_adc[Thermocouple]);  
//    USB_SERIAL.print(" ");
    USB_SERIAL.print(_vCurrent / _resistanceCurrent);  
    USB_SERIAL.print(" ");
    USB_SERIAL.print(_adc[Voltage] * _voltageConversion - _vCurrent);  
//    USB_SERIAL.print(" ");
//    USB_SERIAL.print((OCR1A / 2) - 1000);  
    USB_SERIAL.print(" ");
    USB_SERIAL.print(_twi, DEC);  
    USB_SERIAL.print(" > 0x");
    printHex(USB_SERIAL, _twi_addr);
    USB_SERIAL.print(" : 0x");
    printHex(USB_SERIAL, _p->state);
    USB_SERIAL.print(" SCL ");
    USB_SERIAL.print(PIN_TWI & (1<<SCL) ? "1" : "0");
    USB_SERIAL.print(" SDA ");
    USB_SERIAL.print(PIN_TWI & (1<<SDA) ? "1" : "0");
    USB_SERIAL.println();
  }
}
