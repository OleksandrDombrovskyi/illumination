/*
 * Fast PWM mode. The counter counts from BOTTOM to TOP then restarts from BOTTOM.
 * 
 * 
 * What is the frequency of my timer?
 * 
 * The Arduino has a system clock of 16MHz and the timer clock frequency will be 
 * the system clock frequency divided by the prescale factor.
 * In our case prescale factor is 1 (TCCR1B |= 1 << CS10), thus one tick of the system clock 
 * corresponds to one tick of our Timer1.
 * We have TOP ticks per cycle. In the code ICR1 = MAX_PWM_VALUE, meaning TOP = MAX, or 65535.
 * Thus cycle frequency is 16MHz / 65535 = 244 Hz.
 * 
 * 
 * Timeline, scale is one timer cycle.
 * On the timeline TOP != MAX for illustrative purpose (in the code TOP = MAX).
 * 
 *    pin 10 is HIGH            <-->        pin 10 is LOW             unused
 * |------------------------------                                               |
 * |                              _____________________________                  |
 * |                                                           ..................|
 * |                             |                             |                 |
 * 0 (BOTTOM)              OCR1B (PWM variable)              ICR1 (TOP)          65535 (MAX)
 * |                                                           |
 * |<--------------------timer cycle-------------------------->|
 *
 */

#include "TOF10120.h"

TOF10120 sensor;

// constants
const byte BUILD_IN_LED_PIN = 13;
const byte MOVEMENT_SENSOR_PIN = 6;
const byte PWM_PIN = 10;            // OC1B

const int MIN_BRIGHTNESS = 40;
const int MAX_BRIGHTNESS = 80;

const byte TIMER_2_SUBROUTINE_DELAY = 100;  //value 100 means one second
const byte LIGHTING_TIMEOUT = 15;    //seconds, depends on RELOAD and TIMER_2_SUBROUTINE_DELAY value
const byte LIGHT_DIMMER_DELAY = 5;  //value inverse to velocity of dimming, depends on RELOAD

const byte RELOAD = 0x9C;           //Timer2 TOP value. 0x9C - once per 0.01sec. 0xFF - once per 0.015sec
const uint16_t MAX_PWM_VALUE = 65535U;


//state
volatile bool isOn = false;
volatile int brightness = 0;
volatile bool isMovement = false;

//increments
volatile byte timer2SubroutineInc = 0;
volatile byte lightDimmerSubroutineInc = 0;
volatile byte lightingTimeInc = 0;

//flashing state
bool isFlashOn = false;


//--------------------------------------------------------------------------------------------------
// Setup timer1 for 16 bit PWM
void initPWM() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  /*
   * WGM - Waveform Generation Mode (See Waveform Generation Mode Bit Description)
   * For fast PWM, WGM13, WGM12 and WGM11 are set (WGM10 is not set) 
   * and the ICR1 value determines the top of the counter.
   */
  TCCR1A |= 1 << WGM11;
  TCCR1B |= 1 << WGM12;
  TCCR1B |= 1 << WGM13;
  
  ICR1 = MAX_PWM_VALUE; // 16-bit Input Capture Register - TOP of the counter

  TCCR1B |= 1 << CS10;   // Clock Select. Mode: clk/1 (No prescaling) - mode 14 fast PWM

  /*
   * COM - Compare Output Mode for Channel.
   * The COM1A[1:0] and COM1B[1:0] control the Output Compare pins (OC1A and OC1B respectively)
   * behavior.
   * Here OC1A - pin 9, OC1B - pin 10.
   * 
   * When the WGM1[3:0] bits are set to the fast PWM mode,  COM1x[1:0] mean the following: 
   * clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode)
   */
  TCCR1A |= 1 << COM1A1; 
  TCCR1A |= 1 << COM1B1;
  
  interrupts();
  pinMode(PWM_PIN, OUTPUT);
}

//--------------------------------------------------------------------------------------------------
// Set a 16 bit PWM value for a channel
void setPWM(uint16_t pwm) {
  noInterrupts();
  /*
   * The double buffered Output Compare Registers (OCR1A/B) are compared with the 
   * Timer/Counter value at all time. The result of the compare can be used by the 
   * Waveform Generator to generate a PWM or variable frequency output on the 
   * Output Compare pin (OC1A/B). 
   * Here OC1A/B - pins 9/10 respectively.
   */
  OCR1B = pwm; // 16-bit Output Compare Register related to 10 pin -> PWM_PIN
  interrupts();
}

void initTimer2() {
  noInterrupts();
  OCR2A = RELOAD;
  TCCR2A = 1<<WGM21; //CTC mode
  TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20); // Clock Select. Prescaling: 1/1024
  TIMSK2 = (1<<OCIE2A);

  interrupts();
}

//--------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting...");
  initPWM();

  pinMode(BUILD_IN_LED_PIN, OUTPUT);
  digitalWrite(BUILD_IN_LED_PIN, LOW);
  initTimer2();

  //init lazer sensor
  sensor.initSensor();
}

//--------------------------------------------------------------------------------------------------
void loop() {
  handleTimer2Subroutine();

  handleMovement();
  
  handleTimeout();
  
  handleDimmerSubroutine();
}

void handleTimer2Subroutine() {
  if (timer2SubroutineInc >= TIMER_2_SUBROUTINE_DELAY) {
    timer2SubroutineInc = 0;
    timer2Subroutine();
  }
}

void handleMovement() {
  bool isMovementPiro = digitalRead(MOVEMENT_SENSOR_PIN) == HIGH;
  bool isMovementLaser = sensor.isMovement();
  isMovement = isMovementPiro || isMovementLaser;
  if (isMovement) {
    isOn = true;
  }
}

void handleTimeout() {
  if (lightingTimeInc > LIGHTING_TIMEOUT) {
    isOn = false;
  }
}

void handleDimmerSubroutine() {
  if (lightDimmerSubroutineInc >= LIGHT_DIMMER_DELAY) {
    lightDimmerSubroutineInc = 0;
    handleDimmer();
  }
}

void handleDimmer() {
  if (isOn) {
    if (brightness < MIN_BRIGHTNESS) {
      brightness = MIN_BRIGHTNESS;
    }
    if (brightness < MAX_BRIGHTNESS) {
      setPWM(brightness++);
    } else {
      setPWM(brightness);
    }
  } else {
    if (brightness > MIN_BRIGHTNESS) {
      setPWM(--brightness);
    } else {
      setPWM(0);
    }
  }
}

// is triggered once per second, depends on RELOAD and TIMER_2_SUBROUTINE_DELAY
void timer2Subroutine() {
  if (isOn && !isMovement) {
    lightingTimeInc++;
  } else {
    lightingTimeInc = 0;
  }
  
  // blink for test
  isFlashOn = !isFlashOn;
  digitalWrite(BUILD_IN_LED_PIN, isFlashOn);
}

// is triggered once per 0.01sec (hundred times per second) if RELOAD = 0x9C
ISR(TIMER2_COMPA_vect) {
  timer2SubroutineInc++;
  lightDimmerSubroutineInc++;
  sensor.clk();
  
  OCR2A = RELOAD;
}
