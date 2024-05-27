#include "DS3231Clock.h"

#include <DS3231.h>

DS3231  rtc(SDA, SCL);

//constants
const int READ_TIME_DELAY = 500; // once per 5 seconds
const int NIGHT_START = 22; // illumination is on from 11pm
const int NIGHT_END = 6; // illumination is on to 6am

//increments
volatile int readTimeDelayInc = 0;

//state
volatile int timeInHours = 0;

void DS3231Clock::initClock() {
    rtc.begin();
}

void DS3231Clock::clk() {
    if (readTimeDelayInc < READ_TIME_DELAY) {
        readTimeDelayInc++;
    }
}

boolean DS3231Clock::isNight() {
    if (readTimeDelayInc >= READ_TIME_DELAY) {
        readTimeDelayInc = 0;
        timeInHours = getIntHours();
        Serial.println(rtc.getTimeStr());
    }

    return timeInHours >= NIGHT_START || timeInHours < NIGHT_END;
}

void DS3231Clock::setTime(int hours) {
    rtc.setTime(hours, 0, 0);
}

int DS3231Clock::getIntHours() {
  char* s = rtc.getTimeStr();
  strtok(s,":");
  int timeInt = atoi(s);
  return timeInt;
}
