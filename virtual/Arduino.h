#ifndef VIRTUAL_ARDUINO_H
#define VIRTUAL_ARDUINO_H

#define NULL 0
#define TWCR 0
#define TCCR0B 0
#define TCCR1B 0
#define TCCR2B 0

typedef unsigned char byte;

#include "Serial.h"
#include "Servo.h"
#include "HardwareSerial.h"
#include "WString.h"

unsigned long micros();
unsigned long millis();
void digitalWrite(int pin, int value);
void analogWrite(int pin, int value);
void pinMode(int pin, int value);
int digitalRead(int pin);
int analogRead(int pin);

#endif VIRTUAL_ARDUINO_H

