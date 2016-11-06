#ifndef VIRTUAL_ARDUINO_H
#define VIRTUAL_ARDUINO_H

#define NULL 0
#define TWCR 0
#define TCCR0B 0
#define TCCR1B 0
#define TCCR2B 0
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

typedef unsigned char byte;
typedef unsigned char uint8_t;

//#include "Serial.h"
#include <iostream>
#include "Servo.h"
#include "HardwareSerial.h"
#include "String.h"

unsigned long micros();
unsigned long millis();
void digitalWrite(int pin, int value);
void analogWrite(int pin, int value);
void pinMode(int pin, int value);
int digitalRead(int pin);
int analogRead(int pin);
void delay(int);

uint8_t digitalPinToBitMask(int);

#define PORTB 0
#define PORTC 0
#define PORTD 0
#define F_CPU 1


void cli();
void sei();
bool bitRead(unsigned char, unsigned char);
unsigned int random(unsigned int);



#endif /*VIRTUAL_ARDUINO_H*/

