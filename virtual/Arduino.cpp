#include "Arduino.h"

unsigned long micros(){return 0;}
unsigned long millis(){return 0;}
void digitalWrite(int pin, int value){};
void analogWrite(int pin, int value){};
void pinMode(int pin, int value){};
int digitalRead(int pin){return 0;};
int analogRead(int pin){return 0;};
void delay(int){};

uint8_t digitalPinToBitMask(int){return 0;};

void cli(){};
void sei(){};
bool bitRead(unsigned char, unsigned char){return 0;};
unsigned int random(unsigned int){return 0;};

int __brkval=0;
int __heap_start=0;