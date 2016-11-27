/*
 * HardwareSerial_h.h
 *
 *  Created on: Nov 3, 2016
 *      Author: gperry
 */

#ifndef VIRTUAL_HARDWARE_SERIAL_H_
#define VIRTUAL_HARDWARE_SERIAL_H_

class HardwareSerial {
public:
	HardwareSerial(){};
    virtual ~HardwareSerial(){};
    void begin(int){};
    void write(const unsigned char){};
    void write(const unsigned char*, int){};
    unsigned char read(){return 0;};
    int available(){return 0;};
    void end(){};
    void flush(){};
    operator bool() { return true; }
};


extern HardwareSerial Serial;

#endif /* VIRTUAL_HARDWARE_SERIAL_H_ */
