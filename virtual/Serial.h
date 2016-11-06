/*
 * Serial.h
 *
 *  Created on: Nov 3, 2016
 *      Author: gperry
 */

#ifndef VIRTUAL_SERIAL_H_
#define VIRTUAL_SERIAL_H_

class VSerial {
public:
	VSerial();
	virtual ~VSerial();
};

extern VSerial Serial;

#endif /* VIRTUAL_SERIAL_H_ */
