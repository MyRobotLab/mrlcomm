/*
 * Wire.h
 *
 *  Created on: Nov 3, 2016
 *      Author: gperry
 */

#ifndef VIRTUAL_WIRE_H_
#define VIRTUAL_WIRE_H_


class VWire {
public:
	VWire();
	virtual ~VWire();
	void begin();
	void setClock(long);
	void beginTransmission(unsigned char);
	void write(unsigned char);
	void endTransmission();
	int requestFrom(unsigned char, unsigned char);
	unsigned char read();
	
};

extern VWire Wire;

#endif /* VIRTUAL_WIRE_H_ */
