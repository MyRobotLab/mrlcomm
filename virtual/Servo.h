/*
 * Servo.h
 *
 *  Created on: Nov 3, 2016
 *      Author: gperry
 */

#ifndef VIRTUAL_SERVO_H_
#define VIRTUAL_SERVO_H_

class Servo {
public:
	Servo(){};
	virtual ~Servo(){};
	void detach(){};
	void write(int){};
	void attach(int){};
	void writeMicroseconds(int){};
};

#endif /* VIRTUAL_SERVO_H_ */
