#include "Msg.h"
#include "Device.h"
#include "MrlUltrasonicSensor.h"

MrlUltrasonicSensor::MrlUltrasonicSensor(int deviceId) : Device(deviceId, DEVICE_TYPE_ULTRASONIC) {
	msg->publishDebug("ctor MrlUltrasonicSensor " + String(deviceId));
	timeoutUS=1000; //this need to be set
	trigPin=0;//this need to be set
	echoPin=0;//this need to be set
}

MrlUltrasonicSensor::~MrlUltrasonicSensor() {}

void MrlUltrasonicSensor::attach(byte trigPin, byte echoPin){
	msg->publishDebug("Ultrasonic.attach " + String(trigPin) + " " + String(echoPin));
	this->trigPin = trigPin;
	this->echoPin = echoPin;
}

void MrlUltrasonicSensor::startRanging(long timeout) {
	msg->publishDebug("Ultrasonic.startRanging " + String(timeout));
	isRanging = true;
	state = ECHO_STATE_START;
}

void MrlUltrasonicSensor::stopRanging() {
	msg->publishDebug(F("Ultrasonic.stopRanging"));
	isRanging = false;
}

void MrlUltrasonicSensor::update() {

	if (!isRanging){
		return;
	}

	msg->publishDebug("state " + String(state));

	++lastValue;
	msg->publishUltrasonicSensorData(id, lastValue);
	return;

	if (state == ECHO_STATE_START) {
		// trigPin prepare - start low for an
		// upcoming high pulse
		pinMode(trigPin, OUTPUT);
		digitalWrite(trigPin, LOW);
		// put the echopin into a high state
		// is this necessary ???
		pinMode(echoPin, OUTPUT);
		digitalWrite(echoPin, HIGH);
		unsigned long newts = micros();
		if (newts - ts > 2) {
			ts = newts;
			state = ECHO_STATE_TRIG_PULSE_BEGIN;
		}
	} else if (state == ECHO_STATE_TRIG_PULSE_BEGIN) {
		// begin high pulse for at least 10 us
		pinMode(trigPin, OUTPUT);
		digitalWrite(trigPin, HIGH);
		unsigned long newts = micros();
		if (newts - ts > 10) {
			ts = newts;
			state = ECHO_STATE_TRIG_PULSE_END;
		}
	} else if (state == ECHO_STATE_TRIG_PULSE_END) {
		// end of pulse
		pinMode(trigPin, OUTPUT);
		digitalWrite(trigPin, LOW);
		state = ECHO_STATE_MIN_PAUSE_PRE_LISTENING;
		ts = micros();
	} else if (state == ECHO_STATE_MIN_PAUSE_PRE_LISTENING) {
		unsigned long newts = micros();
		if (newts - ts > 1500) {
			ts = newts;
			// putting echo pin into listen mode
			pinMode(echoPin, OUTPUT);
			digitalWrite(echoPin, HIGH);
			pinMode(echoPin, INPUT);
			state = ECHO_STATE_LISTENING;
		}
	} else if (state == ECHO_STATE_LISTENING) {
		// timeout or change states..
		int value = digitalRead(echoPin);
		unsigned long newts = micros();
		if (value == LOW) {
			lastValue = newts - ts;
			ts = newts;
			state = ECHO_STATE_GOOD_RANGE;
		} else if (newts - ts > timeoutUS) {
			state = ECHO_STATE_TIMEOUT;
			ts = newts;
			lastValue = 0;
		}
	} else if (state == ECHO_STATE_GOOD_RANGE || state == ECHO_STATE_TIMEOUT) {
		msg->publishUltrasonicSensorData(id, lastValue);
		state = ECHO_STATE_START;
	} // end else if
}

