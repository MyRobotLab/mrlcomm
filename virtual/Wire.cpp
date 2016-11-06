#include "Wire.h"

	VWire::VWire(){};
	VWire::~VWire(){};
	void VWire::begin(){};
	void VWire::setClock(long){};
	void VWire::beginTransmission(unsigned char){};
	void VWire::write(unsigned char){};
	void VWire::endTransmission(){};
	int VWire::requestFrom(unsigned char, unsigned char){return 0;};
	unsigned char VWire::read(){return 0;};


VWire Wire;