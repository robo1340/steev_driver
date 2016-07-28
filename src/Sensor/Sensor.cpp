#include "steev_driver/Sensor.h"

#include <string>

Sensor::Sensor(const std::string id) {
	_id = id;
	_val = -1;
	_refreshRate = -1;
	_enabled = true;
	_streamEnabled = false;
}

void Sensor::setValue(int val){_val = val;}
int Sensor::getValue() {return _val;}

std::string Sensor::getID(){
	return _id;
}
void Sensor::setRefreshRate(int rate) {_refreshRate = rate;}
int Sensor::getRefreshRate() {return _refreshRate;}

void Sensor::setEnabled(bool en) {_enabled = en;}
bool Sensor::isEnabled() {return _enabled;}

bool Sensor::isStreamEnabled() {return _streamEnabled;}
void Sensor::enableStream(bool b) {_streamEnabled = b;}
