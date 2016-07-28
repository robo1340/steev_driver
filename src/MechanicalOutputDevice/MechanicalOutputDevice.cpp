#include <string>
#include "steev_driver/MechanicalOutputDevice.h"

MechanicalOutputDevice::MechanicalOutputDevice(std::string id){
	_id = id;
	_val = 0;
	_incrementAmount = -1;
}

std::string MechanicalOutputDevice::getID(){return _id;}

int MechanicalOutputDevice::getValue() {return _val;}
void MechanicalOutputDevice::setValue(int val){_val = val;}

void MechanicalOutputDevice::setIncrementAmount(int incrementAmount) {	_incrementAmount = incrementAmount;}
int MechanicalOutputDevice::getIncrementAmount() {return _incrementAmount;}
