#ifndef MECHANICALOUTPUTDEVICE_H
#define MECHANICALOUTPUTDEVICE_H

#include<string>

class MechanicalOutputDevice {

	public:
	MechanicalOutputDevice(std::string id);

	std::string getID();

	int getValue();
	void setValue(int val);

	void setIncrementAmount(int incrementAmount);
	int getIncrementAmount();

	private:
	std::string _id;
	int _incrementAmount;
	int _val;
};

#endif //MECHANICALOUTPUTDEVICE_H
