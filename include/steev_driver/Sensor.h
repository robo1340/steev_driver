#ifndef SENSOR_H
#define SENSOR_H

#include <string>

class Sensor {

   public:
	Sensor(const std::string id);

	void setValue(int val);
	int getValue();

	std::string getID();

	void setRefreshRate(int rate);
	int getRefreshRate();

	void setEnabled(bool en);
	bool isEnabled();

	bool isStreamEnabled();
	void enableStream(bool b);

   private:

	std::string _id;
	int _val;
	int _refreshRate;
	bool _enabled;
	bool _streamEnabled;

};

#endif //SENSOR_H
