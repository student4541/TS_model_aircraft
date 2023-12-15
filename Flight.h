#pragma once

#include <iostream>
#include <string>
#include "Station.h"
#include "Aircraft.h"

class Flight {
private:

	std::string fltNumber;
	unsigned fltID;

	Station* depStation;
	Station* arrStation;

	std::string depTime;
	std::string arrTime;
	int duration;


	Aircraft* aircraft;

public:
	Flight(unsigned id, std::string fnum, Station* depS, Station* arrS, std::string depT, std::string arrT, Aircraft* ac, int dura) :
		fltID(id),
		fltNumber(fnum),
		depStation(depS),
		arrStation(arrS),
		depTime(depT),
		arrTime(arrT),
		aircraft(ac),
		duration(dura)
	{}

	bool operator==(const Flight& flt) const {
		if (this->fltID == flt.fltID) 
			return true;
		else
			return false;
	}

	void setID(int id) { fltID = id; }
	unsigned getID() const { return fltID; }

	std::string& getFlightNum() { return fltNumber; }

	Station* getArrStation() const { return arrStation; }
	Station* getDepStation() const { return depStation; }

	std::string getArrTime() const { return arrTime; }
	std::string getDepTime() const { return depTime; }
	int getDuration() const { return duration; }

	Aircraft* getAircraft() const { return aircraft; }
	
	
};

class Leg {
private:
	Flight* flt;
	std::string fltNumber;

	std::string depTime;
	std::string arrTime;

	Station* depStation;
	Station* arrStation;

	Aircraft* aircraft;

	int legID;
	int duration;

public:
	Leg(std::string fN, std::string _depTime, std::string _arrTime, Station* dS, Station* aS, int d, int id) :
		flt(nullptr),
		fltNumber(fN),
		depTime(_depTime),
		arrTime(_arrTime),
		depStation(dS),
		arrStation(aS),
		duration(d),
		legID(id), 
		aircraft(nullptr)
	{}
	bool operator==(const Leg& leg) const {
		return (this->getFltID() == leg.getFltID() && this->getDepTime() == leg.getDepTime());
	}

	Flight* getFlight() const { return flt; }
	long getFltID() const { return flt->getID(); }
	std::string& getFlightNum() { return fltNumber; }

	Station* getArrStation() const { return arrStation; }
	Station* getDepStation() const { return depStation; }

	std::string getArrTime() const { return arrTime; }
	std::string getDepTime() const { return depTime; }

	int getDuration() const { return duration; }
	int getID() const { return legID; }
	Aircraft* getAircraft() const { return  flt->getAircraft(); }
	

};