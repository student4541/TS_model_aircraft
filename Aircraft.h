#pragma once

#include <vector>
#include<string>

class Station;
class Flight;

class Aircraft {
private:
	std::string tailNumber;
	unsigned acID;

	int cost;
	int capacity;
	int numAircrafts;

public:
	std::vector<Flight*> scheduledFlights;

	Aircraft(std::string tn, int c, int cap, int num):
		tailNumber(tn),
		acID(std::hash<std::string>{}(tn)),
		cost(c),
		capacity(cap),
		numAircrafts(num)
	{}

	void addScheduledFlight(Flight* leg) { scheduledFlights.push_back(leg); }
	void setID(unsigned i) { acID = i; }

	unsigned getID() const { return acID; }
	const std::string& getTail() const { return tailNumber; }
	int getNumAircrafts() const { return numAircrafts; }

	std::vector<Flight*> getScheduledFlights() const { return scheduledFlights; }
	int getNumberFlights() const { return scheduledFlights.size(); }


	int getCost() const { return cost; }
	int getCapacity() const { return capacity; }
};