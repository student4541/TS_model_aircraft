#pragma once

#include "Flight.h"
#include "Station.h"
#include <vector>

class Product {
private:
	Station* orign;
	Station* destination;

	double fare;
	double averageDemand;
	int productID;
public:
	std::vector<std::string > fltNums;

	Product(Station* ori, Station* des, double f, double d):
		orign(ori),
		destination(des),
		fare(f),
		averageDemand(d),
		productID(0)
	{}

	Station* getOrigin() { return orign; }
	Station* getDestination() { return destination; }
	double getFare() { return fare; }
	const std::vector<std::string>& getFltNums() const { return fltNums; }
	double getDemand() { return averageDemand; }
	void setID(int id) { productID = id; }
	int getID() { return productID; }

	void addFlt(std::string fnum) { fltNums.push_back(fnum); }

};