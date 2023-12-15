#pragma once

#include<string>
#include<vector>

class Leg;


class Station {
private:
	std::string code;
	int staID;

	std::vector<Leg*> depLegs;
	std::vector<Leg*> arrLegs;

public:
	Station(std::string c, int id) : code(c), staID(id) {};

	bool operator==(const Station& s) const { return (this->staID == s.staID); }

	std::string& getCode() { return code; }
	int getID() const { return staID; }
	
	//为什么不返回常引用？
	std::vector<Leg*> getDepLegs() const { return depLegs; }
	std::vector<Leg*> getArrLegs() const { return arrLegs; }

	void addDepLeg(Leg* l) { depLegs.push_back(l); }
	void addArrLeg(Leg* l) { arrLegs.push_back(l); }


};