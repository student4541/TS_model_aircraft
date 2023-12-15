#pragma once

#include "Aircraft.h"
#include "Flight.h"
#include "Station.h"

#include <vector>
#include <algorithm>

class Flight;
class Station;
class TS_Node;
class TS_Arc;

class TS_Node
{
private:
	std::string time;
	Station* station;

	int nodeID;

public:
	std::vector<int > enteringFlightArcs;
	std::vector<int > leavingFlightArcs;
	std::vector<int > enteringGroundArcs;
	std::vector<int > leavingGroundArcs;

	std::vector<TS_Arc* > leavingArcs;
	std::vector<TS_Arc* > enteringArcs;
	//前向标签与后向标签？
	double fwdLabel;
	TS_Arc* predecessor;

	double bwdLabel;
	TS_Arc* sucessor;

	TS_Node(const std::string _t, Station* _s) :
		station(_s),
		time(_t),
		nodeID(0),
		fwdLabel(0),
		bwdLabel(0),
		predecessor(nullptr),
		sucessor(nullptr)
	{
		enteringFlightArcs.clear();
		leavingFlightArcs.clear();
		enteringGroundArcs.clear();
		leavingGroundArcs.clear();
	}

	TS_Node() :
		station(nullptr),
		time("0000"),
		nodeID(0),
		fwdLabel(0),
		bwdLabel(0),
		predecessor(nullptr),
		sucessor(nullptr) 
	{}

	bool operator==(const TS_Node& nd)
	{
		if ((this->station == nd.station) && (this->time == nd.time) && (this->nodeID == nd.nodeID))
			return true;
		else
			return false;
	}

	int getID() const { return nodeID; }
	void setID(int i) { nodeID = i; }

	std::string getTime() const { return time; }
	Station* getStation() const { return station; }
};

class TS_Arc {
private:
	TS_Node* tail;
	TS_Node* head;
	Leg* leg;

public:
	TS_Arc(TS_Node* _tail, TS_Node* _head, Leg* l)
	{
		tail = _tail;
		head = _head;
		leg = l;
	}
	TS_Arc() :
		tail(nullptr),
		head(nullptr),
		leg(nullptr)
	{}

	TS_Node* getTailNode() const { return tail; }
	TS_Node* getHeadNode() const { return head; }
	void setTailNode(TS_Node* _tail) { tail = _tail; }
	void setHeadNode(TS_Node* _head) { head = _head; }

	std::string getStartTime() const { return tail->getTime(); }
	std::string getEndTime() const { return head->getTime(); }
	Station* getDepStation() const { return leg->getDepStation(); }
	Station* getArrStation() const { return leg->getArrStation(); }

	Leg* getLeg() const { return leg; }

	//微小的改动
	int getDuration() const { return leg->getDuration(); }
};

class TS_Network {
public:
	std::vector<TS_Node* > nodes;
	std::vector<TS_Arc*> flightArcs;
	std::vector<TS_Arc* > groundArcs;

	TS_Network()
	{
		nodes.clear(); flightArcs.clear(); groundArcs.clear();
	}

	void addNode(TS_Node* n)
	{
		if (std::find(nodes.begin(), nodes.end(), n) == nodes.end())
			nodes.push_back(n);
	}

	void addFlightArc(TS_Arc* a) { flightArcs.push_back(a); }
	void addGroundArc(TS_Arc* a) { groundArcs.push_back(a); }

	int getNumNodes() const { return static_cast<int>(nodes.size()); }
	int getNumArcs() const { return static_cast<int>(flightArcs.size() + groundArcs.size()); }
	int getNumFlightArcs() const { return static_cast<int>(flightArcs.size()); }
	int getNumGroundArcs() const { return static_cast<int>(groundArcs.size()); }

};