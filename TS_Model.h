#ifndef TS_MODEL_H
#define TS_MODEL_H


#define _CRT_SECURE_NO_WARNINGS


#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <ctime>
#include <memory>
#include <ilcplex/ilocplex.h>

#include "Flight.h"
#include "Station.h"
#include "Product.h"
#include "DataManager.h"
#include "TS_Network.h"

typedef IloArray<IloNumVarArray> IloNumVarArray2;
typedef IloArray<IloIntVarArray> IloIntVarArray2;
typedef IloArray<IloConversion> IloConversion2;
typedef IloArray<IloRangeArray> IloRangeArray2;

ILOSTLBEGIN


class TS_Model {
private:
	std::vector<std::shared_ptr<TS_Node> > allNodes;
	std::vector<std::shared_ptr<TS_Arc> > allArcs;

	std::map<int, std::vector<TS_Node*> > stationNodesMap;
	std::vector<TS_Arc* > allFlightArcs;
	std::vector<TS_Arc* > allGroundArcs;

	std::vector<Flight* > unassignedFlights;
	std::map<unsigned, unsigned > assignment;

	std::string input_directory;
	std::string output_directory;

	double cpuTime;

	IloEnv env;
	IloCplex masterCplex;
	IloModel masterModel;
	IloObjective masterObj;
	IloIntVarArray2 varAssignFlightArcs;
	IloIntVarArray2 varAssignGroundArcs;
	IloIntVarArray varSatisfiedDemand;
	IloRangeArray FlightCover;
	IloRangeArray2 NetworkBalance;
	IloRangeArray AircraftCapacity;
	IloRangeArray ProductDemand;
	IloRangeArray FleetNum;
	IloRangeArray2 NonDirectFlights;

public:
	explicit TS_Model(const std::string& directory);

	virtual void optimize();
	virtual void buildNetwork();

	void buildFormulation();
	void initObjective();
	void initVariables();
	void initConstraints();
	void solveModel();
	void updateSolution();
	void writeResults();
	void deleteModel();

	static int getNumTypeAircrafts() { return static_cast<int>(DataRegistry::instance()->aircrafts.size()); }

	void setInputDirectory(const std::string& _dir) { input_directory = _dir; }
	void setOutputDirectory(const std::string& dir) { output_directory = dir; }
	std::string getInputDirectory() const { return input_directory; }

	TS_Node* addNode(std::string _t, Station* _s) {
		const auto it = std::find_if(allNodes.begin(), allNodes.end(), [_t, _s](const shared_ptr<TS_Node>& n)->bool {
			return (n->getTime() == _t) && (n->getStation() == _s);
			});
		if (it == allNodes.end()) {
			auto pNode = std::make_shared<TS_Node>(_t, _s);
			allNodes.push_back(pNode);
			return pNode.get();
		}
		else {
			return it->get();
		}
	}

	void addArc(TS_Node* _depNode, TS_Node* _arrNode, Leg* _leg) {
		auto pArc = std::make_shared<TS_Arc>(_depNode, _arrNode, _leg);
		allArcs.push_back(pArc);
	}

	TS_Node* getNode(std::string _t, Station* _s) {
		const auto it = std::find_if(allNodes.begin(), allNodes.end(), [_t, _s](const shared_ptr<TS_Node>& n)->bool {
			return (n->getTime() == _t) && (n->getStation() == _s);
			});
		return it->get();
	}

	static int getIndex(const Aircraft* a) {
		const auto& aircrafts = DataRegistry::instance()->aircrafts;
		auto it = std::find_if(aircrafts.begin(), aircrafts.end(), [a](const std::shared_ptr<Aircraft>& aircraft)->bool {
			return a->getID() == aircraft->getID();
			});
		if (it == aircrafts.end()) exit(EXIT_FAILURE);
		return static_cast<int>(it - aircrafts.begin());
	}

	static int getIndex(const Leg* l) {
		const auto& schLegs = DataRegistry::instance()->schLegs;
		auto it = std::find_if(schLegs.begin(), schLegs.end(), [l](const std::shared_ptr<Leg>& _l)->bool {
			return _l->getID() == l->getID();
			});
		if (it == schLegs.end()) exit(EXIT_FAILURE);
		return int(it - schLegs.begin());
	}


};

#endif // !TS_MODEL_H


