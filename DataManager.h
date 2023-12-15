#ifndef CG_DataManager_H
#define CG_DataManager_H

#include "Flight.h"
#include "Aircraft.h"
#include "Station.h"
#include "Product.h"

#include <unordered_map>

class DataRegistry {
private:
	static DataRegistry* dataInstance;

	DataRegistry();
	~DataRegistry() = default;

public:

	//只允许通过静态函数构造指向对象指针的静态成员，而不允许直接调用构造函数
	static DataRegistry* instance() {
		if (!dataInstance) {
			dataInstance = new DataRegistry();
		}

		return dataInstance;
	}

	std::vector<std::shared_ptr<Leg> > schLegs;
	std::vector<std::shared_ptr<Aircraft> > aircrafts;
	std::vector<std::shared_ptr<Station> > stations;
	std::vector<std::shared_ptr<Product> > products;

	std::unordered_map<std::string, std::shared_ptr<Station> > _stationMap;
	std::unordered_map<std::string, std::shared_ptr<Aircraft> > _taiMap;

	void readInputDataFile(const std::string& input_directory);
	Station* getOrCreateStation(const std::string& stnName);
};

class ParamRegistry {
private:
	static ParamRegistry* paramInstance;

	ParamRegistry();

public:
	static ParamRegistry* instance()
	{
		if (!paramInstance)
			paramInstance = new ParamRegistry();
		return paramInstance;
	}

	bool writeLpFiles;
	bool printNetwork;
	bool printAlgProcess;

	double maxIpRunTime;
	double maxRunTime;
	double bigM;
	double mpGapTol;

	int maxIterations;
	double scoreThreshold;
	double maxCopyRatio;
};


#endif // !CG_DataManager_H



