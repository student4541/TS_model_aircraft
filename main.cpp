#include "DataManager.h"
#include <iostream>
#include "TS_Model.h"


int main()
{
	auto data = DataRegistry::instance();
	data->readInputDataFile("C:/Users/yuyl_Allen/Desktop/");
	TS_Model tsModel("C:/Users/yuyl_Allen/Desktop/");
	tsModel.optimize();
}