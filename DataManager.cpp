#include "DataManager.h"
#include "boost/algorithm/string.hpp"
#include <fstream>

DataRegistry* DataRegistry::dataInstance = nullptr;
ParamRegistry* ParamRegistry::paramInstance = nullptr;

DataRegistry::DataRegistry()
{
    aircrafts.clear();
    schLegs.clear();
}

ParamRegistry::ParamRegistry()
{
    writeLpFiles = false;
    printNetwork = false;
    printAlgProcess = true;

    bigM = 1.0e10;
    maxRunTime = 20 * 60;
    maxIpRunTime = 10 * 60;
    mpGapTol = 0.01;
    
    maxIterations = 10;
    scoreThreshold = -0.1;
    maxCopyRatio = 2;
}

void DataRegistry::readInputDataFile(const std::string& input_directory)
{
    auto cg_dataReg = DataRegistry::instance();

    std::string schFile = input_directory + "schedule.csv";
    std::string acFile = input_directory + "ac.csv";
    std::string pdFile = input_directory + "product.csv";

    std::ifstream ifsAc;
    ifsAc.open(acFile, std::ifstream::in);
    bool first_line = true;
    while (ifsAc.good())
    {

        char lineChars[256];
        ifsAc.getline(lineChars, 256);
        if (first_line) {
            first_line = false;
            continue;
        }
        std::string strLine(lineChars);

        std::vector<std::string> sVals;
        boost::split(sVals, strLine, boost::is_any_of(","));
        if (sVals.size() == 1)
        {
            break;
        }
        int i = 0;
        std::string tail;
        int capcity, cost, num;
        for (auto const& sVal : sVals)
        {
            switch (i)
            {
            case 0:
                tail = sVal;
                break;
            case 1:
                capcity = std::stoi(sVal);
                break;
            case 2:
                cost = std::stoi(sVal);
                break;
            case 3:
                num = std::stoi(sVal);
                break;
            default:
                break;
            }
            ++i;
        }
        auto pAc = std::make_shared<Aircraft>(tail, cost, capcity, num);
        cg_dataReg->aircrafts.push_back(pAc);
        pAc->setID(cg_dataReg->aircrafts.size());
    }
    ifsAc.close();


    std::ifstream ifsSch;
    ifsSch.open(schFile, std::ifstream::in);
    first_line = true;
    while (ifsSch.good())
    {
        char lineChars[256];
        ifsSch.getline(lineChars, 256);
        if (first_line) {
            first_line = false;
            continue;
        }
        std::string strLine(lineChars);

        std::vector<std::string> sVals;
        boost::split(sVals, strLine, boost::is_any_of(","));
        if (sVals.size() == 1)
        {
            break;
        }

        int lID, dur;
        std::string fltNum, depTime, arrTime, depSta, arrSta;
        int i = 0;
        for (auto const& sVal : sVals)
        {
            switch (i)
            {
            case 0:
                fltNum = sVal;
                break;
            case 1:
                depTime = sVal;
                break;
            case 2:
                arrTime = sVal;
                break;
            case 3:
                depSta = sVal;
                break;
            case 4:
                arrSta = sVal;
                break;
            case 5:
                dur = std::stoi(sVal);
                break;
            case 6:
                lID = std::stoi(sVal);
                break;
            default:
                break;
            }
            ++i;
        }
        auto pDepStn = getOrCreateStation(depSta);
        auto pArrStn = getOrCreateStation(arrSta);

        auto pLeg = std::make_shared<Leg>(fltNum, depTime, arrTime, pDepStn, pArrStn, dur, lID);
        
        cg_dataReg->schLegs.push_back(pLeg);
    }
    ifsSch.close();

    std::ifstream ifsPd;
    ifsPd.open(pdFile, std::ifstream::in);
    first_line = true;
    while (ifsPd.good())
    {

        char lineChars[256];
        ifsPd.getline(lineChars, 256);
        if (first_line) {
            first_line = false;
            continue;
        }
        std::string strLine(lineChars);

        std::vector<std::string> sVals;
        boost::split(sVals, strLine, boost::is_any_of(","));
        if (sVals.size() == 1)
        {
            break;
        }
        int i = 0;
        std::string ori, des;
        std::vector<std::string> flts;
        double f, d;
        for (auto const& sVal : sVals)
        {
            switch (i)
            {
            case 0:
                ori = sVal;
                break;
            case 1:
                des = sVal;
                break;
            case 2:
                f = std::stod(sVal);
                break;
            case 3:
                d = std::stod(sVal);
                break;
            default:
                flts.push_back(sVal);
                break;
            }
            ++i;
        }
        
        auto pOri = getOrCreateStation(ori);
        auto pDes = getOrCreateStation(des);
        auto pPro = std::make_shared<Product>(pOri, pDes, f, d);
        
        for (auto f : flts)
            if (f != ".")
                pPro->addFlt(f);
        cg_dataReg->products.push_back(pPro);
        pPro->setID(cg_dataReg->products.size());
    }
    ifsPd.close();

    /*for(auto &ac : aircrafts){
       std::sort(ac->scheduledFlights.begin(), ac->scheduledFlights.end(), [](Flight* a, Flight* b)->bool{
          return a->getDepTime() < b->getDepTime();
       });
    }*/

    
}

Station* DataRegistry::getOrCreateStation(const std::string& stnName)
{
    if (stnName.empty())
    {
        return nullptr;
    }

    auto reg = DataRegistry::instance();
    auto itrStn = reg->_stationMap.find(stnName);
    if (itrStn == reg->_stationMap.end())
    {
        int staID = reg->stations.size() + 1;
        auto pStn = std::make_shared<Station>(stnName, staID);
        reg->stations.push_back(pStn);
        reg->_stationMap[stnName] = pStn;
        return pStn.get();
    }
    return itrStn->second.get();
}

