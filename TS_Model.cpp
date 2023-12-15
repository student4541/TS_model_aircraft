#include "TS_Model.h"
#include "DataManager.h"

TS_Model::TS_Model(const std::string& d)
{
    cpuTime = 0;

    setInputDirectory(d);
    setOutputDirectory(d + "out/");

    allNodes.clear();
    allArcs.clear();
}

void TS_Model::optimize()
{
    buildNetwork();
    buildFormulation();
    solveModel();
    writeResults();
}

void TS_Model::buildNetwork()
{
    const auto paramReg = ParamRegistry::instance();
    auto& aircrafts = DataRegistry::instance()->aircrafts;
    auto& schLegs = DataRegistry::instance()->schLegs;

    /* ********************* Flight Arcs ******************** */
    // flight and maintenance arcs and their nodes
    for (const auto& leg : schLegs)
    {
        this->addNode(leg->getDepTime(), leg->getDepStation());
        this->addNode(leg->getArrTime(), leg->getArrStation());

        const auto depNode = getNode(leg->getDepTime(), leg->getDepStation());
        const auto arrNode = getNode(leg->getArrTime(), leg->getArrStation());
        addArc(depNode, arrNode, leg.get());
    }

    for (const auto& arc : allArcs) {
        allFlightArcs.push_back(arc.get());
    }

    /* ********************* Nodes ******************** */
    // sort all the nodes by time
    std::sort(allNodes.begin(), allNodes.end(), [](const shared_ptr<TS_Node>& a, const shared_ptr<TS_Node>& b) -> bool
        {
            return a->getTime() < b->getTime();
        });

    // assign node ID
    int ndId = 0;
    for (auto& node : allNodes)
    {
        node->setID(ndId++);
    }

    // create station:nodes map
    stationNodesMap.clear();
    for (const auto& node : allNodes) {
        const auto staId = node->getStation()->getID();
        auto it = stationNodesMap.find(staId);
        if (it != stationNodesMap.end()) {
            auto itNode = std::find(it->second.begin(), it->second.end(), node.get());
            if (itNode == it->second.end()) {
                stationNodesMap[staId].push_back(node.get());
            }
        }
        else {
            stationNodesMap[staId].push_back(node.get());
        }
    }

    /* ********************* Ground Arcs ******************** */
    /* create ground arcs between consecutive nodes */
    for (const auto& itrMap : stationNodesMap) {
        const auto& staNodes = itrMap.second;
        for (auto it = staNodes.begin(); it != staNodes.end(); it++) {
            auto itNextNode = std::next(it);
            if (itNextNode == staNodes.end()) {
                addArc(*it, *staNodes.begin(), nullptr);
                allGroundArcs.push_back(allArcs.back().get());
            }
            else {
                addArc(*it, *itNextNode, nullptr);
                allGroundArcs.push_back(allArcs.back().get());
            }
        }
    }
}

void TS_Model::buildFormulation()
{
    masterModel = IloModel(env);
    masterCplex = IloCplex(masterModel);
    masterObj = IloObjective(env, IloObjective::Maximize);

    initVariables();
    initObjective();
    initConstraints();

    masterCplex.extract(masterModel);
    if (ParamRegistry::instance()->writeLpFiles)
    {
        std::string filename = output_directory + "AAM.lp";
        masterCplex.exportModel(filename.c_str());
    }

    masterCplex.setOut(env.getNullStream());
}

void TS_Model::initVariables()
{
    const int numLegs = static_cast<int>(DataRegistry::instance()->schLegs.size());
    const int numProducts = static_cast<int>(DataRegistry::instance()->products.size());
    const int numFlightArcs = static_cast<int>(allFlightArcs.size());
    const int numGroundArcs = static_cast<int>(allGroundArcs.size());
    const int numAircraft = getNumTypeAircrafts();
    char buf[500];

    try
    {
        // Variables
        varSatisfiedDemand = IloIntVarArray(env, numProducts, 0, +IloInfinity);
        for (int i = 0; i < numProducts; i++)
        {
            std::sprintf(buf, "Product(%ld)", DataRegistry::instance()->products[i]->getID());
            varSatisfiedDemand[i] = IloIntVar(env, buf);
        }

        varAssignFlightArcs = IloIntVarArray2(env, numFlightArcs);
        varAssignGroundArcs = IloIntVarArray2(env, numGroundArcs);

        for (int i = 0; i < numFlightArcs; i++)
            varAssignFlightArcs[i] = IloIntVarArray(env, numAircraft, 0, +IloInfinity);
        for (int i = 0; i < numGroundArcs; i++)
            varAssignGroundArcs[i] = IloIntVarArray(env, numAircraft, 0, +IloInfinity);
        
        int j = 0;
        for (const auto pArc : allFlightArcs) {
            for (int i = 0; i < 9; i++)
            {
                std::sprintf(buf, "AssignFlight(%ld_%lld)",i, pArc->getLeg()->getID());
                varAssignFlightArcs[j][i] = IloIntVar(env, buf);
            }
            j++;
        }

        j = 0;
        for (const auto pArc : allGroundArcs) {
            const auto station = pArc->getHeadNode()->getStation();
            for (int i = 0; i < 9; i++)
            {
                std::sprintf(buf, "AssignGround(%ld_%d(%s_%s))", i, station->getID(),
                    pArc->getTailNode()->getTime(), pArc->getHeadNode()->getTime());
                varAssignGroundArcs[j][i] = IloIntVar(env, buf);
                cout << buf << endl;
            }
            j++;
        }
    }
    
    
    catch (const IloException& e)
    {
        cerr << "Exception caught: " << e << endl;
    }
    catch (...)
    {
        cerr << "Unknown exception caught!" << endl;
    }
}

void TS_Model::initObjective()
{
    const auto& schLegs = DataRegistry::instance()->schLegs;
    const int numProducts = static_cast<int>(DataRegistry::instance()->products.size());
    const int numLegs = static_cast<int>(schLegs.size());


    try {
        IloExpr obj(env);

        // Earning of all products
        for (int i = 0; i < numProducts; i++)
        {
            obj += varSatisfiedDemand[i] * DataRegistry::instance()->products[i]->getFare();
        }

        // Cost of all flight legs
        int j = 0;
        for (const auto& arc : allFlightArcs)
        {
            for (int i = 0; i < 9; i++)
            {
                obj -= varAssignFlightArcs[j][i] * DataRegistry::instance()->aircrafts[i]->getCost() * arc->getDuration() / 60;
            }
            j++;
        }


        masterObj = IloAdd(masterModel, IloMaximize(env, obj));
        obj.end();
    }
    catch (const IloException& e)
    {
        std::cerr << "Exception caught: " << e << endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception caught!" << endl;
    }
}

void TS_Model::initConstraints()
{
    const auto& schLegs = DataRegistry::instance()->schLegs;
    const int numFlights = static_cast<int>(schLegs.size());
    const auto& aircrafts = DataRegistry::instance()->aircrafts;
    const int numAircraft = getNumTypeAircrafts();
    const int numProducts = static_cast<int>(DataRegistry::instance()->products.size());

    char buf[100];
    // Flight Cover Constraints
    FlightCover = IloRangeArray(env, numFlights);
    int j = 0;
    for (const auto& pArc : allFlightArcs)
    {
        IloExpr tempExpr(env);
        for (int i = 0; i < 9; i++)
        {
            tempExpr += varAssignFlightArcs[j][i];
        }

        //可能有问题
        std::sprintf(buf, "FltCover(%ld)", schLegs[j]->getID());
        FlightCover[j] = IloAdd(masterModel, IloRange(env, 1, tempExpr, 1, buf));
        tempExpr.end();
        
        j++;
    }

    // Network Flow Balance Constraints
    NetworkBalance = IloRangeArray2(env, static_cast<int>(this->allNodes.size()));
    for(int i = 0; i < static_cast<int>(this->allNodes.size()); i++)
        NetworkBalance[i] = IloRangeArray(env, this->getNumTypeAircrafts());
    for (auto node : allNodes) {
        node->enteringFlightArcs.clear();
        node->leavingFlightArcs.clear();
        node->enteringGroundArcs.clear();
        node->leavingGroundArcs.clear();
    }

    for (int i = 0; i < allFlightArcs.size(); i++)
    {
        allFlightArcs[i]->getHeadNode()->enteringFlightArcs.emplace_back(i);
        allFlightArcs[i]->getTailNode()->leavingFlightArcs.emplace_back(i);
    }
    for (int i = 0; i < allGroundArcs.size(); i++)
    {

        allGroundArcs[i]->getHeadNode()->enteringGroundArcs.emplace_back(i);
        allGroundArcs[i]->getTailNode()->leavingGroundArcs.emplace_back(i);
    }

    // constraints

    //有问题吗？
    for (int k = 0; k < 9 ;k++)
    {
        for (int n = 0; n < static_cast<int>(this->allNodes.size()); n++) 
        {
            const auto node = allNodes[n];
            IloExpr tempExpr(env);
            for (int index : node->enteringFlightArcs) {
                tempExpr += varAssignFlightArcs[index][k];
            }
            for (const auto index : node->leavingFlightArcs) {
                tempExpr -= varAssignFlightArcs[index][k];
            }
            for (const auto index : node->enteringGroundArcs) {
                tempExpr += varAssignGroundArcs[index][k];
            }
            for (const auto index : node->leavingGroundArcs) {
                tempExpr -= varAssignGroundArcs[index][k];
            }
            //======
            std::sprintf(buf, "FlowBalance(%s,%ld)", node->getTime(), node->getStation()->getID());
            NetworkBalance[n][k] = IloAdd(masterModel, IloRange(env, 0, tempExpr, 0, buf));
            // masterMod.add(IloRange(env, rhs, tempExpr, rhs, buf));
            tempExpr.end();
        }   
    }

    //Aircraft Capacity Constraint
    AircraftCapacity = IloRangeArray(env, numProducts);
    int i = 0;
    for (const auto& leg : DataRegistry::instance()->schLegs)
    {
        IloExpr tempExpr(env);
        for (int j = 0; j < 9; j++)
        {
            tempExpr += varAssignFlightArcs[i][j] * aircrafts[j]->getCapacity();
        }
        int j = 0;
        for (const auto& product : DataRegistry::instance()->products)
        {
            for (const auto& fltNum : product->getFltNums())
                if (fltNum == leg->getFlightNum())
                    tempExpr -= varSatisfiedDemand[j];
            j++;
        }
        std::sprintf(buf, "AircraftCapacity(%s)", leg->getFlightNum());
        AircraftCapacity[i] = IloAdd(masterModel, IloRange(env, 0, tempExpr,+IloInfinity, buf));
        // masterMod.add(IloRange(env, rhs, tempExpr, rhs, buf));
        tempExpr.end();
        i++;
    }

    //Fleet Number Constraint
    FleetNum = IloRangeArray(env, numAircraft);
    for (int i = 0; i < numAircraft; i++)
    {
        IloExpr tempExpr(env);
        int j = 0;
        for (const auto& fArc : allFlightArcs)
        {
            if ((fArc->getLeg()->getDepTime() <= std::string("2300")) && (fArc->getLeg()->getArrTime() >= std::string("2300")))
                tempExpr -= varAssignFlightArcs[j][i];
        }
        j = 0;
        for (const auto& gArc : allGroundArcs)
            if ((gArc->getTailNode()->getTime() <= std::string("2300")) && (gArc->getHeadNode()->getTime() >= std::string("2300")))
                tempExpr -= varAssignGroundArcs[j][i];
        tempExpr += aircrafts[i]->getNumAircrafts();

        FleetNum[i] = IloAdd(masterModel, IloRange(env, 0, tempExpr, +IloInfinity, buf));
        tempExpr.end();

    }

    //Demand Constraint
    ProductDemand = IloRangeArray(env, numProducts);
    for (int i = 0; i < numProducts; i++)
    {
        IloExpr tempExpr(env);
        tempExpr = DataRegistry::instance()->products[i]->getDemand() - varSatisfiedDemand[i];

        std::sprintf(buf, "ProductDemand(%ld)", i);
        ProductDemand[i] = IloAdd(masterModel, IloRange(env, 0, tempExpr, +IloInfinity, buf));
        // masterMod.add(IloRange(env, rhs, tempExpr, rhs, buf));
        tempExpr.end();
    }

    //NonDirect Flights Constraint
    //=====
    //std::vector<TS_Node* > ndNodes;
    //for (const auto& fArc : allFlightArcs)
    //    for (const auto& flt : ParamRegistry::instance()->NonDirectFlights)
    //        if (fArc->getLeg()->getFlightNum() == flt)
    //            ndNodes.push_back(fArc->getHeadNode());
    //NonDirectFlights = IloRangeArray2(env, static_cast<int>(ndNodes.size()));
    //for (int i = 0; i < ndNodes.size(); i++)
    //    NonDirectFlights[i] = IloRangeArray(env, numAircraft);

    //for (int i = 0; i < numAircraft; i++)
    //{
    //    int j = 0;
    //    for (const auto& n : ndNodes)
    //    {
    //        IloExpr tempExpr(env);
    //        tempExpr += varAssignFlightArcs[][i]
    //        j++;
    //    }
    //}
    


}

void TS_Model::solveModel()
{
    masterCplex.extract(masterModel);
    masterCplex.setParam(IloCplex::RootAlg, IloCplex::Auto);
    masterCplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, ParamRegistry::instance()->mpGapTol);
    masterCplex.setParam(IloCplex::Param::TimeLimit,ParamRegistry::instance()->maxIpRunTime);
    if (ParamRegistry::instance()->writeLpFiles)
    {
        std::string filename = output_directory + "Direct.lp";
        masterCplex.exportModel(filename.c_str());
    }
    masterCplex.solve();
}

void TS_Model::updateSolution()
{
    const auto &legs = DataRegistry::instance()->schLegs;
    try
    {
        if (masterCplex.getStatus() == IloAlgorithm::Infeasible || masterCplex.getStatus() == IloAlgorithm::Unbounded)
        {
            return;
        }

        for (const auto& ac : DataRegistry::instance()->aircrafts) {
            int j = 0;
            for (const auto pArc : allFlightArcs) {
                int i = 0;
                if (masterCplex.getValue(varAssignFlightArcs[i][j]) > 0.99) {
                    assignment.emplace(legs[i]->getFltID(), j);   
                }
                i++;
            }
            j++;
        }
        
    }
    catch (const IloException& e)
    {
        cerr << "Exception caught: " << e << endl;
    }
    catch (...)
    {
        cerr << "Unknown exception caught!" << endl;
    }
}

void TS_Model::writeResults()
{
    std::string filename = output_directory + "result.out";
    std::ofstream output;
    output.open(filename.c_str());

    output << "Objective:\t" << masterCplex.getObjValue() << std::endl;
    output << "Total CPU time:\t" << cpuTime << std::endl;
    output << "================== Aircraft Assignment ==================" << std::endl;
    const auto& aircrafts = DataRegistry::instance()->aircrafts;
    const auto& legs = DataRegistry::instance()->schLegs;
    for (auto& it : assignment)
    {
        output << legs[it.first]->getFlightNum();
        output << aircrafts[it.second]->getTail() << std::endl;
   }


    output.close();

   
}

void TS_Model::deleteModel()
{
    /*varAssignFlightArcs = IloBoolVarArray2(env, numAircraft);
    varAssignGroundArcs = IloBoolVarArray2(env, numAircraft);

    for (int i = 0; i < getNumAircrafts(); i++) {
        varAssignFlightArcs[i].endElements();
    }*/


    varAssignFlightArcs.end();
    varAssignGroundArcs.end();

    FlightCover.end();

    NetworkBalance.end();

    masterObj.end();
    masterCplex.end();
    masterModel.end();
}