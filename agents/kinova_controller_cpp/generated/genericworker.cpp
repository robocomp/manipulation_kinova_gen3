/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "genericworker.h"
/**
* \brief Default constructor
*/
GenericWorker::GenericWorker(const ConfigLoader& configLoader, TuplePrx tprx) : Ui_guiDlg()
{
	QLoggingCategory::setFilterRules("*.debug=false\n");

	this->configLoader = configLoader;
	
	contactile_proxy = std::get<0>(tprx);

	states["Initialize"] = std::make_unique<GRAFCETStep>("Initialize", BASIC_PERIOD, nullptr, std::bind(&GenericWorker::initialize, this));
	states["Compute"] = std::make_unique<GRAFCETStep>("Compute", configLoader.get<int>("Period.Compute"), std::bind(&GenericWorker::compute, this));
	states["Emergency"] = std::make_unique<GRAFCETStep>("Emergency", configLoader.get<int>("Period.Emergency"), std::bind(&GenericWorker::emergency, this));
	states["Restore"] = std::make_unique<GRAFCETStep>("Restore", BASIC_PERIOD, nullptr, std::bind(&GenericWorker::restore, this));

	states["Initialize"]->addTransition(states["Initialize"].get(), SIGNAL(entered()), states["Compute"].get());
	states["Compute"]->addTransition(this, SIGNAL(goToEmergency()), states["Emergency"].get());
	states["Emergency"]->addTransition(this, SIGNAL(goToRestore()), states["Restore"].get());
	states["Restore"]->addTransition(states["Restore"].get(), SIGNAL(entered()), states["Compute"].get());

	statemachine.addState(states["Initialize"].get());
	statemachine.addState(states["Compute"].get());
	statemachine.addState(states["Emergency"].get());
	statemachine.addState(states["Restore"].get());

	statemachine.setInitialState(states["Initialize"].get());

	connect(&hibernationChecker, SIGNAL(timeout()), this, SLOT(hibernationCheck()));


	#ifdef USE_QTGUI
		setupUi(this);
		show();
	#endif

    agent_name = this->configLoader.get<std::string>("Agent.name");
    agent_id = this->configLoader.get<int>("Agent.id");

    // Create graph
    G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, this->configLoader.get<std::string>("Agent.configFile")); // Init nodes
    std::cout<< "Graph loaded" << std::endl;  
    
    // Graph viewer
    using opts = DSR::DSRViewer::view;
    int current_opts = 0;
    opts main = opts::none;
    if(this->configLoader.get<bool>("ViewAgent.tree"))
    {
        current_opts = current_opts | opts::tree;
    }
    if(this->configLoader.get<bool>("ViewAgent.graph"))
    {
        current_opts = current_opts | opts::graph;
        main = opts::graph;
    }
    if(this->configLoader.get<bool>("ViewAgent.2d"))
    {
        current_opts = current_opts | opts::scene;
    }
    if(this->configLoader.get<bool>("ViewAgent.3d"))
    {
        current_opts = current_opts | opts::osg;
    }
    graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
    setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

	widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{
    auto grid_nodes = G->get_nodes_by_type("grid");
    for (auto grid : grid_nodes)
    {
    	G->delete_node(grid);
    }
    G.reset();
}
void GenericWorker::killYourSelf()
{
	qDebug("Killing myself");
	emit kill();
}

/**
* \brief Change compute period of state
* @param state name of state
* @param period Period in ms
*/
void GenericWorker::setPeriod(const std::string& state, int period)
{
    auto it = states.find(state); 
    if (it != states.end() && it->second != nullptr)
    {
		it->second->setPeriod(period);
		std::cout << "Period for state " << state << " changed to " << period << "ms" << std::endl << std::flush;
	}
    else
        std::cerr << "No change in the period, the state is not valid or not configured."<< std::endl;
}

int GenericWorker::getPeriod(const std::string& state)
{
    auto it = states.find(state);

    if (it == states.end() || it->second == nullptr)
    {
        std::cerr << "Invalid or unconfigured state: " << state << std::endl;
        return -1; 
	}
    return it->second->getPeriod();
}

void GenericWorker::hibernationCheck()
{
	//Time between activity to activate hibernation
    static const int HIBERNATION_TIMEOUT = 5000;

    static std::chrono::high_resolution_clock::time_point lastWakeTime = std::chrono::high_resolution_clock::now();
	static int originalPeriod;
    static bool isInHibernation = false;

	// Update lastWakeTime by calling a function
    if (hibernation)
    {
        hibernation = false;
        lastWakeTime = std::chrono::high_resolution_clock::now();

		// Restore period
        if (isInHibernation)
        {
            this->setPeriod("Compute", originalPeriod);
            isInHibernation = false;
        }
    }

    auto now = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastWakeTime);

	//HIBERNATION_TIMEOUT exceeded, change period
    if (elapsedTime.count() > HIBERNATION_TIMEOUT && !isInHibernation)
    {
        isInHibernation = true;
		originalPeriod = this->getPeriod("Compute");
        this->setPeriod("Compute", 500);
    }
}
