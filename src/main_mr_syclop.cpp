#include <ompl/multirobot/control/SpaceInformation.h>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/control/planners/pp/PP.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/objectives/ControlDurationObjective.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/goals/GoalRegion.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <utility>
#include <unordered_map>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>


int main(int argc, char* argv[]){
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    std::string statsFile;
    std::string plannerDesc;
    std::string cfgFile;
    int timelimit;
    desc.add_options()
        ("help", "produce help message")
        ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
        ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
        ("stats", po::value<std::string>(&statsFile)->default_value("ompl_stats.yaml"), "output file (yaml)")
        ("planner,p", po::value<std::string>(&plannerDesc)->default_value("mr-syclop"), "Planner")
        ("timelimit", po::value<int>(&timelimit)->default_value(60), "Time limit for planner")
        ("cfg,c", po::value<std::string>(&cfgFile)->required(), "configuration file (yaml)");

    try {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help") != 0u) {
        std::cout << desc << "\n";
        return 0;
        }
    } catch (po::error& e) {
        std::cerr << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    // TODO: implement mr-syclop main
 
    return 0;
}