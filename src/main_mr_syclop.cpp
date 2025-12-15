#include <ompl/multirobot/base/SpaceInformation.h>
#include <ompl/multirobot/base/ProblemDefinition.h>
#include <ompl/multirobot/geometric/planners/mrsyclop/MRSyCLoP.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/geometric/planners/rrt/RRT.h>

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

// Include robots and FCL checker if available in your setup
// #include "robots.h"
// #include "fclStateValidityChecker.hpp"

namespace omrb = ompl::multirobot::base;
namespace omrg = ompl::multirobot::geometric;
namespace ob = ompl::base;

// Simple concrete GridDecomposition implementation
class SimpleGridDecomposition : public ompl::control::GridDecomposition
{
public:
    SimpleGridDecomposition(int len, int dim, const ob::RealVectorBounds &bounds,
                           const std::vector<int> &gridSizes)
        : GridDecomposition(len, dim, bounds), gridSizes_(gridSizes)
    {
        // Calculate total number of regions
        numRegions_ = 1;
        for (int size : gridSizes_)
            numRegions_ *= size;
    }

    void project(const ob::State *s, std::vector<double> &coord) const override
    {
        // For SE2 states, project the (x, y) position
        const auto *se2 = s->as<ob::SE2StateSpace::StateType>();
        coord.resize(2);
        coord[0] = se2->getX();
        coord[1] = se2->getY();
    }

    void sampleFullState(const ob::StateSamplerPtr &sampler, const std::vector<double> &coord,
                        ob::State *s) const override
    {
        // Sample a full state with the given projection coordinates
        sampler->sampleUniform(s);
        auto *se2 = s->as<ob::SE2StateSpace::StateType>();
        se2->setX(coord[0]);
        se2->setY(coord[1]);
        // Yaw is already sampled randomly by sampleUniform
    }

    int locateRegion(const ob::State *s) const override
    {
        // Project state to coordinates
        std::vector<double> coord;
        project(s, coord);

        // Calculate cell dimensions
        double cellWidth = (bounds_.high[0] - bounds_.low[0]) / gridSizes_[0];
        double cellHeight = (bounds_.high[1] - bounds_.low[1]) / gridSizes_[1];

        // Determine which cell the coordinates are in
        int col = static_cast<int>((coord[0] - bounds_.low[0]) / cellWidth);
        int row = static_cast<int>((coord[1] - bounds_.low[1]) / cellHeight);

        // Clamp to valid range
        col = std::min(std::max(col, 0), gridSizes_[0] - 1);
        row = std::min(std::max(row, 0), gridSizes_[1] - 1);

        // Compute region ID in row-major order
        return row * gridSizes_[0] + col;
    }

    void getNeighbors(int rid, std::vector<int> &neighbors) const override
    {
        neighbors.clear();

        // Convert region ID to row/col
        int row = rid / gridSizes_[0];
        int col = rid % gridSizes_[0];

        // Add all 4-connected neighbors (up, down, left, right)
        // Up (row + 1)
        if (row + 1 < gridSizes_[1])
            neighbors.push_back((row + 1) * gridSizes_[0] + col);

        // Down (row - 1)
        if (row - 1 >= 0)
            neighbors.push_back((row - 1) * gridSizes_[0] + col);

        // Right (col + 1)
        if (col + 1 < gridSizes_[0])
            neighbors.push_back(row * gridSizes_[0] + (col + 1));

        // Left (col - 1)
        if (col - 1 >= 0)
            neighbors.push_back(row * gridSizes_[0] + (col - 1));
    }

private:
    std::vector<int> gridSizes_;
    int numRegions_;
};

// Simple planner allocator for RRT
ompl::base::PlannerPtr PlannerAllocator(const ompl::base::SpaceInformationPtr &si)
{
    return std::make_shared<ompl::geometric::RRT>(si);
}

// Robot information for collision checking
struct RobotInfo
{
    double radius;
    ob::SpaceInformationPtr si;
    unsigned int index;
};

// Multi-robot state validity checker with distance-based collision detection
class CircularRobotStateValidityChecker : public ob::StateValidityChecker
{
public:
    CircularRobotStateValidityChecker(const ob::SpaceInformationPtr &si,
                                     const std::vector<RobotInfo> &allRobots,
                                     unsigned int robotIndex)
        : ob::StateValidityChecker(si)
        , allRobots_(allRobots)
        , robotIndex_(robotIndex)
    {
    }

    bool isValid(const ob::State *state) const override
    {
        // Basic bounds checking
        return si_->satisfiesBounds(state);
    }

    // Inter-robot collision checking using distance between circular robots
    bool areStatesValid(const ob::State* state1,
                       const std::pair<const ob::SpaceInformationPtr, const ob::State*> state2) const override
    {
        const auto &otherSI = state2.first;
        const auto *otherState = state2.second;

        // Find which robot the other SpaceInformation corresponds to
        int otherRobotIdx = -1;
        for (size_t i = 0; i < allRobots_.size(); ++i)
        {
            if (allRobots_[i].si.get() == otherSI.get())
            {
                otherRobotIdx = i;
                break;
            }
        }

        if (otherRobotIdx == -1)
        {
            // Couldn't find other robot, assume no collision
            return true;
        }

        // Get positions of both robots (assuming SE2 state space)
        const auto *se2_state1 = state1->as<ob::SE2StateSpace::StateType>();
        const auto *se2_state2 = otherState->as<ob::SE2StateSpace::StateType>();

        double x1 = se2_state1->getX();
        double y1 = se2_state1->getY();
        double x2 = se2_state2->getX();
        double y2 = se2_state2->getY();

        // Calculate distance between robot centers
        double dx = x2 - x1;
        double dy = y2 - y1;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Get radii of both robots
        double r1 = allRobots_[robotIndex_].radius;
        double r2 = allRobots_[otherRobotIdx].radius;

        // Check for collision (distance < sum of radii)
        // Add small buffer for numerical stability
        const double buffer = 0.01;
        return distance >= (r1 + r2 - buffer);
    }

private:
    const std::vector<RobotInfo> &allRobots_;
    unsigned int robotIndex_;
};

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;

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

    // Load environment from YAML
    YAML::Node env = YAML::LoadFile(inputFile);
    YAML::Node cfg = YAML::LoadFile(cfgFile);

    // Get environment bounds
    const auto &env_min = env["environment"]["min"];
    const auto &env_max = env["environment"]["max"];
    ob::RealVectorBounds position_bounds(env_min.size());
    for (size_t i = 0; i < env_min.size(); ++i) {
        position_bounds.setLow(i, env_min[i].as<double>());
        position_bounds.setHigh(i, env_max[i].as<double>());
    }

    // Create multi-robot space information and problem definition
    auto ma_si = std::make_shared<omrb::SpaceInformation>();
    auto ma_pdef = std::make_shared<omrb::ProblemDefinition>(ma_si);

    // First pass: collect robot information
    std::vector<RobotInfo> allRobots;
    for (const auto &robot_node : env["robots"]) {
        RobotInfo info;
        info.radius = robot_node["radius"] ? robot_node["radius"].as<double>() : 0.5;  // Default radius
        info.index = allRobots.size();
        // si will be set in second pass
        allRobots.push_back(info);
    }

    // Second pass: create spaces and checkers with full robot info
    std::vector<double> start_reals, goal_reals;
    unsigned int robotIdx = 0;
    for (const auto &robot_node : env["robots"]) {
        // Create SE2 state space for this robot
        auto space = std::make_shared<ob::SE2StateSpace>();
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, position_bounds.low[0]);
        bounds.setHigh(0, position_bounds.high[0]);
        bounds.setLow(1, position_bounds.low[1]);
        bounds.setHigh(1, position_bounds.high[1]);
        space->setBounds(bounds);

        // Create space information
        auto si = std::make_shared<ob::SpaceInformation>(space);

        // Store SI in robot info for collision checking
        allRobots[robotIdx].si = si;

        // Set collision checker with all robot info
        auto checker = std::make_shared<CircularRobotStateValidityChecker>(si, allRobots, robotIdx);
        si->setStateValidityChecker(checker);
        si->setup();

        // Create problem definition
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);

        // Set start state
        start_reals.clear();
        for (const auto& v : robot_node["start"]) {
            start_reals.push_back(v.as<double>());
        }
        auto startState = si->allocState();
        space->copyFromReals(startState, start_reals);
        si->enforceBounds(startState);
        pdef->addStartState(startState);
        // Note: Don't free startState - pdef manages this memory

        // Set goal state
        goal_reals.clear();
        for (const auto& v : robot_node["goal"]) {
            goal_reals.push_back(v.as<double>());
        }
        auto goalState = si->allocState();
        space->copyFromReals(goalState, goal_reals);
        si->enforceBounds(goalState);
        pdef->setGoalState(goalState, cfg["goal_epsilon"].as<double>());
        // Note: Don't free goalState - pdef manages this memory

        // Add to multi-robot containers
        ma_si->addIndividual(si);
        ma_pdef->addIndividual(pdef);

        ++robotIdx;
    }

    // Lock the multi-robot SpaceInformation and ProblemDefinitions
    ma_si->lock();
    ma_pdef->lock();

    // Set planner allocator
    ompl::base::PlannerAllocator allocator = PlannerAllocator;
    ma_si->setPlannerAllocator(allocator);

    if (plannerDesc == "mr-syclop") {
        // Create decomposition for SyCLoP
        ob::RealVectorBounds decomp_bounds(2);
        decomp_bounds.setLow(0, position_bounds.low[0]);
        decomp_bounds.setHigh(0, position_bounds.high[0]);
        decomp_bounds.setLow(1, position_bounds.low[1]);
        decomp_bounds.setHigh(1, position_bounds.high[1]);

        // Create grid decomposition (you can adjust grid resolution)
        int gridResolution = cfg["grid_resolution"].as<int>(32);  // Default 32x32 grid
        auto decomposition = std::make_shared<SimpleGridDecomposition>(
            gridResolution, 2, decomp_bounds, std::vector<int>{gridResolution, gridResolution});

        // Create MR-SyCLoP planner
        auto planner = std::make_shared<omrg::MRSyCLoP>(ma_si);
        planner->setProblemDefinition(ma_pdef);
        planner->setDecomposition(decomposition);

        // Set planner parameters from config
        if (cfg["num_region_expansions"]) {
            planner->setNumRegionExpansions(cfg["num_region_expansions"].as<unsigned int>());
        }
        if (cfg["num_tree_selections"]) {
            planner->setNumTreeSelections(cfg["num_tree_selections"].as<unsigned int>());
        }
        if (cfg["prob_shortest_path"]) {
            planner->setProbShortestPath(cfg["prob_shortest_path"].as<double>());
        }

        // Solve
        std::cout << "Planning with MR-SyCLoP..." << std::endl;
        std::ofstream stats(statsFile);
        stats << "stats:" << std::endl;
        auto start = std::chrono::steady_clock::now();

        bool solved = planner->as<omrb::Planner>()->solve(timelimit);

        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        stats << "  - t: " << t/1000.0 << std::endl;

        if (solved) {
            std::cout << "Found solution!" << std::endl;

            // Write solution to output file
            std::ofstream resultFile(outputFile);
            resultFile << "result:" << std::endl;

            // Extract paths for each robot
            for (unsigned int r = 0; r < ma_si->getIndividualCount(); ++r) {
                auto pdef = ma_pdef->getIndividual(r);
                auto si = ma_si->getIndividual(r);

                if (pdef->getSolutionPath() != nullptr) {
                    auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
                        pdef->getSolutionPath());

                    resultFile << "  - states:" << std::endl;
                    std::vector<double> reals;

                    for (size_t i = 0; i < path->getStateCount(); ++i) {
                        si->getStateSpace()->copyToReals(reals, path->getState(i));
                        resultFile << "    - [";
                        for (size_t j = 0; j < reals.size(); ++j) {
                            resultFile << reals[j];
                            if (j < reals.size() - 1) {
                                resultFile << ",";
                            }
                        }
                        resultFile << "]" << std::endl;
                    }
                }
            }

            // Export decomposition/region data for visualization
            resultFile << "decomposition:" << std::endl;
            resultFile << "  type: grid" << std::endl;
            resultFile << "  grid_size: [" << gridResolution << ", " << gridResolution << "]" << std::endl;
            resultFile << "  bounds:" << std::endl;
            resultFile << "    min: [" << decomp_bounds.low[0] << ", " << decomp_bounds.low[1] << "]" << std::endl;
            resultFile << "    max: [" << decomp_bounds.high[0] << ", " << decomp_bounds.high[1] << "]" << std::endl;

            // Export leads (region sequences) for each robot
            const auto &leads = planner->getLeads();
            resultFile << "  leads:" << std::endl;
            for (unsigned int r = 0; r < leads.size(); ++r) {
                resultFile << "    - [";
                for (size_t i = 0; i < leads[r].size(); ++i) {
                    resultFile << leads[r][i];
                    if (i < leads[r].size() - 1) {
                        resultFile << ", ";
                    }
                }
                resultFile << "]" << std::endl;
            }

            stats << "    solved: true" << std::endl;
        } else {
            std::cout << "No solution found within time limit." << std::endl;
            stats << "    solved: false" << std::endl;
        }
    } else {
        std::cerr << "Unknown planner: " << plannerDesc << std::endl;
        return 1;
    }

    return 0;
}