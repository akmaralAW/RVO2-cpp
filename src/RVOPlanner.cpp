#include <cmath>
#include <cstddef>
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include <RVO.h>
#include <Trajectory.h>

static std::vector<RVO::Vector2> goals;
// later use with input.yaml
void setupScenario(RVO::RVOSimulator *sim)
{
	/* Specify the global time step of the simulation. */
	sim->setTimeStep(0.25f);

	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(15.0f, 10, 10.0f, 10.0f, 0.5f, 2.0f);
}

void setPreferredVelocities(RVO::RVOSimulator* sim)
{
    for (size_t i = 0; i < sim->getNumAgents(); ++i) {
        RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

        if (RVO::absSq(goalVector) > 1.0f) {
            goalVector = RVO::normalize(goalVector); // velocity (vx,vy) towards the goal
        }

        sim->setAgentPrefVelocity(i, goalVector);
    }
}

bool reachedGoal(RVO::RVOSimulator* sim)
{
    for (size_t i = 0; i < sim->getNumAgents(); ++i) {
        if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) >
            sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
            return false;
        }
    }
    return true;
}

void runRVO(const std::string& input_yaml,
            const std::string& output_yaml,
            const std::string& stats_yaml)
{
    
    auto start = std::chrono::steady_clock::now();

	RVO::RVOSimulator *sim = new RVO::RVOSimulator();
    setupScenario(sim);
    YAML::Node config = YAML::LoadFile(input_yaml);
    auto robots = config["robots"];

    if (!robots) {
        throw std::runtime_error("Missing 'robots' field in YAML");
    }

    create_dir_if_necessary(stats_yaml);
    std::ofstream stats(stats_yaml, std::ios::app);
    if (!stats)
    {
      std::cerr << "Failed to open stats.yaml file.\n";
      return;
    }

    std::vector<Trajectory> trajectories;

    goals.clear();
    trajectories.clear();

    // init. robots
    for (size_t i = 0; i < robots.size(); ++i) {
        auto robot = robots[i];

        auto start = robot["start"];
        auto goal  = robot["goal"];

        RVO::Vector2 start_pos(
            start[0].as<float>(),
            start[1].as<float>()
        );

        RVO::Vector2 goal_pos(
            goal[0].as<float>(),
            goal[1].as<float>()
        );

        sim->addAgent(start_pos);

        goals.push_back(goal_pos);
        trajectories.emplace_back();
    }

    // motion planning
    do {
        for (size_t i = 0; i < sim->getNumAgents(); ++i) {
            trajectories[i].positions.push_back(
                sim->getAgentPosition(i));
        }

        setPreferredVelocities(sim);
        sim->doStep();

    } while (!reachedGoal(sim));

    // save results
    saveTrajectoriesYAML(trajectories, output_yaml);

    std::cout << "RVO simulation finished with "
              << sim->getNumAgents()
              << " agents." << std::endl;

    auto end = std::chrono::steady_clock::now();
    double elapsed_sec =
      std::chrono::duration<double>(end - start).count();

    std::cout << "Time: " << elapsed_sec << " sec\n";

    // save stats
    double cost = compute_cost(trajectories);
    double makespan = compute_makespan(trajectories);
    // save stats
    stats << "stats: " << "\n";
    stats << "  - t: " << elapsed_sec << "\n";
    stats << "    cost: " << cost << "\n";
    stats << "    makespan: " << makespan << "\n";
    stats.flush();
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr << "Usage: RVOPlanner <input.yaml> <output.yaml> <stats.yaml>\n";
        return 1;
    }

    try {
        runRVO(argv[1], argv[2], argv[3]);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}