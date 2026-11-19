#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include <fstream>
#include <string>
#include "Vector2.h"
#include <filesystem>

namespace fs = std::filesystem;

struct Trajectory {
    std::vector<RVO::Vector2> positions;
    std::vector<RVO::Vector2> velocities;

};

// save only position for now
inline void saveTrajectoriesYAML(
    const std::vector<Trajectory>& trajectories,
    const std::string& filename)
{
    std::ofstream out(filename);

    out << "result:\n";

    for (const auto& traj : trajectories) {

        out << "-\n";
        out << "  num_states: "
            << traj.positions.size() << "\n";

        out << "  states:\n";

        for (const auto& state : traj.positions) {

            out << "    - ["
                << state.x() << ", "
                << state.y() << "]\n";
        }
    }
}

// for stats
double compute_cost(const std::vector<Trajectory>& trajectories)
{
    double cost = 0.0;

    for (const auto& traj : trajectories) {
        cost += static_cast<double>(traj.positions.size());
    }

    return (cost - 1) * 0.1;
}

size_t compute_makespan(const std::vector<Trajectory>& trajectories)
{
    size_t makespan = 0;

    for (const auto& traj : trajectories) {
        makespan = std::max(makespan, (traj.positions.size() - 1));
    }

    return makespan * 0.1;
}

void create_dir_if_necessary(const std::string& file_path)
{

  fs::path path(file_path);
  if (path.has_parent_path()) {
      fs::create_directories(path.parent_path());
  }
}

// generate instance.yaml ONLY for proof of concept
inline std::string vec2ToYAML(const RVO::Vector2 &v)
{
    return "[" +
        std::to_string(v.x()) + ", " +
        std::to_string(v.y()) + "]";
}

inline std::string stateToYAML(const RVO::Vector2 &pos)
{
    return "[" +
        std::to_string(pos.x()) + ", " +
        std::to_string(pos.y()) + ", 0, 0]";
}

inline void writeInstanceYAML(
    const std::vector<Trajectory> &trajectories,
    const std::string &filename)
{
    if (trajectories.empty()) return;

    std::ofstream out(filename);

    // ---- environment (hardcoded) ----
    out << "environment:\n";
    out << "  min: [-20, -20]\n";
    out << "  max: [20, 20]\n";
    out << "  obstacles: []\n\n";

    // ---- robots ----
    out << "robots:\n";

    for (const auto &tr : trajectories) {
        if (tr.positions.empty()) continue;

        const RVO::Vector2 &start_pos = tr.positions.front();
        const RVO::Vector2 &goal_pos  = tr.positions.back();

        out << "  - type: integrator2_2d_v0\n";
        out << "    start: " << stateToYAML(start_pos) << "\n";
        out << "    goal: "  << stateToYAML(goal_pos)  << "\n\n";
    }

    out.close();
}




#endif