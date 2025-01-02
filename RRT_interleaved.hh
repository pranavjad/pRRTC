#pragma once

#include <vector>
#include <array>
#include <chrono>

#include "Robots.hh"
#include "collision/environment.hh"

template <typename Robot>
struct PlannerResult {
    bool solved = false;
    std::vector<int> path; // path found by planner by node index
    std::vector<typename Robot::Configuration> nodes; // all nodes in the RRT
    int tree_size = 0;
    int iters = 0;
    float cost = 0.0;
    std::size_t nanoseconds = 0;
    std::size_t attempted_tree_size = 0;
};

template <typename Robot>
inline float l2dist(typename Robot::Configuration &a, typename Robot::Configuration &b)
{
    float res = 0;
    float diff;
    for (int i = 0; i < a.size(); i++) {
        diff = a[i] - b[i];
        res += diff * diff;
    }
    return sqrt(res);
}

inline std::size_t get_elapsed_nanoseconds(const std::chrono::time_point<std::chrono::steady_clock> &start)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
}

namespace RRT {
    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goal, ppln::collision::Environment<float> &environment);
    // template PlannerResult<typename ppln::robots::Sphere> solve<ppln::robots::Sphere>(std::array<float, 3>&, std::vector<std::array<float, 3>>&, ppln::collision::Environment<float> &environment);
    // template PlannerResult<typename ppln::robots::Panda> solve<ppln::robots::Panda>(std::array<float, 7>&, std::vector<std::array<float, 7>>&, ppln::collision::Environment<float> &environment);
}

namespace RRTC {
    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goal, ppln::collision::Environment<float> &environment);
    // template PlannerResult<typename ppln::robots::Sphere> solve<ppln::robots::Sphere>(std::array<float, 3>&, std::vector<std::array<float, 3>>&, ppln::collision::Environment<float> &environment);
    // template PlannerResult<typename ppln::robots::Panda> solve<ppln::robots::Panda>(std::array<float, 7>&, std::vector<std::array<float, 7>>&, ppln::collision::Environment<float> &environment);
}

namespace RRT_new {
    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goal, ppln::collision::Environment<float> &environment);
}