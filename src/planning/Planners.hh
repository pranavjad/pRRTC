#pragma once

#include <vector>
#include <array>
#include <chrono>

#include "Robots.hh"
#include "src/collision/environment.hh"
#include "pRRTC_settings.hh"

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

template <typename Robot>
inline void print_cfg(float *config) {
    for (int i = 0; i < Robot::dimension; i++) {
        std::cout << config[i] << " ";
    }
    std::cout << "\n";
}

inline std::size_t get_elapsed_nanoseconds(const std::chrono::time_point<std::chrono::steady_clock> &start)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
}

/* This file handles the declarations of each solve function so that they may be called from .cpp files. Implementation is in .cu files.*/
namespace RRT {
    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goals, ppln::collision::Environment<float> &environment);
}

namespace pRRT {
    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goals, ppln::collision::Environment<float> &environment);
}


namespace nRRT {
    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goals, ppln::collision::Environment<float> &environment);
}

namespace pRRTC {
    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goals, ppln::collision::Environment<float> &environment, pRRTC_settings &settings);
}