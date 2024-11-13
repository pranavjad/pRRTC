#pragma once

#include <vector>
#include <array>

#include "Robots.hh"


/* nearest neighbors stuff */
// using Metric = nigh::LPMetric<2>;
// using Space = nigh::metric::Space<Configuration, Metric>;
// using State = typename Configuration;

// struct NNNode {
//     Configuration config;
//     std::size_t idx;
// };

// struct KeyFn {
//     const State& operator() (const NNNode& n) const {
//         return n;
//     }
// };

// using NN = nigh::Nigh<
//     NNNode,
//     Space,
//     KeyFn,
//     nigh::Concurrent,
//     nigh::KDTreeBatch<>
// >;
/* end nearest neighbors stuff */

template <typename Robot>
void solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goal, std::vector<float> &obstacles);

extern template void solve<ppln::robots::Sphere>(std::array<float, 3>&, std::vector<std::array<float, 3>>&, std::vector<float>&);
extern template void solve<ppln::robots::Panda>(std::array<float, 7>&, std::vector<std::array<float, 7>>&, std::vector<float>&);