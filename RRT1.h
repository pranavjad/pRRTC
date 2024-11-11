#pragma once
#include <vector>

using Configuration = std::vector<float>;

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

template <int dim>
void solve(Configuration &start, Configuration &goal, std::vector<float> &obstacles);

extern template void solve<3>(std::vector<float>&, std::vector<float>&, std::vector<float>&);