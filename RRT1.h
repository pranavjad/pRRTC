
using Configuration = std::vector<float>;


/* nearest neighbors stuff */
using Metric = nigh::LPMetric<2>;
using Space = nigh::metric::Space<Configuration, Metric>;
using State = typename Configuration;

struct NNNode {
    Configuration config;
    std::size_t idx;
};

struct KeyFn {
    const State& operator() (const NNNode& n) const {
        return n;
    }
};

using NN = nigh::Nigh<
    NNNode,
    Space,
    KeyFn,
    nigh::Concurrent,
    nigh::KDTreeBatch<>
>;
/* end nearest neighbors stuff */

