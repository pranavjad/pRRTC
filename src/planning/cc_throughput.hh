#pragma once


namespace bench_cc {
    template <typename Robot>
    size_t test_cc_throughput(ppln::collision::Environment<float> &h_environment, int num_blocks, int granularity, int edges_per_block);
} // namespace bench_cc