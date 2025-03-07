# pRRTC: GPU-Parallel RRT-Connect

This repository holds the code for IROS 2025 submission "pRRTC: GPU-Parallel RRT-Connect for Fast, Consistent, and Low-Cost Motion Planning."

We introduce pRRTC, a GPU-based, parallel RRT-Connect-based algorithm. Our approach has three key improvements: 
- Concurrent sampling, expansion and connection of start and goal trees via GPU multithreading
- SIMT-optimized collision checking to quickly validate edges, inspired by the SIMD-optimized validation of [Vector-Accelerated Motion Planning](https://github.com/KavrakiLab/vamp/tree/main)
- Efficient memory management between block- and thread-level parallelism, reducing expensive memory transfer overheads

pRRTC shows up to 6x average speedup on constrained reaching tasks at high collision checking resolution, 5x reduction in solution time variance, and 1.5x improvement in initial path costs compared to state-of-the-art motion planners in complex environments.


```
cmake -B build
cmake --build build
build/evaluate_mbm panda test
```
