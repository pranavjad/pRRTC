# pRRTC: GPU-Parallel RRT-Connect

This repository holds the code for IROS 2025 submission "pRRTC: GPU-Parallel RRT-Connect for Fast, Consistent, and Low-Cost Motion Planning."

We introduce pRRTC, a GPU-based, parallel RRT-Connect-based algorithm. Our approach has three key improvements: 
- Concurrent sampling, expansion and connection of start and goal trees via GPU multithreading
- SIMT-optimized collision checking to quickly validate edges, inspired by the SIMD-optimized validation of [Vector-Accelerated Motion Planning](https://github.com/KavrakiLab/vamp/tree/main)
- Efficient memory management between block- and thread-level parallelism, reducing expensive memory transfer overheads

Our empirical evaluations show that pRRTC achieves up to 6x average speedup on constrained reaching tasks at high collision checking resolution. pRRTC also demonstrates a 5x reduction in solution time variance and 1.5x improvement in initial path costs compared to state-of-the-art motion planners in complex environments.

## Supported Robots and Dataset
pRRTC currently supports 7-DoF Franka Emika Panda, 8- DoF Fetch, and 14-DoF Rethink Robotics Baxter, and the repository includes the [MotionBenchMaker](https://github.com/KavrakiLab/motion_bench_maker) dataset corresponding to the three robots.

## Building

```
git clone git@github.com:CoMMALab/pRRTC.git
cmake -B build
cmake --build build
```


