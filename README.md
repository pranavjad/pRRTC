# pRRTC

pRRTC: GPU-Parallel RRT-Connect for Fast, Consistent, and Low-Cost Motion Planning.

We introduce pRRTC, a GPU-based, parallel RRT-Connect-based algorithm that achieves parallelism on multiple levels: mid-level planning iteration parallelism and low-level primitive operation parallelism with GPU-optimized memory management. Our approach has three key improvements over a baseline GPU-implementation of RRT-Connect: 
- Concurrent sampling, expansion and connection of start and goal trees via GPU multithreading
- SIMT-optimized collision checking to quickly validate edges, inspired by the SIMD-optimized validation of [Vector-Accelerated Motion Planning](https://github.com/KavrakiLab/vamp/tree/main)
- Efficient memory management between block- and thread-level parallelism, reducing expensive memory transfer overheads

```
cmake -B build
cmake --build build
build/evaluate_mbm panda test
```
