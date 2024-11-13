#include "RRT_interleaved.hh"
#include "Robots.hh"
#include "collision_backends.cu"

#include <curand.h>
#include <curand_kernel.h>

#include <vector>
#include <iostream>
#include <cassert>
#include <algorithm>

/*
Parallelized RRT with parallelized collision checking.
Interleaved strategy: sample states in parallel, then check edges in parallel, then repeat.
*/

__device__ int atomic_free_index;
__device__ bool reached_goal = false;
const int MAX_SAMPLES = 1000000;
const int MAX_ITERS = 1000000;
const int COORD_BOUND = 2.0;
const int NUM_NEW_CONFIGS = 1024;
const int GRANULARITY = 1024;
const float RRT_RADIUS = 1.0;


__device__ inline void print_config(float *config, int dim) {
    for (int i = 0; i < dim; i++) {
        printf("%f ", config[i]);
    }
    printf("\n");
}

// granularity = number of interpolated points to check along each edge
// total number of threads we need is edges * granularity
// Each block is of size granularity and it checks one edge. Each thread in the block checks a consecutive interpolated point along the edge.
template <typename Robot>
__global__ void validate_edges(float *new_configs, int *new_config_parents, bool *cc_result, int *num_colliding_edges, float *obstacles, float *nodes, int num_obstacles, int granularity, int num_samples) {
    static constexpr auto dim = Robot::dimension;
    int tid_in_block = threadIdx.x;
    int bid = blockIdx.x;
    // total_threads = num_samples * granularity;
    if (bid >= num_samples) return;
    if (tid_in_block >= granularity) return;

    float *edge_start = &nodes[new_config_parents[bid] * dim];
    float *edge_end = &new_configs[bid * dim];
    float len = utils::l2_dist(edge_start, edge_end, dim);


    float delta = len / (float) granularity;

    // calculate the configuration this thread will be checking
    float config[dim];
    for (int i = 0; i < dim; i++) {
        config[i] = edge_start[i] + (tid_in_block * delta);
    }

    // check for collision
    bool config_in_collision = not collision::fkcc<Robot>(config, obstacles, num_obstacles);
    if (cc_result[bid] == false && config_in_collision) {
        atomicAdd(num_colliding_edges, 1);
        // printf("collision: %d\n", num_colliding_edges);
    }
    cc_result[bid] |= config_in_collision;
}


// initialize cuda random
__global__ void init_rng(curandState* states, unsigned long seed) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    curand_init(seed, idx, 0, &states[idx]);
}

// each thread is responsible for finding a new edge to check
// sample a new state -> connect it to nearest neighbor in our tree
template <typename Robot>
__global__ void sample_edges(float *new_configs, int *new_config_parents, float *nodes, float *goal_config, curandState *rng_states, int num_samples) {
    static constexpr auto dim = Robot::dimension;
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_samples) return;
    curandState local_rng_state = rng_states[tid];

    float *config = &new_configs[tid * dim];

    // if this is the first thread always sample the goal
    if (tid == 0) {
        for (int i = 0; i < dim; i++) {
            config[i] = goal_config[i];
        }
    }
    // otherwise sample a random config
    else {
        for (int i = 0; i < dim; i++) {
            config[i] = (2 * COORD_BOUND * curand_uniform(&local_rng_state)) - COORD_BOUND;
            // printf("%f, ", config[i]);
        }
        // printf("\n");
        rng_states[tid] = local_rng_state;
    }

    // find nearest neighbor
    float min_dist = 1000000000.0;
    int nearest_idx = -1;
    for (int i = 0; i < atomic_free_index; i++) {
        float dist = utils::l2_dist(&nodes[i * dim], config, dim);
        if (dist < min_dist) {
            nearest_idx = i;
            min_dist = dist;
        }
    }

    if (tid == 0) {
        printf("%d\n", atomic_free_index);
        printf("%f\n", min_dist);
    }

    // keep it within the rrt range
    if (min_dist > RRT_RADIUS) {
        float scale = RRT_RADIUS / min_dist;
        // printf("min dist: %f\n", min_dist);
        for (int i = 0; i < dim; i++) {
            config[i] *= scale;
        }
    }
    // printf("dist to new cfg: %f\n", l2_dist(config, &nodes[nearest_idx * dim], dim));

    // set the parent of the new config
    new_config_parents[tid] = nearest_idx;
}

// grow the RRT tree after we figure out what edges have no collisions
// each thread is responsible for adding one edge to the tree
template <typename Robot>
__global__ void grow_tree(float *new_configs, int *new_config_parents, bool *cc_result, float *nodes, int *parents, int *num_colliding_edges, float *goal_config, int num_samples) {
    static constexpr auto dim = Robot::dimension;
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_samples) return;
    if (cc_result[tid]) return;  // this edge had a collision, don't add it
    // printf("growing tree!\n");
    // The first edge is always to the goal so if we get here, we need to check if we reached the goal.
    if (tid == 0) {
        float dist_to_goal = utils::l2_dist(new_configs, goal_config, dim);
        if (dist_to_goal < 0.001) {
            reached_goal = true;
            return;
        }
    }

    // Atomically get the next free index
    int my_index = atomicAdd(&atomic_free_index, 1);
    
    // Copy the configuration to the nodes array
    for (int i = 0; i < dim; i++) {
        nodes[my_index * dim + i] = new_configs[tid * dim + i];
    }
    
    // Set the parent
    parents[my_index] = new_config_parents[tid];
}

template <typename Robot>
void solve(typename Robot::Configuration &start, typename Robot::Configuration &goal, std::vector<float> &h_obstacles) {
    static constexpr auto dim = Robot::dimension;
    std::size_t iter = 0;
    std::size_t start_index = 0;
    std::size_t free_index = start_index + 1;

    // copy stuff to GPU
    // GPU needs: start, goal, tree, parents, nodes
    // const int dim = start.size();
    // constexpr int dim = DIM; 

    float *start_config;
    float *goal_config;
    float *nodes;
    int *parents;
    const std::size_t config_size = dim * sizeof(float);
    cudaMalloc(&start_config, config_size);
    cudaMalloc(&goal_config, config_size);
    cudaMalloc(&nodes, MAX_SAMPLES * config_size);
    cudaMalloc(&parents, MAX_SAMPLES * sizeof(int));
    cudaMemcpy(start_config, start.data(), config_size, cudaMemcpyHostToDevice);
    cudaMemcpy(goal_config, goal.data(), config_size, cudaMemcpyHostToDevice);
    // add the start config to the tree, and set the start to be it's own parent.
    cudaMemcpy(nodes, start.data(), config_size, cudaMemcpyHostToDevice);
    cudaMemcpy(parents, &start_index, sizeof(int), cudaMemcpyHostToDevice);

    // create a curandState for each thread -> holds state of RNG for each thread seperately
    // For growing the tree we will create NUM_NEW_CONFIGS threads
    curandState *rng_states;
    cudaMalloc(&rng_states, NUM_NEW_CONFIGS * sizeof(curandState));
    int blockSize = 256;
    int numBlocks = (NUM_NEW_CONFIGS + blockSize - 1) / blockSize;
    init_rng<<<numBlocks, blockSize>>>(rng_states, 0);

    // create arrays on the gpu to hold the newly sampled configs, and their parents
    float *new_configs;
    cudaMalloc(&new_configs, NUM_NEW_CONFIGS * config_size);
    int *new_config_parents;
    cudaMalloc(&new_config_parents, NUM_NEW_CONFIGS * sizeof(int));

    // create an array to hold the result of collision check for each new edge
    bool *cc_result;
    cudaMalloc(&cc_result, NUM_NEW_CONFIGS * sizeof(bool));
    cudaMemset(cc_result, 0, NUM_NEW_CONFIGS);
    int *num_colliding_edges;
    cudaMalloc(&num_colliding_edges, sizeof(int));

    // free index for next available position in the nodes array
    cudaMemcpyToSymbol(atomic_free_index, &free_index, sizeof(int));

    // allocate for obstacles
    float *obstacles;
    assert(h_obstacles.size() % 4 == 0);
    int num_obstacles = h_obstacles.size() / 4;
    std::size_t obstacles_size = h_obstacles.size() * sizeof(float);
    cudaMalloc(&obstacles, obstacles_size);
    cudaMemcpy(obstacles, h_obstacles.data(), obstacles_size, cudaMemcpyHostToDevice);


    bool done = false;
    // main RRT loop
    while (iter++ < MAX_ITERS && free_index < MAX_SAMPLES) {
        std::cout << iter << std::endl;
        // sample configurations and get edges to be checked
        sample_edges<Robot><<<numBlocks, blockSize>>>(new_configs, new_config_parents, nodes, goal_config, rng_states, NUM_NEW_CONFIGS);

        // collision check all the edges
        cudaMemset(cc_result, 0, NUM_NEW_CONFIGS);
        validate_edges<Robot><<<NUM_NEW_CONFIGS, GRANULARITY>>>(new_configs, new_config_parents, cc_result, num_colliding_edges, obstacles, nodes, num_obstacles, GRANULARITY, NUM_NEW_CONFIGS);

        // add all the new edges to the tree
        grow_tree<Robot><<<numBlocks, blockSize>>>(new_configs, new_config_parents, cc_result, nodes, parents, num_colliding_edges, goal_config, NUM_NEW_CONFIGS);

        // update free index
        cudaMemcpyFromSymbol(&free_index, atomic_free_index, sizeof(int), 0, cudaMemcpyDeviceToHost);
        cudaMemcpyFromSymbol(&done, reached_goal, sizeof(bool), 0, cudaMemcpyDeviceToHost);
        if (done) break;
    }

    // retrieve data from gpu
    std::vector<int> h_parents(MAX_SAMPLES);
    std::vector<float> h_nodes(MAX_SAMPLES * dim);
    cudaMemcpy(h_parents.data(), parents, MAX_SAMPLES * sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_nodes.data(), nodes, MAX_SAMPLES * config_size, cudaMemcpyDeviceToHost);

    std::vector<int> path;

    if (done) {
        std::cout << "Found Goal!" << std::endl;

        // get parent at position 0 in new_config_parents, because that will be the parent of the goal
        int parent_idx = -1;
        cudaMemcpy(&parent_idx, new_config_parents, sizeof(int), cudaMemcpyDeviceToHost);
        assert(parent_idx != -1);
        std::cout << parent_idx << std::endl;
        while (parent_idx != h_parents[parent_idx]) {
            std::cout << parent_idx << std::endl;
            path.emplace_back(parent_idx);
            parent_idx = h_parents[parent_idx];
        }
        path.emplace_back(parent_idx);
        std::reverse(path.begin(), path.end());
    }

    cudaFree(start_config);
    cudaFree(goal_config);
    cudaFree(nodes);
    cudaFree(parents);
    cudaFree(rng_states);
    cudaFree(new_configs);
    cudaFree(new_config_parents);
    cudaFree(cc_result);
    cudaFree(num_colliding_edges);
    cudaFree(obstacles);
}

template void solve<ppln::robots::Sphere>(std::array<float, 3>&, std::array<float, 3>&, std::vector<float>&);
template void solve<ppln::robots::Panda>(std::array<float, 7>&, std::array<float, 7>&, std::vector<float>&);