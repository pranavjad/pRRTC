#include "RRT_interleaved.hh"
#include "Robots.hh"
#include "collision_backends.cuh"
#include "collision/environment.hh"

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
__device__ int found_goal_idx;

const int MAX_SAMPLES = 1000000;
const int MAX_ITERS = 1000000;
const int COORD_BOUND = 3.0;
const int NUM_NEW_CONFIGS = 1024;
const int GRANULARITY = 4096;
const float RRT_RADIUS = 2.0;

using namespace ppln;

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
__global__ void validate_edges(float *new_configs, int *new_config_parents, float *new_config_dist, bool *cc_result, int *num_colliding_edges, ppln::collision::Environment<float> *env, float *nodes, int granularity, int num_samples) {
    static constexpr auto dim = Robot::dimension;
    int tid_in_block = threadIdx.x;
    int bid = blockIdx.x;
    // total_threads = num_samples * granularity;
    if (bid >= num_samples) return;
    if (tid_in_block >= granularity) return;
    
    __shared__ float len;
    __shared__ float shared_edge_start[dim];
    if (tid_in_block == 0) {
        float *edge_start = &nodes[new_config_parents[bid] * dim];
        len = new_config_dist[bid];
        for (int i = 0; i < dim; i++) {
            shared_edge_start[i] = edge_start[i];
        }
    }
    __syncthreads();



    float delta = len / (float) granularity;

    // calculate the configuration this thread will be checking
    float config[dim];
    for (int i = 0; i < dim; i++) {
        config[i] = shared_edge_start[i] + (tid_in_block * delta);
    }

    // check for collision
    bool config_in_collision = not ppln::collision::fkcc<Robot>(config, env);
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
__global__ void sample_edges(float *new_configs, int *new_config_parents, float *new_config_dist, float *nodes, float *goal_configs, int num_goals, curandState *rng_states, int num_samples) {
    static constexpr auto dim = Robot::dimension;
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_samples) return;
    curandState local_rng_state = rng_states[tid];

    float *config = &new_configs[tid * dim];

    // if this is the first thread always sample a random goal
    if (tid == 0) {
        int goal_idx = (int)(num_goals * curand_uniform(&local_rng_state));
        float *goal_config = goal_configs + (dim * sizeof(float) * goal_idx);
        for (int i = 0; i < dim; i++) {
            config[i] = goal_config[i];
        }
    }
    // otherwise sample a random config
    else {
        for (int i = 0; i < dim; i++) {
            config[i] = (2 * COORD_BOUND * curand_uniform(&local_rng_state)) - COORD_BOUND;
        }
        rng_states[tid] = local_rng_state;
    }

    // find nearest neighbor
    float min_dist = 1000000000.0;
    int nearest_idx = -1;
    float dist;
    for (int i = 0; i < atomic_free_index; i++) {
        dist = device_utils::l2_dist(&nodes[i * dim], config, dim);
        if (dist < min_dist) {
            nearest_idx = i;
            min_dist = dist;
        }
    }

    // if (tid == 0) {
    //     printf("%d\n", atomic_free_index);
    //     printf("%f\n", min_dist);
    // }

    // keep it within the rrt range
    float scale = min(1.0f, RRT_RADIUS / min_dist);
    for (int i = 0; i < dim; i++) {
        config[i] *= scale;
    }
    min_dist *= scale;
    
    new_config_dist[tid] = min_dist;

    // set the parent of the new config
    new_config_parents[tid] = nearest_idx;
}

// grow the RRT tree after we figure out what edges have no collisions
// each thread is responsible for adding one edge to the tree
template <typename Robot>
__global__ void grow_tree(float *new_configs, int *new_config_parents, float *new_config_dist, bool *cc_result, float *nodes, int *parents, int *num_colliding_edges, float *goal_configs, int num_goals, int num_samples) {
    static constexpr auto dim = Robot::dimension;
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_samples) return;
    if (cc_result[tid]) return;  // this edge had a collision, don't add it

    // The first edge is always to the goal so if we get here, we need to check if we reached the goal.
    if (tid == 0) {
        int goal_size = dim * sizeof(float);
        float dist_to_goal;
        for (int i = 0; i < num_goals; i++) {
            dist_to_goal = device_utils::l2_dist(new_configs, &goal_configs[i * goal_size], dim);
            if (dist_to_goal < 0.0001) {
                reached_goal = true;
                found_goal_idx = i;
                return;
            }
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

inline void reset_device_variables() {
    int zero = 0;
    bool false_val = false;
    
    cudaMemcpyToSymbol(atomic_free_index, &zero, sizeof(int));
    cudaMemcpyToSymbol(reached_goal, &false_val, sizeof(bool));
    cudaMemcpyToSymbol(found_goal_idx, &zero, sizeof(int));
}


template <typename Robot>
PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goals, ppln::collision::Environment<float> &h_environment) {
    static constexpr auto dim = Robot::dimension;
    std::size_t iter = 0;
    std::size_t start_index = 0;
    std::size_t free_index = start_index + 1;

    auto start_time = std::chrono::steady_clock::now();
    PlannerResult<Robot> res;

    // copy stuff to GPU
    // GPU needs: start, goal, tree, parents, nodes

    float *start_config;
    float *goal_configs;
    int num_goals = goals.size();
    float *nodes;
    int *parents;
    const std::size_t config_size = dim * sizeof(float);
    cudaMalloc(&start_config, config_size);
    cudaMalloc(&goal_configs, config_size * num_goals);
    cudaMalloc(&nodes, MAX_SAMPLES * config_size);
    cudaMalloc(&parents, MAX_SAMPLES * sizeof(int));
    cudaMemcpy(start_config, start.data(), config_size, cudaMemcpyHostToDevice);
    cudaMemcpy(goal_configs, goals.data(), config_size, cudaMemcpyHostToDevice);
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

    // create arrays on the gpu to hold the newly sampled configs, and their parents, and dist to parent
    float *new_configs;
    cudaMalloc(&new_configs, NUM_NEW_CONFIGS * config_size);
    int *new_config_parents;
    cudaMalloc(&new_config_parents, NUM_NEW_CONFIGS * sizeof(int));
    float *new_config_dist;
    cudaMalloc(&new_config_dist, NUM_NEW_CONFIGS * sizeof(float));
    cudaMemset(new_config_dist, 0, NUM_NEW_CONFIGS * sizeof(float));

    // create an array to hold the result of collision check for each new edge
    bool *cc_result;
    cudaMalloc(&cc_result, NUM_NEW_CONFIGS * sizeof(bool));
    cudaMemset(cc_result, 0, NUM_NEW_CONFIGS);
    int *num_colliding_edges;
    cudaMalloc(&num_colliding_edges, sizeof(int));

    // free index for next available position in the nodes array
    cudaMemcpyToSymbol(atomic_free_index, &free_index, sizeof(int));

    // allocate for obstacles
    ppln::collision::Environment<float> *env;
    cudaMalloc(&env, sizeof(env));
    cudaMemcpy(env, &h_environment, sizeof(env), cudaMemcpyHostToDevice);


    bool done = false;
    // main RRT loop
    while (iter++ < MAX_ITERS && free_index < MAX_SAMPLES) {
        // std::cout << iter << std::endl;
        // sample configurations and get edges to be checked
        sample_edges<Robot><<<numBlocks, blockSize>>>(new_configs, new_config_parents, new_config_dist, nodes, goal_configs, num_goals, rng_states, NUM_NEW_CONFIGS);
        cudaDeviceSynchronize();
        // collision check all the edges
        cudaMemset(cc_result, 0, NUM_NEW_CONFIGS);
        validate_edges<Robot><<<NUM_NEW_CONFIGS, GRANULARITY>>>(new_configs, new_config_parents, new_config_dist, cc_result, num_colliding_edges, env, nodes, GRANULARITY, NUM_NEW_CONFIGS);
        cudaDeviceSynchronize();
        // add all the new edges to the tree
        grow_tree<Robot><<<numBlocks, blockSize>>>(new_configs, new_config_parents, new_config_dist, cc_result, nodes, parents, num_colliding_edges, goal_configs, num_goals, NUM_NEW_CONFIGS);
        cudaDeviceSynchronize();
        
        // update free index
        cudaMemcpyFromSymbol(&free_index, atomic_free_index, sizeof(int), 0, cudaMemcpyDeviceToHost);
        cudaMemcpyFromSymbol(&done, reached_goal, sizeof(bool), 0, cudaMemcpyDeviceToHost);
        if (done) break;
    }
    res.tree_size = free_index;
    res.iters = iter;
    res.attempted_tree_size = NUM_NEW_CONFIGS * iter;
    // retrieve data from gpu
    std::vector<int> h_parents(MAX_SAMPLES);
    std::vector<float> h_nodes(MAX_SAMPLES * dim);
    cudaMemcpy(h_parents.data(), parents, MAX_SAMPLES * sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_nodes.data(), nodes, MAX_SAMPLES * config_size, cudaMemcpyDeviceToHost);

    // change representation of RRT nodes from flattened vector to vector of Configurations
    typename Robot::Configuration cfg;
    for (int i = 0; i < MAX_SAMPLES; i++) {
        for (int j = 0; j < dim; j++) {
            cfg[j] = h_nodes[i * dim + j];
        }
        res.nodes.emplace_back(cfg);
    }

    // compute the path
    // std::vector<int> path;
    
    if (done) {
        // get the index of the goal we found in the goals array
        int h_goal_idx;
        cudaMemcpyFromSymbol(&h_goal_idx, found_goal_idx, sizeof(int), 0, cudaMemcpyDeviceToHost);
        // std::cout << "Found Goal: " << h_goal_idx << std::endl;
        res.solved = true;
        // get parent at position 0 in new_config_parents, because that will be the parent of the goal
        int parent_idx = -1;
        cudaMemcpy(&parent_idx, new_config_parents, sizeof(int), cudaMemcpyDeviceToHost);
        assert(parent_idx != -1);
        typename Robot::Configuration cfg;
        typename Robot::Configuration cfg_parent;
        std::copy_n(h_nodes.begin() + parent_idx, dim, cfg.begin());
        res.cost += l2dist<Robot>(goals[h_goal_idx], cfg);
        // std::cout << "\n----\n";
        // std::cout << "goal: ";
        // for (int i = 0; i < goals[h_goal_idx].size(); i++) std::cout << goals[h_goal_idx][i] << " ";
        // std::cout << '\n';
        while (parent_idx != h_parents[parent_idx]) {
            // std::cout << parent_idx << std::endl;
            std::copy_n(h_nodes.begin() + parent_idx, dim, cfg.begin());
            std::copy_n(h_nodes.begin() + h_parents[parent_idx], dim, cfg_parent.begin());

            // for (int i = 0; i < cfg.size(); i++) std::cout << cfg[i] << " ";
            // std::cout << '\n';
            // for (int i = 0; i < cfg.size(); i++) std::cout << cfg_parent[i] << " ";
            // std::cout << '\n';

            // std::cout << l2dist<Robot>(cfg, cfg_parent) << "\n";
            res.cost += l2dist<Robot>(cfg, cfg_parent);
            res.path.emplace_back(parent_idx);
            parent_idx = h_parents[parent_idx];
        }

        // std::copy_n(h_nodes.begin() + parent_idx, dim, cfg.begin());
        // std::copy_n(h_nodes.begin() + h_parents[parent_idx], dim, cfg_parent.begin());
        // for (int i = 0; i < cfg.size(); i++) std::cout << cfg[i] << " ";
        // std::cout << '\n';
        // for (int i = 0; i < cfg.size(); i++) std::cout << cfg_parent[i] << " ";
        // std::cout << "\nstart: ";
        // for (int i = 0; i < cfg.size(); i++) std::cout << start[i] << " ";
        // std::cout << '\n';
        // std::cout << "\n----\n";
        res.path.emplace_back(parent_idx);
        std::reverse(res.path.begin(), res.path.end());
    }
    res.nanoseconds = get_elapsed_nanoseconds(start_time);
    reset_device_variables();
    cudaFree(start_config);
    cudaFree(goal_configs);
    cudaFree(nodes);
    cudaFree(parents);
    cudaFree(rng_states);
    cudaFree(new_configs);
    cudaFree(new_config_parents);
    cudaFree(cc_result);
    cudaFree(num_colliding_edges);
    cudaFree(env);
    return res;
}

template PlannerResult<typename ppln::robots::Sphere> solve<ppln::robots::Sphere>(std::array<float, 3>&, std::vector<std::array<float, 3>>&, ppln::collision::Environment<float>&);
template PlannerResult<typename ppln::robots::Panda> solve<ppln::robots::Panda>(std::array<float, 7>&, std::vector<std::array<float, 7>>&, ppln::collision::Environment<float>&);