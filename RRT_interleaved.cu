#include "RRT_interleaved.hh"
#include "Robots.hh"

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
const int MAX_SAMPLES = 10000;
const int MAX_ITERS = 10000;
const int COORD_BOUND = 50;
const float RRT_RADIUS = 5.0;

/* Collision Checking functions */
__device__ inline constexpr auto dot3(
    const float ax,
    const float ay,
    const float az,
    const float bx,
    const float by,
    const float bz) -> float
{
    return ax * bx + ay * by + az * bz;
}

__device__ inline constexpr auto sphere_sphere_sql2(
    const float ax,
    const float ay,
    const float az,
    const float ar,
    const float bx,
    const float by,
    const float bz,
    const float br) -> float
{
    const auto dx = ax - bx;
    const auto dy = ay - by;
    const auto dz = az - bz;
    const auto rsq = ar + br;
    return dot3(dx, dy, dz, dx, dy, dz) - rsq * rsq;
}

// returns squared l2 distance between two configs
__device__ float l2_dist(float *config_a, float *config_b, const int dim) {
    float ans = 0;
    float diff;
    for (int i = 0; i < dim; i++) {
        diff = config_a[i] - config_b[i];
        ans += diff * diff;
    }
    return sqrt(ans);
}

// assuming obstacles is a num_obstacles x 4 array holding x,y,z,r values
// returns true if there is a collision, otherwise false
__device__ bool sphere_robot_cc_backend(float *config, float *obstacles, int num_obstacles, float robot_radius) {
    for (int i = 0; i < num_obstacles; i++) {
        float obs_x = obstacles[i * 4];
        float obs_y = obstacles[i * 4 + 1];
        float obs_z = obstacles[i * 4 + 2];
        float obs_r = obstacles[i * 4 + 3];
        if (sphere_sphere_sql2(config[0], config[1], config[2], robot_radius, obs_x, obs_y, obs_z, obs_r) < 0)
        {
            // printf("colliding obstacle idx: %d\n", i);
            // printf("colliding obstacle: %f, %f, %f, %f\n", obs_x, obs_y, obs_z, obs_r);
            // printf("colliding config: %f, %f, %f\n", config[0], config[1], config[2]);
            return true;
        }
    }
    return false;
}
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
    int total_threads = num_samples * granularity;
    if (bid >= num_samples) return;
    if (tid_in_block >= granularity) return;
    // printf("parent idx: %d\n", new_config_parents[bid]);
    float *edge_start = &nodes[new_config_parents[bid] * dim];
    float *edge_end = &new_configs[bid * dim];
    float len = l2_dist(edge_start, edge_end, dim);

    // print_config(edge_start, dim);
    // print_config(edge_end, dim);
    // printf("len: %f\n", len);
    float delta = len / (float) granularity;

    // calculate the configuration this thread will be checking
    float config[dim];
    for (int i = 0; i < dim; i++) {
        config[i] = edge_start[i] + (tid_in_block * delta);
    }

    // check for collision
    bool config_in_collision = sphere_robot_cc_backend(config, obstacles, num_obstacles, 1.0);
    if (cc_result[bid] == false && config_in_collision) {
        atomicAdd(num_colliding_edges, 1);
        // printf("collision: %d\n", num_colliding_edges);
    }
    cc_result[bid] |= config_in_collision;
}
/* End of collision checking stuff */


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
        float dist = l2_dist(&nodes[i * dim], config, dim);
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
        float dist_to_goal = l2_dist(new_configs, goal_config, dim);
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
void solve(Robot::Configuration &start, Robot::Configuration &goal, std::vector<float> &h_obstacles) {
    static constexpr auto dim = Robot::dimension;
    std::size_t iter = 0;
    std::size_t start_index = 0;
    std::size_t free_index = start_index + 1;
    // NN tree;

    // add start to tree
    // tree.clinsert(NNNode{start_index, start});

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
    // For growing the tree we will create num_new_configs threads
    const int num_new_configs = 10;
    curandState *rng_states;
    cudaMalloc(&rng_states, num_new_configs * sizeof(curandState));
    int blockSize = 256;
    int numBlocks = (num_new_configs + blockSize - 1) / blockSize;
    init_rng<<<numBlocks, blockSize>>>(rng_states, 0);

    // create arrays on the gpu to hold the newly sampled configs, and their parents
    float *new_configs;
    cudaMalloc(&new_configs, num_new_configs * config_size);
    int *new_config_parents;
    cudaMalloc(&new_config_parents, num_new_configs * sizeof(int));

    // create an array to hold the result of collision check for each new edge
    bool *cc_result;
    cudaMalloc(&cc_result, num_new_configs * sizeof(bool));
    cudaMemset(cc_result, 0, num_new_configs);
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


    const int granularity = 32;
    bool done = false;
    // main RRT loop
    while (iter++ < MAX_ITERS && free_index < MAX_SAMPLES) {
        std::cout << iter << std::endl;
        // sample configurations and get edges to be checked
        sample_edges<Robot><<<numBlocks, blockSize>>>(new_configs, new_config_parents, nodes, goal_config, rng_states, num_new_configs);

        // collision check all the edges
        cudaMemset(cc_result, 0, num_new_configs);
        validate_edges<Robot><<<num_new_configs, granularity>>>(new_configs, new_config_parents, cc_result, num_colliding_edges, obstacles, nodes, num_obstacles, granularity, num_new_configs);

        // add all the new edges to the tree
        grow_tree<Robot><<<numBlocks, blockSize>>>(new_configs, new_config_parents, cc_result, nodes, parents, num_colliding_edges, goal_config, num_new_configs);

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

template void solve<ppln::robots::Sphere>(std::vector<float>&, std::vector<float>&, std::vector<float>&);