#include "RRT1.h"

#include <curand.h>
#include <curand_kernel.h>

__device__ int atomic_free_index;
__device__ bool reached_goal;

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

// assuming obstacles is a num_obstacles x 4 array holding x,y,z,r values
// returns true if there is a collision, otherwise false
__device__ bool sphere_robot_cc_backend(float *config, float *obstacles, int num_obstacles, float robot_radius) {
    for (int i = 0; i < num_obstacles; i++) {
        float ox = obstacles[i * 4];
        float oy = obstacles[i * 4 + 1];
        float oz = obstacles[i * 4 + 2];
        float or = obstacles[i * 4 + 3];
        if (sphere_sphere_sql2(config[0], config[1], config[2], robot_radius, ox, oy, oz, or) < 0)
        {
            return true;
        }
    }
    return false;
}



// granularity = number of interpolated points to check along each edge
// total number of threads we need is edges * granularity
// Each block is of size granularity and it checks one edge. Each thread in the block checks a consecutive interpolated point along the edge.
__global__ void validate_edges(float *new_configs, int *new_config_parents, bool *cc_result, int *num_colliding_edges, float *obstacles, float *nodes, int num_obstacles, int granularity, int num_samples, int dim) {
    int tid_in_block = threadIdx.x;
    int bid = blockIdx.x;
    int total_threads = num_samples * granularity;
    if (bid > num_samples) return;
    if (tid_in_block > granularity) return;
    float *edge_start = &nodes[new_config_parents[bid] * dim];
    float *edge_end = &new_configs[bid * dim];
    float len = sq_l2_dist(edge_start, edge_end, dim);
    float delta = len / (float) granularity;

    // calculate the configuration this thread will be checking
    float config[dim];
    for (int i = 0; i < dim; i++) {
        config[i] = edge_start[i] + (tid_in_block * delta);
    }

    // check for collision
    bool config_in_collision = sphere_robot_cc_backend(config, obstacles, num_obstacles, 1.0);
    if (cc_result[bid] == false && config_in_collision) atomicAdd(num_colliding_edges, 1)
    cc_result[bid] |= config_in_collision;
}
/* End of collision checking stuff */


const int MAX_SAMPLES = 1000;
const int MAX_ITERS = 1000;

// initialize cuda random
__global__ void init_rng(curandState* states, unsigned long seed) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    curand_init(seed, idx, 0, &states[idx]);
}

// returns squared l2 distance between two configs
__device__ float sq_l2_dist(float *config_a, float *config_b, int dim) {
    float ans = 0;
    float diff;
    for (int i = 0; i < dim; i++) {
        diff = config_a[i] - config_b[i];
        ans += diff * diff;
    }
    return ans;
}

// each thread is responsible for finding a new edge to check
// sample a new state -> connect it to nearest neighbor in our tree
__global__ void sample_edges(float *new_configs, int *new_config_parents, float *nodes, float *goal_config, curandState *rng_states, int num_samples, int dim) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid > num_samples) return;
    curandState local_rng_state = rng_states[idx];

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
            config[i] = curand_uniform(&local_rng_state);
        }
        rng_states[tid] = local_rng_state;
    }

    // find nearest neighbor
    float min_dist = 1000000000.0;
    int nearest_idx = -1;
    for (int i = 0; i < atomic_free_index; i++) {
        float dist = sq_l2_dist(nodes[i * dim], config);
        if (dist < min_dist) {
            nearest_idx = i;
            min_dist = dist;
        }
    }

    // set the parent of the new config
    new_config_parents[tid] = nearest_idx;
}

// grow the RRT tree after we figure out what edges have no collisions
// each thread is responsible for adding one edge to the tree
__global__ void grow_tree(float *new_configs, int *new_config_parents, bool *cc_result, float *nodes, int *parents, int *num_colliding_edges, int num_samples, int dim) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_samples) return;
    if (cc_result[tid]) return;  // this edge had a collision, don't add it
    
    // The first edge is always to the goal so if we get here, then the goal edge has no collision.
    if (tid == 0) { 
        reached_goal = true;
        return;
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

void solve(Configuration &start, Configuration &goal) {
    std::vector<std::size_t> parents(MAX_SAMPLES);
    std::vector<Configuration> nodes(MAX_SAMPLES);
    std::size_t iter = 0;
    std::size_t start_index = 0;
    std::size_t free_index = start_index + 1;
    NN tree;

    // add start to tree
    nodes[start_index] = start;
    tree.insert(NNNode{start_index, start});

    // copy stuff to GPU
    // GPU needs: start, goal, tree, parents, nodes
    std::size_t dim = start.size();
    float *start_config;
    float *goal_config;
    float *nodes;
    int *parents;
    const std::size_t config_size = dim * sizeof(float);
    cudaMalloc(&start_config, config_size);
    cudaMalloc(&goal_config, config_size);
    cudaMalloc(&nodes, MAX_SAMPLES * config_size);
    cudaMalloc(&parents, MAX_SAMPLES * sizeof(int))
    cudaMemcpy(start_config, start.data(), config_size, cudaMemcpyHostToDevice);
    cudaMemcpy(goal_config, goal.data(), config_size, cudaMemcpyHostToDevice);
    // add the start config to the tree, and set the start to be it's own parent.
    cudaMemcpy(nodes, start.data(), config_size, cudaMemcpyHostToDevice);
    cudaMemcpy(parents, &start_index, sizeof(int), cudaMemcpyHostToDevice);

    // create a curandState for each thread -> holds state of RNG for each thread seperately
    const int num_new_configs = 100;
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

    // get free index ready
    cudaMemcpyToSymbol(atomic_free_index, &initial_free_index, sizeof(int));


    const granularity = 256;
    bool done = false;
    // main RRT loop
    while (iter++ < MAX_ITERS && free_index < MAX_SAMPLES) {
        // sample configurations and get edges to be checked
        sample_edges<<<numBlocks, blockSize>>>(new_configs, new_config_parents, nodes, goal_config,  rng_states, num_new_configs, dim);

        // collision check all the edges
        cudaMemset(cc_result, 0, num_new_configs);
        validate_edges<<<num_new_configs, granularity>>>(new_configs, new_config_parents, cc_result, num_colliding_edges, obstacles, nodes, num_obstacles, granularity, num_new_configs, dim);

        // add all the new edges to the tree
        grow_tree<<<numBlocks, blockSize>>>(new_configs, new_config_parents, cc_result, nodes, parents, num_colliding_edges, num_samples, dim);

        // update free index
        cudaMemcpyFromSymbol(&free_index, atomic_free_index, sizeof(int));
        cudaMemcpyFromSymbol(&done, &goal_reached, sizeof(bool));
        if (done) break;
    }

}