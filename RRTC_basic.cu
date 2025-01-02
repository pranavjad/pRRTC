#include "RRT_interleaved.hh"
#include "Robots.hh"
#include "utils.cuh"
#include "collision/environment.hh"

#include <curand.h>
#include <curand_kernel.h>

#include <vector>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <numeric>

/*
Parallelized RRTC with parallelized collision checking.
Interleaved strategy: sample states in parallel, then check edges in parallel, then repeat.
*/

namespace RRTC {

    __global__ void init_rng(curandState* states, unsigned long seed) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        curand_init(seed, idx, 0, &states[idx]);
    }

    __device__ int atomic_free_index;
    __device__ bool reached_goal = false;
    __device__ int found_goal_idx;

    constexpr int MAX_SAMPLES = 1000000;
    constexpr int MAX_ITERS = 1000000;
    constexpr int COORD_BOUND = 3.0;
    constexpr int NUM_NEW_CONFIGS = 8;
    constexpr int GRANULARITY = 8;
    constexpr float RRT_RADIUS = 2.0;

    // threads per block for sample_edges and grow_tree
    constexpr int BLOCK_SIZE = 256;

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
    __global__ void validate_edges(float *new_configs, int *new_config_parents, float *new_config_dist, bool *cc_result, ppln::collision::Environment<float> *env, float *nodes) {
        printf("num spheres, capsules, cuboids: %d, %d, %d\n", env->num_spheres, env->num_capsules, env->num_cuboids);
        static constexpr auto dim = Robot::dimension;
        int tid_in_block = threadIdx.x;
        int bid = blockIdx.x;
        printf("val_edges b:%d, t:%d\n", bid, tid_in_block);
        // total_threads = num_samples * granularity;
        if (bid >= 2 * NUM_NEW_CONFIGS) return;
        if (tid_in_block >= GRANULARITY) return;
        // printf("called validated_edges_%d_%d\n", bid, tid_in_block);

        __shared__ float delta[dim];
        __shared__ float shared_edge_start[dim];
        if (tid_in_block == 0) {
            // if (bid > NUM_NEW_CONFIGS)
                printf("Block %d checking edge from parent %d (tree %c)\n", 
                    bid, new_config_parents[bid], 
                    (bid < NUM_NEW_CONFIGS) ? 'A' : 'B');
            // printf("Block %d checking edge from parent %d, %d\n", bid, new_config_parents[bid], new_config_parents[(bid + NUM_NEW_CONFIGS) % (2 * NUM_NEW_CONFIGS)]);
            float *edge_start = &nodes[new_config_parents[bid] * dim];
            float *edge_end = &new_configs[bid * dim];
            printf("Block %d calculating delta with dim %d\n", bid, dim);
            for (int i = 0; i < dim; i++) {
                shared_edge_start[i] = edge_start[i];
                delta[i] = (edge_end[i] - edge_start[i]) / (float) GRANULARITY;
                printf("%f ", delta[i]);
            }
            printf("\nBlock %d done with initial calcs\n", bid);
        }
        __syncthreads();


        if (tid_in_block == 0) printf("\nBlock %d calcing config to check\n", bid);
        // calculate the configuration this thread will be checking
        float config[dim];
        for (int i = 0; i < dim; i++) {
            config[i] = shared_edge_start[i] + (tid_in_block * delta[i]);
        }

        if (tid_in_block == 0) printf("\nBlock %d cc\n", bid);
        // check for collision
        bool config_in_collision = not ppln::collision::fkcc<Robot>(config, env);
        if (tid_in_block == 0) printf("\nBlock %d cc done\n", bid);
        cc_result[bid] |= config_in_collision;
        if (tid_in_block == 0) printf("\nBlock %d done\n", bid);
    }

    // each thread does the following: sample a new random config. Find NN in tree A and B.
    template <typename Robot>
    __global__ void sample_edges(float *new_configs, int *new_config_parents, float *new_config_dist, float *nodes, float *goal_configs, int num_goals, int *tree_id, curandState *rng_states) {
        static constexpr auto dim = Robot::dimension;
        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if (tid >= NUM_NEW_CONFIGS) return;
        if (tid == 0) printf("fi: %d\n", atomic_free_index);
        curandState local_rng_state = rng_states[tid];

        float *new_config_a = &new_configs[tid * dim];
        float *new_config_b = &new_configs[(tid + NUM_NEW_CONFIGS) * dim];
        float new_config[dim];

        // sample a random config
        for (int i = 0; i < dim; i++) {
            new_config[i] = (2 * COORD_BOUND * curand_uniform(&local_rng_state)) - COORD_BOUND;
        }
        rng_states[tid] = local_rng_state;

        // min dist and nearest idx for tree_A and tree_B
        float min_dist[2] = {1000000000.0, 1000000000.0};
        int nearest_idx[2] = {-1, -1};
        float dist;
        int cur_tree_id;
        for (int i = 0; i < atomic_free_index; i++) {
            // if (tid == 0) printf("%d\n", i);
            dist = device_utils::l2_dist(&nodes[i * dim], new_config, dim);

            cur_tree_id = tree_id[i];
            if (dist < min_dist[cur_tree_id]) {
                nearest_idx[cur_tree_id] = i;
                min_dist[cur_tree_id] = dist;
            }
        }

        if (tid == 0) {
            printf("%d\n", atomic_free_index);
            printf("%f %f\n", min_dist[0], min_dist[1]);
            printf("%d %d\n", nearest_idx[0], nearest_idx[1]);
        }

        // calculate vector to new config
        float scale[2] = {min(1.0f, RRT_RADIUS / min_dist[0]), min(1.0f, RRT_RADIUS / min_dist[1])};
        float *nearest_a = &nodes[nearest_idx[0] * dim];
        float *nearest_b = &nodes[nearest_idx[1] * dim];
        float vec[2][dim];
        for (int i = 0; i < dim; i++) {
            vec[0][i] = (new_config[i] - nearest_a[i]) * scale[0];
            vec[1][i] = (new_config[i] - nearest_b[i]) * scale[1];
        }

        for (int i = 0; i < dim; i++) {
            new_config_a[i] = nearest_a[i] + vec[0][i];
            new_config_b[i] = nearest_b[i] + vec[1][i];
        }
        min_dist[0] *= scale[0];
        min_dist[1] *= scale[1];
        
        new_config_dist[tid] = min_dist[0];
        new_config_dist[tid + NUM_NEW_CONFIGS] = min_dist[1];

        // set the parent of the new config
        new_config_parents[tid] = nearest_idx[0];
        new_config_parents[tid + NUM_NEW_CONFIGS] = nearest_idx[1];
    }

    // grow the RRT tree after we figure out what edges have no collisions
    // each thread is responsible for adding one edge to the tree
    template <typename Robot>
    __global__ void grow_tree(float *new_configs, int *new_config_parents, float *new_config_dist, bool *cc_result, float *nodes, int *parents, float *goal_configs, int num_goals, int *tree_id, PlannerResult<Robot> *d_res) {
        static constexpr auto dim = Robot::dimension;
        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if (tid >= NUM_NEW_CONFIGS * 2) return;
        if (cc_result[tid]) return;  // this edge had a collision, don't add it

        if (tid < NUM_NEW_CONFIGS) {
            float *new_config_a = &new_configs[tid * dim];
            float *new_config_b = &new_configs[(tid + NUM_NEW_CONFIGS) * dim];
            if (device_utils::l2_dist(new_config_a, new_config_b, dim) < 0.001) {
                reached_goal = true;
                d_res->solved = true;
                // add cost of the tree_a portion of the path
                int cur = new_config_parents[tid];
                d_res->cost = device_utils::l2_dist(new_config_a, &nodes[cur * dim], dim);
                while (cur != parents[cur]) {
                    d_res->cost += device_utils::l2_dist(&nodes[parents[cur] * dim], &nodes[cur * dim], dim);
                    cur = parents[cur];
                }
                // add cost of the tree_b portion of the path
                cur = new_config_parents[tid + NUM_NEW_CONFIGS];
                d_res->cost += device_utils::l2_dist(new_config_b, &nodes[cur * dim], dim);
                while (cur != parents[cur]) {
                    d_res->cost += device_utils::l2_dist(&nodes[parents[cur] * dim], &nodes[cur * dim], dim);
                    cur = parents[cur];
                }
                return;
            }
        }

        // Atomically get the next free index
        int my_index = atomicAdd(&atomic_free_index, 1);
        
        // Copy the configuration to the nodes array
        for (int i = 0; i < dim; i++) {
            nodes[my_index * dim + i] = new_configs[tid * dim + i];
        }
        tree_id[my_index] = (tid < NUM_NEW_CONFIGS) ? 0 : 1;
        
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

    inline void setup_environment_on_device(ppln::collision::Environment<float> *&d_env, 
        const ppln::collision::Environment<float> &h_env) {
        // Device pointers for primitive arrays
        ppln::collision::Sphere<float> *spheres = nullptr;
        ppln::collision::Capsule<float> *capsules = nullptr;
        ppln::collision::Capsule<float> *z_aligned_capsules = nullptr;
        ppln::collision::Cylinder<float> *cylinders = nullptr;
        ppln::collision::Cuboid<float> *cuboids = nullptr;
        ppln::collision::Cuboid<float> *z_aligned_cuboids = nullptr;

        // Allocate memory for environment struct
        cudaMalloc(&d_env, sizeof(ppln::collision::Environment<float>));

        // Allocate and copy each primitive array
        if (h_env.num_spheres > 0) {
            cudaMalloc(&spheres, sizeof(ppln::collision::Sphere<float>) * h_env.num_spheres);
            cudaMemcpy(spheres, h_env.spheres, sizeof(ppln::collision::Sphere<float>) * h_env.num_spheres, cudaMemcpyHostToDevice);
        }
        if (h_env.num_capsules > 0) {
            cudaMalloc(&capsules, sizeof(ppln::collision::Capsule<float>) * h_env.num_capsules);
            cudaMemcpy(capsules, h_env.capsules, sizeof(ppln::collision::Capsule<float>) * h_env.num_capsules, cudaMemcpyHostToDevice);
        }
        if (h_env.num_z_aligned_capsules > 0) {
            cudaMalloc(&z_aligned_capsules, sizeof(ppln::collision::Capsule<float>) * h_env.num_z_aligned_capsules);
            cudaMemcpy(z_aligned_capsules, h_env.z_aligned_capsules, sizeof(ppln::collision::Capsule<float>) * h_env.num_z_aligned_capsules, cudaMemcpyHostToDevice);
        }
        if (h_env.num_cylinders > 0) {
            cudaMalloc(&cylinders, sizeof(ppln::collision::Cylinder<float>) * h_env.num_cylinders);
            cudaMemcpy(cylinders, h_env.cylinders, sizeof(ppln::collision::Cylinder<float>) * h_env.num_cylinders, cudaMemcpyHostToDevice);
        }
        if (h_env.num_cuboids > 0) {
            cudaMalloc(&cuboids, sizeof(ppln::collision::Cuboid<float>) * h_env.num_cuboids);
            cudaMemcpy(cuboids, h_env.cuboids, sizeof(ppln::collision::Cuboid<float>) * h_env.num_cuboids, cudaMemcpyHostToDevice);
        }
        if (h_env.num_z_aligned_cuboids > 0) {
            cudaMalloc(&z_aligned_cuboids, sizeof(ppln::collision::Cuboid<float>) * h_env.num_z_aligned_cuboids);
            cudaMemcpy(z_aligned_cuboids, h_env.z_aligned_cuboids, sizeof(ppln::collision::Cuboid<float>) * h_env.num_z_aligned_cuboids, cudaMemcpyHostToDevice);
        }

        // Create and copy environment struct with device pointers
        ppln::collision::Environment<float> temp_env;
        temp_env.spheres = spheres;
        temp_env.num_spheres = h_env.num_spheres;
        temp_env.capsules = capsules;
        temp_env.num_capsules = h_env.num_capsules;
        temp_env.z_aligned_capsules = z_aligned_capsules;
        temp_env.num_z_aligned_capsules = h_env.num_z_aligned_capsules;
        temp_env.cylinders = cylinders;
        temp_env.num_cylinders = h_env.num_cylinders;
        temp_env.cuboids = cuboids;
        temp_env.num_cuboids = h_env.num_cuboids;
        temp_env.z_aligned_cuboids = z_aligned_cuboids;
        temp_env.num_z_aligned_cuboids = h_env.num_z_aligned_cuboids;

        cudaMemcpy(d_env, &temp_env, sizeof(ppln::collision::Environment<float>), cudaMemcpyHostToDevice);
    }

    inline void cleanup_environment_on_device(ppln::collision::Environment<float> *d_env, 
                                            const ppln::collision::Environment<float> &h_env) {
        // Create a temporary struct to hold device pointers
        ppln::collision::Environment<float> temp_env;
        cudaMemcpy(&temp_env, d_env, sizeof(ppln::collision::Environment<float>), cudaMemcpyDeviceToHost);
        
        // Free all allocated memory
        if (h_env.num_spheres > 0) cudaFree(temp_env.spheres);
        if (h_env.num_capsules > 0) cudaFree(temp_env.capsules);
        if (h_env.num_z_aligned_capsules > 0) cudaFree(temp_env.z_aligned_capsules);
        if (h_env.num_cylinders > 0) cudaFree(temp_env.cylinders);
        if (h_env.num_cuboids > 0) cudaFree(temp_env.cuboids);
        if (h_env.num_z_aligned_cuboids > 0) cudaFree(temp_env.z_aligned_cuboids);
        cudaFree(d_env);
    }

    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goals, ppln::collision::Environment<float> &h_environment) {
        // printCUDADeviceInfo();
        static constexpr auto dim = Robot::dimension;
        std::size_t iter = 0;
        printf("num goals: %d\n", goals.size());
        printf("num spheres, capsules, cuboids: %d, %d, %d\n", h_environment.num_spheres, h_environment.num_capsules, h_environment.num_cuboids);
        // std::size_t start_index = 0;
        std::size_t free_index = 0;
        // int tree_cnt = 0;

        auto start_time = std::chrono::steady_clock::now();
        PlannerResult<Robot> res;
        PlannerResult<Robot> *d_res;
        cudaMalloc(&d_res, sizeof(PlannerResult<Robot>));

        // copy stuff to GPU
        // GPU needs: start, goal, tree, parents, nodes

        float *start_config;
        float *goal_configs;
        int num_goals = goals.size();
        float *nodes;
        int *parents;
        int *tree_id;
        const std::size_t config_size = dim * sizeof(float);
        cudaMalloc(&start_config, config_size);
        cudaMalloc(&goal_configs, config_size * num_goals);
        cudaMalloc(&nodes, MAX_SAMPLES * config_size);
        cudaMalloc(&parents, MAX_SAMPLES * sizeof(int));
        cudaMalloc(&tree_id, MAX_SAMPLES * sizeof(int));
        cudaMemcpy(start_config, start.data(), config_size, cudaMemcpyHostToDevice);
        cudaMemcpy(goal_configs, goals.data(), config_size, cudaMemcpyHostToDevice);

        // add the start and goal configs to nodes list, and set each to have their own parents.
        // tree_id of the start node is 0, tree id of goals nodes are 1 (tree A and tree B)
        cudaMemcpy(nodes, start.data(), config_size, cudaMemcpyHostToDevice);
        cudaMemcpy(&nodes[1 * dim], goals.data(), config_size * num_goals, cudaMemcpyHostToDevice);
        std::vector<int> parents_init(num_goals + 1);
        std::iota(parents_init.begin(), parents_init.end(), 0); // consecutive integers from 0 ... num_goals
        cudaMemcpy(parents, parents_init.data(), (num_goals + 1) * sizeof(int), cudaMemcpyHostToDevice);
        std::vector<int> tree_id_init(num_goals + 1, 1);
        tree_id_init[0] = 0;
        cudaMemcpy(tree_id, tree_id_init.data(), (num_goals + 1) * sizeof(int), cudaMemcpyHostToDevice);
        free_index = num_goals + 1;

        // create a curandState for each thread -> holds state of RNG for each thread seperately
        // For growing the tree we will create NUM_NEW_CONFIGS threads
        curandState *rng_states;
        cudaMalloc(&rng_states, NUM_NEW_CONFIGS * sizeof(curandState));
        // constexpr int blockSize = 256;
        int numBlocks = (NUM_NEW_CONFIGS + BLOCK_SIZE - 1) / BLOCK_SIZE;
        int numBlocks2 = (2 * NUM_NEW_CONFIGS + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_rng<<<numBlocks, BLOCK_SIZE>>>(rng_states, 0);

        // create arrays on the gpu to hold the newly sampled configs, and their parents, and dist to parent
        // Each config will have 2 parents (one from tree A, and one from tree B)
        // We also need double the space for the configs themselves because the actual nodes added to A and B will not
        // be the config itself.
        float *new_configs;
        cudaMalloc(&new_configs, 2 * NUM_NEW_CONFIGS * config_size);
        int *new_config_parents;
        cudaMalloc(&new_config_parents, 2 * NUM_NEW_CONFIGS * sizeof(int));
        float *new_config_dist;
        cudaMalloc(&new_config_dist, 2 * NUM_NEW_CONFIGS * sizeof(float));
        cudaMemset(new_config_dist, 0, 2 * NUM_NEW_CONFIGS * sizeof(float));

        // create an array to hold the result of collision check for each new edge
        bool *cc_result;
        cudaMalloc(&cc_result, 2 * NUM_NEW_CONFIGS * sizeof(bool));
        cudaMemset(cc_result, 0, 2 * NUM_NEW_CONFIGS);

        // free index for next available position in the nodes array
        cudaMemcpyToSymbol(atomic_free_index, &free_index, sizeof(int));

        // allocate for environment
        ppln::collision::Environment<float> *env;
        setup_environment_on_device(env, h_environment);


        bool done = false;
        // main RRTC loop
        while (iter++ < MAX_ITERS && free_index < MAX_SAMPLES) {
            std::cout << "iter: " << iter << std::endl;
            // sample configurations and get edges to be checked
            sample_edges<Robot><<<numBlocks, BLOCK_SIZE>>>(new_configs, new_config_parents, new_config_dist, nodes, goal_configs, num_goals, tree_id, rng_states);
            cudaDeviceSynchronize();
            cudaCheckError(cudaGetLastError());

            // collision check all the edges
            cudaMemset(cc_result, 0, 2 * NUM_NEW_CONFIGS);
            validate_edges<Robot><<<2 * NUM_NEW_CONFIGS, GRANULARITY>>>(new_configs, new_config_parents, new_config_dist, cc_result, env, nodes);
            cudaDeviceSynchronize();
            cudaCheckError(cudaGetLastError());

            // add all the new edges to the tree
            grow_tree<Robot><<<numBlocks2, BLOCK_SIZE>>>(new_configs, new_config_parents, new_config_dist, cc_result, nodes, parents, goal_configs, num_goals, tree_id, d_res);
            cudaDeviceSynchronize();
            cudaCheckError(cudaGetLastError());
            
            // update free index
            cudaMemcpyFromSymbol(&free_index, atomic_free_index, sizeof(int), 0, cudaMemcpyDeviceToHost);
            cudaMemcpyFromSymbol(&done, reached_goal, sizeof(bool), 0, cudaMemcpyDeviceToHost);
            cudaCheckError(cudaGetLastError());

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
        cudaMemcpy(&res, d_res, sizeof(PlannerResult<Robot>), cudaMemcpyDeviceToHost);
        if (done) {
            printf("done\n");
        }
        res.nanoseconds = get_elapsed_nanoseconds(start_time);
        reset_device_variables();
        cleanup_environment_on_device(env, h_environment);
        cudaFree(start_config);
        cudaFree(goal_configs);
        cudaFree(nodes);
        cudaFree(parents);
        cudaFree(rng_states);
        cudaFree(new_configs);
        cudaFree(new_config_parents);
        cudaFree(cc_result);
        cudaFree(env);
        return res;
    }

    template PlannerResult<typename ppln::robots::Sphere> solve<ppln::robots::Sphere>(std::array<float, 3>&, std::vector<std::array<float, 3>>&, ppln::collision::Environment<float>&);
    template PlannerResult<typename ppln::robots::Panda> solve<ppln::robots::Panda>(std::array<float, 7>&, std::vector<std::array<float, 7>>&, ppln::collision::Environment<float>&);
}