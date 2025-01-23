#include "Planners.hh"
#include "Robots.hh"
#include "utils.cuh"
#include "collision/environment.hh"

#include <curand.h>
#include <curand_kernel.h>

#include <vector>
#include <iostream>
#include <cassert>
#include <algorithm>

/*
Parallelized RRTC
Add start to tree_a, and all goals to tree_b
use an array to keep track of the tree_id of each node
1. sample_edges: Sample new configs for tree_a and tree_b. For each new config find the NN in it's corresponding tree.
2. validate_edges: For each edge from a new config to it's tree, validate the edge.
3. grow_tree: Add all the valid points to their respective trees
4. connect_1: For each new config added to tree_a or tree_b find it's NN in the opposite tree
5. connect_2: For each edge from new_cfg-->NN_opposite_tree validate the edge
6. Extend as far as we can along this edge. and add it to new_cfg's tree
If we were able to go all the way, we reached goal.

Parallelized RRTC
Separate nodes/parents arrays for tree_a and tree_b
Add start to tree_a, add goals to tree_b
1. sample_Edges: Sample new configs for tree_a and tree_b and for each new config find the NN in corresponding tree
2. validate_Edges: for each edge to a new config from it's tree validate the edge
3. grow_tree: Add all te valid points to their respective trees
4. connect_1: for each new config_added to tree_a or tree_b find NN in the opposite tree
5. connect_2: For each edge from new_cfg-->NN_opposite_tree validate the edge
6. Extend as far as we can along this edge. and add it to new_cfg's tree
If we were able to go all the way, we reached goal.
*/

namespace pRRTC {
    __device__ int atomic_free_index[2]; // separate for tree_a and tree_b
    __device__ int reached_goal = 0;
    __device__ int reached_goal_idx = -1;
    __device__ int goal_parent_idx = -1;

    constexpr int MAX_SAMPLES = 1000000;
    constexpr int MAX_ITERS = 1000000;
    constexpr int NUM_NEW_CONFIGS = 64;
    constexpr int GRANULARITY = 256;
    constexpr float RRT_RADIUS = 1.0;

    // threads per block for sample_edges and grow_tree
    constexpr int BLOCK_SIZE = 256;

    using namespace ppln;

    // Constants
    __constant__ float primes[16] = {
        3.f, 5.f, 7.f, 11.f, 13.f, 17.f, 19.f, 23.f,
        29.f, 31.f, 37.f, 41.f, 43.f, 47.f, 53.f, 59.f
    };

    template<typename Robot>
    struct HaltonState {
        float b[Robot::dimension];   // bases
        float n[Robot::dimension];   // numerators
        float d[Robot::dimension];   // denominators
    };

    template<typename Robot>
    __device__ void halton_initialize(HaltonState<Robot>& state, size_t skip_iterations) {
        // Initialize bases from primes
        for (size_t i = 0; i < Robot::dimension; i++) {
            state.b[i] = primes[i];
            state.n[i] = 0.0f;
            state.d[i] = 1.0f;
        }
        
        // Skip iterations if requested
        float temp_result[Robot::dimension];
        for (size_t i = 0; i < skip_iterations; i++) {
            halton_next(state, temp_result);
        }
    }

    template<typename Robot>
    __device__ void halton_next(HaltonState<Robot>& state, float* result) {
        for (size_t i = 0; i < Robot::dimension; i++) {
            float xf = state.d[i] - state.n[i];
            bool x_eq_1 = (xf == 1.0f);
            
            if (x_eq_1) {
                // x == 1 case
                state.d[i] = floorf(state.d[i] * state.b[i]);
                state.n[i] = 1.0f;
            } else {
                // x != 1 case
                float y = floorf(state.d[i] / state.b[i]);
                
                // Continue dividing by b until we find the right digit position
                while (xf <= y) {
                    y = floorf(y / state.b[i]);
                }
                
                state.n[i] = floorf((state.b[i] + 1.0f) * y) - xf;
            }
            
            result[i] = state.n[i] / state.d[i];
        }
    }

    __global__ void init_rng(curandState* states, unsigned long seed) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= NUM_NEW_CONFIGS) return;
        curand_init(seed + idx, idx, 0, &states[idx]);
    }

    template <typename Robot>
    __global__ void init_halton(HaltonState<Robot>* states, curandState* cr_states) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= NUM_NEW_CONFIGS) return;
        int skip = (curand_uniform(&cr_states[idx]) * 500000.0f);
        printf("idx, skip: %d, %d\n", idx, skip);
        if (idx == 0) skip = 0;
        // if (idx == 1) skip = 14000;
        halton_initialize(states[idx], skip);
    }

    __device__ inline void print_config(float *config, int dim) {
        for (int i = 0; i < dim; i++) {
            printf("%f ", config[i]);
        }
        printf("\n");
    }

    inline void reset_device_variables() {
        int zero = 0;
        int neg1 = -1;
        
        cudaMemcpyToSymbol(atomic_free_index, &zero, sizeof(int));
        cudaMemcpyToSymbol(reached_goal, &zero, sizeof(int));
        cudaMemcpyToSymbol(reached_goal_idx, &neg1, sizeof(int));
        cudaMemcpyToSymbol(goal_parent_idx, &neg1, sizeof(int));
    }

    inline void setup_environment_on_device(ppln::collision::Environment<float> *&d_env, 
                                      const ppln::collision::Environment<float> &h_env) {
        // First allocate the environment struct
        cudaMalloc(&d_env, sizeof(ppln::collision::Environment<float>));
        
        // Initialize struct to zeros first
        cudaMemset(d_env, 0, sizeof(ppln::collision::Environment<float>));

        // Handle each primitive type separately
        if (h_env.num_spheres > 0) {
            // Allocate and copy spheres array
            ppln::collision::Sphere<float> *d_spheres;
            cudaMalloc(&d_spheres, sizeof(ppln::collision::Sphere<float>) * h_env.num_spheres);
            cudaMemcpy(d_spheres, h_env.spheres, 
                    sizeof(ppln::collision::Sphere<float>) * h_env.num_spheres, 
                    cudaMemcpyHostToDevice);
            
            // Update the struct fields directly
            cudaMemcpy(&(d_env->spheres), &d_spheres, sizeof(ppln::collision::Sphere<float>*), 
                    cudaMemcpyHostToDevice);
            cudaMemcpy(&(d_env->num_spheres), &h_env.num_spheres, sizeof(unsigned int), 
                    cudaMemcpyHostToDevice);
        }

        if (h_env.num_capsules > 0) {
            ppln::collision::Capsule<float> *d_capsules;
            cudaMalloc(&d_capsules, sizeof(ppln::collision::Capsule<float>) * h_env.num_capsules);
            cudaMemcpy(d_capsules, h_env.capsules,
                    sizeof(ppln::collision::Capsule<float>) * h_env.num_capsules,
                    cudaMemcpyHostToDevice);
            
            cudaMemcpy(&(d_env->capsules), &d_capsules, sizeof(ppln::collision::Capsule<float>*),
                    cudaMemcpyHostToDevice);
            cudaMemcpy(&(d_env->num_capsules), &h_env.num_capsules, sizeof(unsigned int),
                    cudaMemcpyHostToDevice);
        }

        // Repeat for each primitive type...
        if (h_env.num_z_aligned_capsules > 0) {
            ppln::collision::Capsule<float> *d_z_capsules;
            cudaMalloc(&d_z_capsules, sizeof(ppln::collision::Capsule<float>) * h_env.num_z_aligned_capsules);
            cudaMemcpy(d_z_capsules, h_env.z_aligned_capsules,
                    sizeof(ppln::collision::Capsule<float>) * h_env.num_z_aligned_capsules,
                    cudaMemcpyHostToDevice);
            
            cudaMemcpy(&(d_env->z_aligned_capsules), &d_z_capsules, sizeof(ppln::collision::Capsule<float>*),
                    cudaMemcpyHostToDevice);
            cudaMemcpy(&(d_env->num_z_aligned_capsules), &h_env.num_z_aligned_capsules, sizeof(unsigned int),
                    cudaMemcpyHostToDevice);
        }

        if (h_env.num_cylinders > 0) {
            ppln::collision::Cylinder<float> *d_cylinders;
            cudaMalloc(&d_cylinders, sizeof(ppln::collision::Cylinder<float>) * h_env.num_cylinders);
            cudaMemcpy(d_cylinders, h_env.cylinders,
                    sizeof(ppln::collision::Cylinder<float>) * h_env.num_cylinders,
                    cudaMemcpyHostToDevice);
            
            cudaMemcpy(&(d_env->cylinders), &d_cylinders, sizeof(ppln::collision::Cylinder<float>*),
                    cudaMemcpyHostToDevice);
            cudaMemcpy(&(d_env->num_cylinders), &h_env.num_cylinders, sizeof(unsigned int),
                    cudaMemcpyHostToDevice);
        }

        if (h_env.num_cuboids > 0) {
            ppln::collision::Cuboid<float> *d_cuboids;
            cudaMalloc(&d_cuboids, sizeof(ppln::collision::Cuboid<float>) * h_env.num_cuboids);
            cudaMemcpy(d_cuboids, h_env.cuboids,
                    sizeof(ppln::collision::Cuboid<float>) * h_env.num_cuboids,
                    cudaMemcpyHostToDevice);
            
            cudaMemcpy(&(d_env->cuboids), &d_cuboids, sizeof(ppln::collision::Cuboid<float>*),
                    cudaMemcpyHostToDevice);
            cudaMemcpy(&(d_env->num_cuboids), &h_env.num_cuboids, sizeof(unsigned int),
                    cudaMemcpyHostToDevice);
        }

        if (h_env.num_z_aligned_cuboids > 0) {
            ppln::collision::Cuboid<float> *d_z_cuboids;
            cudaMalloc(&d_z_cuboids, sizeof(ppln::collision::Cuboid<float>) * h_env.num_z_aligned_cuboids);
            cudaMemcpy(d_z_cuboids, h_env.z_aligned_cuboids,
                    sizeof(ppln::collision::Cuboid<float>) * h_env.num_z_aligned_cuboids,
                    cudaMemcpyHostToDevice);
            
            cudaMemcpy(&(d_env->z_aligned_cuboids), &d_z_cuboids, sizeof(ppln::collision::Cuboid<float>*),
                    cudaMemcpyHostToDevice);
            cudaMemcpy(&(d_env->num_z_aligned_cuboids), &h_env.num_z_aligned_cuboids, sizeof(unsigned int),
                    cudaMemcpyHostToDevice);
        }
    }


    inline void cleanup_environment_on_device(ppln::collision::Environment<float> *d_env, 
                                        const ppln::collision::Environment<float> &h_env) {
        // Get the pointers from device struct before freeing
        ppln::collision::Sphere<float> *d_spheres = nullptr;
        ppln::collision::Capsule<float> *d_capsules = nullptr;
        ppln::collision::Capsule<float> *d_z_capsules = nullptr;
        ppln::collision::Cylinder<float> *d_cylinders = nullptr;
        ppln::collision::Cuboid<float> *d_cuboids = nullptr;
        ppln::collision::Cuboid<float> *d_z_cuboids = nullptr;

        // Copy each pointer from device memory
        if (h_env.num_spheres > 0) {
            cudaMemcpy(&d_spheres, &(d_env->spheres), sizeof(ppln::collision::Sphere<float>*), cudaMemcpyDeviceToHost);
            cudaFree(d_spheres);
        }
        
        if (h_env.num_capsules > 0) {
            cudaMemcpy(&d_capsules, &(d_env->capsules), sizeof(ppln::collision::Capsule<float>*), cudaMemcpyDeviceToHost);
            cudaFree(d_capsules);
        }
        
        if (h_env.num_z_aligned_capsules > 0) {
            cudaMemcpy(&d_z_capsules, &(d_env->z_aligned_capsules), sizeof(ppln::collision::Capsule<float>*), cudaMemcpyDeviceToHost);
            cudaFree(d_z_capsules);
        }
        
        if (h_env.num_cylinders > 0) {
            cudaMemcpy(&d_cylinders, &(d_env->cylinders), sizeof(ppln::collision::Cylinder<float>*), cudaMemcpyDeviceToHost);
            cudaFree(d_cylinders);
        }
        
        if (h_env.num_cuboids > 0) {
            cudaMemcpy(&d_cuboids, &(d_env->cuboids), sizeof(ppln::collision::Cuboid<float>*), cudaMemcpyDeviceToHost);
            cudaFree(d_cuboids);
        }
        
        if (h_env.num_z_aligned_cuboids > 0) {
            cudaMemcpy(&d_z_cuboids, &(d_env->z_aligned_cuboids), sizeof(ppln::collision::Cuboid<float>*), cudaMemcpyDeviceToHost);
            cudaFree(d_z_cuboids);
        }

        // Finally free the environment struct itself
        cudaFree(d_env);
    }

    // granularity = number of interpolated points to check along each edge
    // total number of threads we need is edges * granularity
    // Each block is of size granularity and it checks one edge. Each thread in the block checks a consecutive interpolated point along the edge.
    template <typename Robot>
    // __launch_bounds__(256, 8)
    __global__ void validate_edges(
        float *new_configs,
        int *new_config_parents,
        unsigned int *cc_result,
        ppln::collision::Environment<float> *env,
        float *nodes[2],
        int *parents[2]
    )
    {
        static constexpr auto dim = Robot::dimension;
        int tid = threadIdx.x;
        int bid = blockIdx.x;
        if (bid >= NUM_NEW_CONFIGS) return;
        if (tid >= GRANULARITY) return;
        const int tree_id = (bid < NUM_NEW_CONFIG / 2);
        float *t_nodes = nodes[tree_id];
        int *atomic_free_index = atomic_free_index[tree_id];
        int *t_parents = parents[tree_id];
        __shared__ float delta[dim];
        __shared__ float shared_edge_start[dim];
        __shared__ float var_cache[GRANULARITY][10];
        __shared__ unsigned int result = 0u;
        __shared__ float *edge_start = &t_nodes[new_config_parents[bid] * dim];
        __shared__ float *edge_end = &new_configs[bid * dim];
        if (tid < dim) {
            shared_edge_start[tid] = edge_start[tid];
            delta[tid] = (edge_end[tid] - edge_start[tid]) / (float) GRANULARITY;
        }
        __syncthreads();

        float config[dim];

        for (int i = 0; i < dim; i++) {
            config[i] = shared_edge_start[i] + ((tid + 1) * delta[i]);
        }
       
        // check for collision
        bool config_in_collision = not ppln::collision::fkcc<Robot>(&config[0], env, var_cache, tid);

        atomicOr(&result, config_in_collision ? 1u : 0u);
        __syncthreads();

        cc_result[bid] = result;
        // add to tree if good
        if (result) {
            if (tid == 0) {
                int index = atomicAdd(&atomic_free_index, 1);
                if (index >= MAX_SAMPLES) return;
                new_config_idxs[bid] = index;
                for (int i = 0; i < dim; i++) {
                    t_nodes[index * dim + i] = edge_end[i];
                }
                t_parents[index] = new_config_parents[bid];
            }
        }
    }

    // each thread is responsible for finding a new edge to check
    // sample a new state -> connect it to nearest neighbor in our tree
    template <typename Robot>
    __global__ void sample_edges(
        float *new_configs,
        int *new_config_parents,
        float *nodes[2],
        float *goal_configs,
        int num_goals,
        curandState *rng_states,
        HaltonState<Robot> *halton_states
    )
    {
        // printf("here!");
        static constexpr auto dim = Robot::dimension;
        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if (tid >= NUM_NEW_CONFIGS) return;
        float *new_config = &new_configs[tid * dim];
        float config[dim];
        const int tree_id = (tid < NUM_NEW_CONFIG / 2);
        float *t_nodes = nodes[tree_id];
        
        // for (int i = 0; i < dim; i++) {
        //     config[i] = curand_uniform(&rng_states[tid]);
        // }
        halton_next(halton_states[tid], config);
        Robot::scale_cfg(config);
        
        // Track both nearest and second nearest
        float min_dist = 1000000000.0;
        int nearest_idx = -1;

        float dist;
    
        for (int i = 0; i < atomic_free_index[tree_id]; i++) {
            dist = device_utils::l2_dist(&t_nodes[i * dim], config, dim);
            // printf("dist: %f\n", dist);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        // if (tid == 0) {
        //     printf("free_index: %d\n", atomic_free_index);
        //     printf("dist to goal: %f\n", min_dist);
        //     printf("neares idx to goal: %d\n", nearest_idx);
        // }

        // keep it within the rrt range
        float scale = min(1.0f, RRT_RADIUS / min_dist);
        float *nearest_node = &t_nodes[nearest_idx * dim];
        float vec[dim];
        for (int i = 0; i < dim; i++) {
            vec[i] = (config[i] - nearest_node[i]) * scale;
        }

        for (int i = 0; i < dim; i++) {
            new_config[i] = nearest_node[i] + vec[i];
        }
        // if (tid == 0) {
        //     printf("nearest (id: %d): ", nearest_idx);
        //     print_config(nearest_node, dim);
        //     printf("nearest + vec: ");
        //     print_config(new_config, dim);
        //     printf("config: ");
        //     print_config(config, dim);
        // }
        // __syncthreads();        

        // set the parent of the new config
        new_config_parents[tid] = nearest_idx;
    }

    // each thread will find the nearest neighbor in opposite tree and go as far as possible towards it
    template <typename Robot>
    __global__ void connect(
        float *new_configs,
        int *new_config_parents,
        float *nodes[2],
        float *goal_configs,
        int num_goals,
        curandState *rng_states,
        HaltonState<Robot> *halton_states
    )
    {
        static constexpr auto dim = Robot::dimension;
        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if (tid >= NUM_NEW_CONFIGS) return;
        float *new_config = &new_configs[tid * dim];
        const int tree_id = (tid < NUM_NEW_CONFIG / 2);
        const int o_tree_id = 1 - tree_id;
        float *t_nodes = nodes[tree_id];
        float *o_nodes = nodes[o_tree_id];

        // find nearest node in other tree
        float min_dist = 1000000000.0;
        int nearest_idx = -1;
        float dist;
        for (int i = 0; i < atomic_free_index[o_tree_id]; i++) {
            dist = device_utils::l2_dist(&o_nodes[i * dim], new_config, dim);
            // printf("dist: %f\n", dist);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }



    }



    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goals, ppln::collision::Environment<float> &h_environment) {
        // printCUDADeviceInfo();
        // printf("num spheres, capsules, cuboids: %d, %d, %d\n", h_environment.num_spheres, h_environment.num_capsules, h_environment.num_cuboids);
        static constexpr auto dim = Robot::dimension;
        for (int i = 0; i < dim; i++) {
            printf("%f ", start[i]);
        }
        printf("\n");
        for (int i = 0; i < dim; i++) {
            printf("%f ", goals[0][i]);
        }
        printf("\n");
        std::size_t iter = 0;
        std::size_t start_index = 0;

        auto start_time = std::chrono::steady_clock::now();
        PlannerResult<Robot> res;

        // copy stuff to GPU
        // GPU needs: start, goal, tree, parents, nodes

        float *start_config;
        float *goal_configs;
        int num_goals = goals.size();

        float *nodes[2];
        int *parents[2];

        const std::size_t config_size = dim * sizeof(float);
        cudaMalloc(&start_config, config_size);
        cudaMalloc(&goal_configs, config_size * num_goals);
        for (int i = 0; i < 2; i++) {
            cudaMalloc(&nodes[i], MAX_SAMPLES * config_size);
            cudaMalloc(&parents[i], MAX_SAMPLES * sizeof(int));
        }

        cudaMemcpy(start_config, start.data(), config_size, cudaMemcpyHostToDevice);
        cudaMemcpy(goal_configs, goals.data(), config_size, cudaMemcpyHostToDevice);

        // add start to tree_a and goals to tree_b
        cudaMemcpy(nodes[0], start.data(), config_size, cudaMemcpyHostToDevice);
        cudaMemcpy(parents[0], &start_index, sizeof(int), cudaMemcpyHostToDevice);

        cudaMemcpy(nodes[1], goals.data(), config_size * num_goals, cudaMemcpyHostToDevice);
        std::vector<int> parents_b_init(num_goals);
        iota(parents_b_init.begin(), parents_b_init.end(), 0); // consecutive integers from 0 ... num_goals - 1
        cudaMemcpy(parents[1], parents_b_init.data(), sizeof(int) * num_goals, cudaMemcpyHostToDevice);
        cudaCheckError(cudaGetLastError());

        // create a curandState for each thread -> holds state of RNG for each thread seperately
        // For growing the tree we will create NUM_NEW_CONFIGS threads
        curandState *rng_states;
        cudaMalloc(&rng_states, NUM_NEW_CONFIGS * sizeof(curandState));
        int numBlocks = (NUM_NEW_CONFIGS + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_rng<<<numBlocks, BLOCK_SIZE>>>(rng_states, 1);

        HaltonState<Robot> *halton_states;
        cudaMalloc(&halton_states, NUM_NEW_CONFIGS * sizeof(HaltonState<Robot>));
        init_halton<Robot><<<numBlocks, BLOCK_SIZE>>>(halton_states, rng_states);

        // create arrays on the gpu to hold the newly sampled configs, and their parents, and dist to parent
        // 1st half of new_configs go to tree_a, 2nd half go the tree_b
        float *new_configs;
        int *new_config_parents;
        int *new_config_idxs;
        cudaMalloc(&new_configs, NUM_NEW_CONFIG * config_size);
        cudaMalloc(&new_config_parents, NUM_NEW_CONFIG * sizeof(int));
        cudaMalloc(&new_config_idxs, NUM_NEW_CONFIG * sizeof(int));

        // create an array to hold the result of collision check for each new edge
        unsigned int *cc_result;
        cudaMalloc(&cc_result, NUM_NEW_CONFIGS * sizeof(unsigned int));
        cudaMemset(cc_result, 0, NUM_NEW_CONFIGS * sizeof(unsigned int));
        cudaCheckError(cudaGetLastError());

        // free index for next available position in tree_a and tree_b
        int h_free_index[2] = {1, num_goals};
        cudaMemcpyToSymbol(atomic_free_index, &h_free_index, sizeof(int) * 2);
        cudaCheckError(cudaGetLastError());

        // allocate for obstacles
        ppln::collision::Environment<float> *env;
        setup_environment_on_device(env, h_environment);
        cudaCheckError(cudaGetLastError());

        int done = 0;

        // calculate launch configuration for check_goals
        dim3 threadsPerBlock1(GRANULARITY);
        dim3 numBlocks1(
            1, NUM_NEW_CONFIGS, num_goals
        );


        // main RRT loop
        while (iter++ < MAX_ITERS && (h_free_index[0] < MAX_SAMPLES && h_free_index[1] < MAX_SAMPLES)) {
            std::cout << "iter: " << iter << std::endl;
            
            // sample configurations and get edges to be checked
            auto kernel_start_time = std::chrono::steady_clock::now();
            sample_edges<Robot><<<numBlocks, BLOCK_SIZE>>>(
                new_configs,
                new_config_parents,
                nodes,
                goal_configs,
                num_goals,
                rng_states,
                halton_states
            );
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();
            std::cout << "sample edges (ns): " << get_elapsed_nanoseconds(kernel_start_time) << "\n";
            
            // collision check all the edges and add valid edges to their respective trees
            kernel_start_time = std::chrono::steady_clock::now();
            cudaMemset(cc_result, 0, NUM_NEW_CONFIGS * sizeof(unsigned int));
            validate_edges<Robot><<<NUM_NEW_CONFIGS, GRANULARITY>>>(
                new_configs,
                new_config_parents,
                cc_result,
                env,
                nodes
            );
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();
            std::cout << "validate edges (ns): " << get_elapsed_nanoseconds(kernel_start_time) << "\n";

            // find nearest neighbor in other tree and extend as far as we can to that tree
            connect<Robot><<<NUM_NEW_CONFIGS, GRANULARITY>>>(

            );
            cudaCheckError(cudaGetLastError());

            cudaMemcpyFromSymbol(&h_free_index, atomic_free_index, sizeof(int) * 2, 0, cudaMemcpyDeviceToHost);
            cudaCheckError(cudaGetLastError());
            cudaMemcpyFromSymbol(&done, reached_goal, sizeof(int), 0, cudaMemcpyDeviceToHost);
            cudaCheckError(cudaGetLastError());
            // std::cout << "sync + check done (ns): " << get_elapsed_nanoseconds(kernel_start_time) << "\n";
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
        
        if (done) {
            printf("done!\n");
            // get the index of the goal we found in the goals array
            int h_goal_idx;
            cudaMemcpyFromSymbol(&h_goal_idx, reached_goal_idx, sizeof(int), 0, cudaMemcpyDeviceToHost);
            std::cout << "Found Goal: " << h_goal_idx << std::endl;
            res.solved = true;
            // get parent at position 0 in new_config_parents, because that will be the parent of the goal
            int parent_idx = -1;
            cudaMemcpyFromSymbol(&parent_idx, goal_parent_idx, sizeof(int), 0, cudaMemcpyDeviceToHost);
            assert(parent_idx != -1);
            typename Robot::Configuration cfg;
            typename Robot::Configuration cfg_parent;
            std::copy_n(h_nodes.begin() + (parent_idx * dim), dim, cfg.begin());
            res.cost += l2dist<Robot>(goals[h_goal_idx], cfg);
            Robot::print_robot_config(goals[h_goal_idx]);
            while (parent_idx != h_parents[parent_idx]) {
                // std::cout << parent_idx << std::endl;
                std::copy_n(h_nodes.begin() + parent_idx * dim, dim, cfg.begin());
                std::copy_n(h_nodes.begin() + h_parents[parent_idx] * dim, dim, cfg_parent.begin());
                Robot::print_robot_config(cfg);
                res.cost += l2dist<Robot>(cfg, cfg_parent);
                res.path.emplace_back(parent_idx);
                parent_idx = h_parents[parent_idx];
            }
            Robot::print_robot_config(start);
            res.path.emplace_back(parent_idx);
            std::reverse(res.path.begin(), res.path.end());
        }
        std::cout << "cost from cu: " << res.cost << "\n";
        res.nanoseconds = get_elapsed_nanoseconds(start_time);
        // printf("almost done\n");
        reset_device_variables();
        cudaCheckError(cudaGetLastError());
        cleanup_environment_on_device(env, h_environment);
        cudaCheckError(cudaGetLastError());
        cudaFree(start_config);
        cudaFree(goal_configs);
        cudaFree(nodes);
        cudaFree(parents);
        cudaFree(rng_states);
        cudaFree(new_configs);
        cudaFree(new_config_parents);
        cudaFree(cc_result);
        cudaFree(new_config_idxs);
        cudaCheckError(cudaGetLastError());
        return res;
    }

    template PlannerResult<typename ppln::robots::Sphere> solve<ppln::robots::Sphere>(std::array<float, 3>&, std::vector<std::array<float, 3>>&, ppln::collision::Environment<float>&);
    template PlannerResult<typename ppln::robots::Panda> solve<ppln::robots::Panda>(std::array<float, 7>&, std::vector<std::array<float, 7>>&, ppln::collision::Environment<float>&);
    template PlannerResult<typename ppln::robots::Fetch> solve<ppln::robots::Fetch>(std::array<float, 8>&, std::vector<std::array<float, 8>>&, ppln::collision::Environment<float>&);
}
