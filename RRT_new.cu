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

/*
Parallelized RRT with parallelized collision checking.
Interleaved strategy: sample states in parallel, then check edges in parallel, then repeat.
sample states in parallel, check edges in parallel, grow tree in parallel, check if the new configs can reach the goal in parallel
*/

namespace RRT_new {
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
        curand_init(seed, idx, 0, &states[idx]);
    }

    template <typename Robot>
    __global__ void init_halton(HaltonState<Robot>* states) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        halton_initialize(states[idx], 0);
    }

    __device__ int atomic_free_index;
    __device__ int reached_goal = 0;
    __device__ int reached_goal_idx = -1;
    __device__ int goal_parent_idx = -1;

    constexpr int MAX_SAMPLES = 1000000;
    constexpr int MAX_ITERS = 1000000;
    constexpr int NUM_NEW_CONFIGS = 256;
    constexpr int GRANULARITY = 256;
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

    inline void reset_device_variables() {
        int zero = 0;
        bool false_val = false;
        
        cudaMemcpyToSymbol(atomic_free_index, &zero, sizeof(int));
        cudaMemcpyToSymbol(reached_goal, &zero, sizeof(int));
        cudaMemcpyToSymbol(reached_goal_idx, &zero, sizeof(int));
        cudaMemcpyToSymbol(goal_parent_idx, &zero, sizeof(int));
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
    __global__ void validate_edges(float *new_configs, int *new_config_parents, unsigned int *cc_result, int *num_colliding_edges, ppln::collision::Environment<float> *env, float *nodes) {
        static constexpr auto dim = Robot::dimension;
        int tid_in_block = threadIdx.x;
        int bid = blockIdx.x;
        // total_threads = num_samples * granularity;
        if (bid >= NUM_NEW_CONFIGS) return;
        if (tid_in_block >= GRANULARITY) return;
        // if (bid == 0 and tid_in_block == 0) {
        //     printf("device num spheres, capsules, cuboids: %d, %d, %d\n", env->num_spheres, env->num_capsules, env->num_cuboids);
        // }
        __shared__ float delta[dim];
        __shared__ float shared_edge_start[dim];
        if (tid_in_block == 0) {
            float *edge_start = &nodes[new_config_parents[bid] * dim];
            float *edge_end = &new_configs[bid * dim];
            for (int i = 0; i < dim; i++) {
                shared_edge_start[i] = edge_start[i];
                delta[i] = (edge_end[i] - edge_start[i]) / (float) GRANULARITY;;
            }
            // if (bid == 0) {
            //     printf("edge end: ");
            //     print_config(edge_end, dim);
            // }
        }
        __syncthreads();
        // if (bid == 0 and tid_in_block == 0) {
        //     printf("edge start: ");
        //     print_config(shared_edge_start, dim);
        //     printf("projected edge end: ");
        //     for (int i = 0; i < dim; i++) {
        //         printf("%f ", shared_edge_start[i] + (GRANULARITY * delta[i]));
        //     }
        //     printf("\n");
        // }
        // calculate the configuration this thread will be checking
        float config[dim];
        for (int i = 0; i < dim; i++) {
            config[i] = shared_edge_start[i] + ((tid_in_block + 1) * delta[i]);
        }
       
        // if (bid == 0 and tid_in_block == 0) {
        //     printf("config being validated: ");
        //     print_config(config, dim);
        // }
        // __syncthreads();
        // check for collision
        bool config_in_collision = not ppln::collision::fkcc<Robot>(config, env);
        // if (bid == 0 and tid_in_block == 0) {
        //     printf("first point in edge colliding?: %d\n", config_in_collision);
        // }
        // __syncthreads();
        // if (bid == 0 and tid_in_block == 255) {
        //     printf("last config in edge: (");
        //     print_config(config, dim);
        //     printf(")last point in edge colliding?: %d\n", config_in_collision);
        // }
        // __syncthreads();
        // if (bid == 0 and (not config_in_collision)) {
        //     printf("no collision: thread %d\n", tid_in_block);
        // }
        // if (bid == 0 and config_in_collision) {
        //     printf("collision: thread %d\n", tid_in_block);
        // }
        atomicOr(&cc_result[bid], config_in_collision ? 1u : 0u);
    }


    // initialize cuda random
    // __global__ void init_rng(curandState* states, unsigned long seed) {
    //     int idx = blockIdx.x * blockDim.x + threadIdx.x;
    //     curand_init(seed, idx, 0, &states[idx]);
    // }

    // each thread is responsible for finding a new edge to check
    // sample a new state -> connect it to nearest neighbor in our tree
    template <typename Robot>
    __global__ void sample_edges(float *new_configs, int *new_config_parents, float *nodes, float *goal_configs, int num_goals, curandState *rng_states, HaltonState<Robot> *halton_states) {
        // printf("here!");
        static constexpr auto dim = Robot::dimension;
        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if (tid >= NUM_NEW_CONFIGS) return;
        curandState local_rng_state = rng_states[tid];

        float *new_config = &new_configs[tid * dim];
        float config[dim];
        
        for (int i = 0; i < dim; i++) {
            config[i] = curand_uniform(&local_rng_state);
        }
        // halton_next(halton_states[tid], config);
        // for (int i = 0; i < dim; i++) {
        //     config[i] = set_configs[set_cfg_idx * dim + i];
        // }
        // set_cfg_idx++;
    
        // if (tid == 0) {printf("config before scaling: "); print_config(config, dim);}
        
        ppln::device_utils::scale_configuration<Robot>(config);
        // if (tid == 0) {printf("config after scaling: "); print_config(config, dim);}
        // __syncthreads();
        rng_states[tid] = local_rng_state;
        
        // Track both nearest and second nearest
        float min_dist = 1000000000.0;
        int nearest_idx = -1;

        float dist;
    
        for (int i = 0; i < atomic_free_index; i++) {
            dist = device_utils::l2_dist(&nodes[i * dim], config, dim);
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
        float *nearest_node = &nodes[nearest_idx * dim];
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
        min_dist *= scale;
        

        // set the parent of the new config
        new_config_parents[tid] = nearest_idx;
    }

    // grow the RRT tree after we figure out what edges have no collisions
    // each thread is responsible for adding one edge to the tree
    template <typename Robot>
    __global__ void grow_tree(float *new_configs, int *new_config_parents, unsigned int *cc_result, float *nodes, int *parents, int *num_colliding_edges, float *goal_configs, int num_goals, int *new_config_idxs) {
        static constexpr auto dim = Robot::dimension;
        int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if (tid >= NUM_NEW_CONFIGS) return;
        if (cc_result[tid] != 0) return;  // this edge had a collision, don't add it

        // Atomically get the next free index
        int my_index = atomicAdd(&atomic_free_index, 1);
        if (my_index >= MAX_SAMPLES) return;

        new_config_idxs[tid] = my_index;
        // Copy the configuration to the nodes array
        for (int i = 0; i < dim; i++) {
            nodes[my_index * dim + i] = new_configs[tid * dim + i];
        }
        
        // Set the parent
        parents[my_index] = new_config_parents[tid];
    }

    // Each thread will check one edge from a new_config to a goal
    template <typename Robot>
    __global__ void check_goal(float *new_configs, float *goal_configs, int num_goals, ppln::collision::Environment<float> *env, float *nodes, int *parents, unsigned int *cc_result, int *new_config_idxs) {
        static constexpr auto dim = Robot::dimension;
        // Calculate 3D thread indices
        // Each block handles one goal
        int goal_idx = blockIdx.z;
        int config_idx = blockIdx.y;  // which config we're checking
        int point_idx = threadIdx.x;  // which interpolated point along path

        if (goal_idx >= num_goals || config_idx >= NUM_NEW_CONFIGS || point_idx >= GRANULARITY) return;
        if (cc_result[config_idx] != 0) return;  // edge had collision during tree growth

        __shared__ float delta[dim];
        __shared__ float shared_start[dim];
    
        // First thread in block computes deltas
        if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0) {
            float *new_config = &new_configs[config_idx * dim];
            float *goal = &goal_configs[goal_idx * dim];
            for (int i = 0; i < dim; i++) {
                shared_start[i] = new_config[i];
                delta[i] = (goal[i] - new_config[i]) / (float) GRANULARITY;
            }
        }
        __syncthreads();

        // Calculate and check this thread's point
        float config[dim];
        for (int i = 0; i < dim; i++) {
            config[i] = shared_start[i] + ((point_idx + 1) * delta[i]);
        }
        
        bool point_valid = ppln::collision::fkcc<Robot>(config, env);
        
        // Use shared memory to track validity within the block
        __shared__ bool edge_valid;
        if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0) {
            edge_valid = true;
        }
        __syncthreads();
        
        // If any point is invalid, mark the edge as invalid
        atomicAnd((int*)&edge_valid, point_valid);
        __syncthreads();

        // Only one thread per edge updates the global state
        if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0 && edge_valid) {
            atomicCAS(&reached_goal, 0, 1);
            atomicCAS(&reached_goal_idx, -1, goal_idx);
            atomicCAS(&goal_parent_idx, -1, new_config_idxs[config_idx]);
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
        cudaCheckError(cudaGetLastError());
        cudaMalloc(&goal_configs, config_size * num_goals);
        cudaCheckError(cudaGetLastError());
        cudaMalloc(&nodes, MAX_SAMPLES * config_size);
        cudaCheckError(cudaGetLastError());
        cudaMalloc(&parents, MAX_SAMPLES * sizeof(int));
        cudaCheckError(cudaGetLastError());
        cudaMemcpy(start_config, start.data(), config_size, cudaMemcpyHostToDevice);
        cudaCheckError(cudaGetLastError());
        cudaMemcpy(goal_configs, goals.data(), config_size, cudaMemcpyHostToDevice);
        cudaCheckError(cudaGetLastError());
        // add the start config to the tree, and set the start to be it's own parent.
        cudaMemcpy(nodes, start.data(), config_size, cudaMemcpyHostToDevice);
        cudaCheckError(cudaGetLastError());
        cudaMemcpy(parents, &start_index, sizeof(int), cudaMemcpyHostToDevice);
        cudaCheckError(cudaGetLastError());

        // create a curandState for each thread -> holds state of RNG for each thread seperately
        // For growing the tree we will create NUM_NEW_CONFIGS threads
        curandState *rng_states;
        cudaMalloc(&rng_states, NUM_NEW_CONFIGS * sizeof(curandState));
        // constexpr int blockSize = 256;
        int numBlocks = (NUM_NEW_CONFIGS + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_rng<<<numBlocks, BLOCK_SIZE>>>(rng_states, 1);

        HaltonState<Robot> *halton_states;
        cudaMalloc(&halton_states, NUM_NEW_CONFIGS * sizeof(HaltonState<Robot>));
        init_halton<Robot><<<numBlocks, BLOCK_SIZE>>>(halton_states);

        // create arrays on the gpu to hold the newly sampled configs, and their parents, and dist to parent
        float *new_configs;
        cudaMalloc(&new_configs, NUM_NEW_CONFIGS * config_size);
        int *new_config_parents;
        cudaMalloc(&new_config_parents, NUM_NEW_CONFIGS * sizeof(int));
        cudaCheckError(cudaGetLastError());
        int *new_config_idxs;
        cudaMalloc(&new_config_idxs, NUM_NEW_CONFIGS * sizeof(int));

        // create an array to hold the result of collision check for each new edge
        unsigned int *cc_result;
        cudaMalloc(&cc_result, NUM_NEW_CONFIGS * sizeof(unsigned int));
        cudaMemset(cc_result, 0, NUM_NEW_CONFIGS * sizeof(unsigned int));
        int *num_colliding_edges;
        cudaMalloc(&num_colliding_edges, sizeof(int));
        cudaCheckError(cudaGetLastError());

        // free index for next available position in the nodes array
        cudaMemcpyToSymbol(atomic_free_index, &free_index, sizeof(int));
        cudaCheckError(cudaGetLastError());

        // allocate for obstacles
        ppln::collision::Environment<float> *env;
        // cudaMalloc(&env, sizeof(env));
        // cudaMemcpy(env, &h_environment, sizeof(env), cudaMemcpyHostToDevice);
        setup_environment_on_device(env, h_environment);
        cudaCheckError(cudaGetLastError());
        int done = 0;

        // calculate launch configuration for check_goals
        dim3 threadsPerBlock1(GRANULARITY);  // 512 threads total
        dim3 numBlocks1(
            1, NUM_NEW_CONFIGS, num_goals
        );


        // main RRT loop
        while (iter++ < MAX_ITERS && free_index < MAX_SAMPLES) {
            std::cout << "iter: " << iter << std::endl;
            
            // sample configurations and get edges to be checked
            // auto kernel_start_time = std::chrono::steady_clock::now();
            sample_edges<Robot><<<numBlocks, BLOCK_SIZE>>>(new_configs, new_config_parents, nodes, goal_configs, num_goals, rng_states, halton_states);
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();
            // std::cout << "sample edges (ns): " << get_elapsed_nanoseconds(kernel_start_time) << "\n";
            
            // collision check all the edges
            // kernel_start_time = std::chrono::steady_clock::now();
            cudaMemset(cc_result, 0, NUM_NEW_CONFIGS * sizeof(unsigned int));
            validate_edges<Robot><<<NUM_NEW_CONFIGS, GRANULARITY>>>(new_configs, new_config_parents, cc_result, num_colliding_edges, env, nodes);
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();
            // std::cout << "validate edges (ns): " << get_elapsed_nanoseconds(kernel_start_time) << "\n";
            
            // add all the new edges to the tree
            // kernel_start_time = std::chrono::steady_clock::now();
            grow_tree<Robot><<<numBlocks, BLOCK_SIZE>>>(new_configs, new_config_parents, cc_result, nodes, parents, num_colliding_edges, goal_configs, num_goals, new_config_idxs);
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();
            // std::cout << "grow tree (ns): " << get_elapsed_nanoseconds(kernel_start_time) << "\n";

            // check whether each new configuration added to the tree can reach the goal
            // kernel_start_time = std::chrono::steady_clock::now();
            check_goal<Robot><<<numBlocks1, threadsPerBlock1>>>(new_configs, goal_configs, num_goals, env, nodes, parents, cc_result, new_config_idxs);
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();
            // std::cout << "check goal (ns): " << get_elapsed_nanoseconds(kernel_start_time) << "\n";

            // update free index
            cudaMemcpyFromSymbol(&free_index, atomic_free_index, sizeof(int), 0, cudaMemcpyDeviceToHost);
            cudaCheckError(cudaGetLastError());
            cudaMemcpyFromSymbol(&done, reached_goal, sizeof(int), 0, cudaMemcpyDeviceToHost);
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
            Robot::print_robot_config(cfg);
            while (parent_idx != h_parents[parent_idx]) {
                // std::cout << parent_idx << std::endl;
                std::copy_n(h_nodes.begin() + parent_idx * dim, dim, cfg.begin());
                std::copy_n(h_nodes.begin() + h_parents[parent_idx] * dim, dim, cfg_parent.begin());
                Robot::print_robot_config(cfg);
                res.cost += l2dist<Robot>(cfg, cfg_parent);
                res.path.emplace_back(parent_idx);
                parent_idx = h_parents[parent_idx];
            }
            res.path.emplace_back(parent_idx);
            std::reverse(res.path.begin(), res.path.end());
        }
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
        cudaFree(num_colliding_edges);
        // cudaFree(env);
        cudaCheckError(cudaGetLastError());
        return res;
    }

    template PlannerResult<typename ppln::robots::Sphere> solve<ppln::robots::Sphere>(std::array<float, 3>&, std::vector<std::array<float, 3>>&, ppln::collision::Environment<float>&);
    template PlannerResult<typename ppln::robots::Panda> solve<ppln::robots::Panda>(std::array<float, 7>&, std::vector<std::array<float, 7>>&, ppln::collision::Environment<float>&);
}



