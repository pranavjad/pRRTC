#include "Planners.hh"
#include "Robots.hh"
#include "utils.cuh"
#include "pRRTC_settings.hh"
#include "src/collision/environment.hh"

#include <curand.h>
#include <curand_kernel.h>
#include <float.h>

#include <vector>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <numeric>

#define FULL_MASK 0xffffffff

/*
Parallelized RRTC:
Each warp grows it's own tree.
*/


namespace nRRTC {
    using namespace ppln;
    __device__ volatile int solved = 0;
    __device__ int solved_free_index[2] = {0, 0};
    __device__ float path[2][500]; // solution path segments for tree_a, and tree_b
    __device__ int path_size[2] = {0, 0};
    __device__ float cost = 0.0;
    __device__ int reached_goal_idx = 0;
    __device__ int solved_iters = 0; // value of iters in the block that solves the problem
    __constant__ pRRTC_settings d_settings;

    // constexpr int MAX_GRANULARITY = 256;
    constexpr int BLOCK_SIZE = 64;
    constexpr int NUM_TREES = 256;
    constexpr int MAX_TREE_SIZE = (10000000 / NUM_TREES);

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
    __device__ void halton_initialize(HaltonState<Robot>& state, size_t skip_iterations, curandState& rng_state, int idx) {
        
        float primes[16] = {
            3.f, 5.f, 7.f, 11.f, 13.f, 17.f, 19.f, 23.f,
            29.f, 31.f, 37.f, 41.f, 43.f, 47.f, 53.f, 59.f
        };
        if (idx != 0) shuffle_array(primes, 16, rng_state);
        
        // Initialize bases from primes
        for (size_t i = 0; i < Robot::dimension; i++) {
            state.b[i] = primes[i];
            state.n[i] = 0.0f;
            state.d[i] = 1.0f;
        }
        
        // Skip iterations if requested
        volatile float temp_result[Robot::dimension];
        for (size_t i = 0; i < skip_iterations; i++) {
            halton_next(state, (float *)temp_result);
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

    __global__ void init_rng(curandState* states, unsigned long seed, int num_rng_states) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= num_rng_states) return;
        curand_init(seed + idx, idx, 0, &states[idx]);
    }

    template <typename Robot>
    __global__ void init_halton(HaltonState<Robot>* states, curandState* cr_states) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= d_settings.num_new_configs) return;
        int skip = (curand_uniform(&cr_states[idx]) * 50000.0f);
        if (idx == 0) skip = 0;
        // if (idx == 1) skip = 100000;
        halton_initialize(states[idx], skip, cr_states[idx], idx);
    }

    __device__ inline void print_config(float *config, int dim) {
        for (int i = 0; i < dim; i++) {
            printf("%f ,", config[i]);
        }
        printf("\n");
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

    __global__ void reset_device_variables_kernel() {
        solved = 0;
        
        path_size[0] = 0;
        path_size[1] = 0;
        
        for (int tree = 0; tree < 2; tree++) {
            for (int i = 0; i < 500; i++) {
                path[tree][i] = 0.0f;
            }
        }
        
        cost = 0.0f;
        reached_goal_idx = 0;
        }

    void reset_device_variables() {
        reset_device_variables_kernel<<<1, 1>>>();
        cudaDeviceSynchronize();
        cudaError_t error = cudaGetLastError();
        if (error != cudaSuccess) {
            printf("CUDA error: %s\n", cudaGetErrorString(error));
        }
    }
    
    /* each warp handles a separate RRTC tree, 32 threads grow the tree 1 at a time*/
    // 64 blocks, 128 threads per block, 4 warps per block, 256 trees total
    // 
    template <typename Robot>
    __global__ void
    __launch_bounds__(128, 4)
    rrtc(
        float **nodes,
        int **parents,
        // float **radii,
        HaltonState<Robot> *halton_states,
        curandState *rng_states,
        ppln::collision::Environment<float> *env,
        int num_goals
    )
    {
        static constexpr auto dim = Robot::dimension;
        // const int tid = blockIdx.x * blockDim.x + threadIdx.x; // global thread id
        const int lid = threadIdx.x % 32; // lane id
        const int wid = threadIdx.x / 32; // warp id
        const int bid = blockIdx.x; // block id
        const int tree_idx = bid * 4 + wid;
        // if (lid == 0) printf("initialized tree_idx: %d\n", tree_idx);
        // if (tree_idx >= NUM_TREES) return;
        __shared__ float config[4][dim]; // new config for each tree
        __shared__ int free_index[4][2]; // free indexes for each tree
        __shared__ float delta[4][dim]; // delta for each tree
        // __shared__ float var_cache[256][10]; // cache for forward kinematics
        int t_tree_id = 0;
        int o_tree_id = 1 - t_tree_id;
        float *t_nodes = nodes[t_tree_id] + (tree_idx * MAX_TREE_SIZE * dim);
        float *o_nodes = nodes[o_tree_id] + (tree_idx * MAX_TREE_SIZE * dim);
        int *t_parents = parents[t_tree_id] + (tree_idx * MAX_TREE_SIZE);
        int *o_parents = parents[o_tree_id] + (tree_idx * MAX_TREE_SIZE);
        unsigned int iter = 0;
        if (threadIdx.x < 4) {
            free_index[threadIdx.x][0] = 1;
            free_index[threadIdx.x][1] = num_goals;
        }
        __syncthreads();
        while (solved == 0) {
            // __syncthreads();
            iter ++;
            if (iter > d_settings.max_iters) {
                if (lid == 0) {
                    printf("DBG: Tree %d exceeded max iterations\n", tree_idx);
                }
                atomicCAS((int *)&solved, 0, -1);
            }
            // __syncthreads();

            if (d_settings.balance == 2) {
                float ratio = abs(free_index[wid][t_tree_id] - free_index[wid][o_tree_id]) / (float) free_index[wid][t_tree_id];
                if (ratio < d_settings.tree_ratio)
                {
                    t_tree_id = 1 - t_tree_id;
                    o_tree_id = 1 - t_tree_id;
                    float *temp_nodes = t_nodes;
                    t_nodes = o_nodes;
                    o_nodes = temp_nodes;
                    int *temp_parents = t_parents;
                    t_parents = o_parents;
                    o_parents = temp_parents;
                }
            }
            else { // swap every iteration
                t_tree_id = 1 - t_tree_id;
                o_tree_id = 1 - t_tree_id;
                float *temp_nodes = t_nodes;
                t_nodes = o_nodes;
                o_nodes = temp_nodes;
                int *temp_parents = t_parents;
                t_parents = o_parents;
                o_parents = temp_parents;
            }
            // __syncthreads();


            /* sample configuration */
            if (lid == 0) {
                // printf("DBG: Tree %d sampling configuration\n", tree_idx);
                halton_next(halton_states[tree_idx], (float *)config[wid]);
                Robot::scale_cfg((float *)config[wid]);
                // printf("DBG: Tree %d sampled config: ", tree_idx);
                // print_config(config[wid], dim);
            }
            // __syncthreads();


            /* find nearest neighbor */
            float min_dist = FLT_MAX;
            int min_index = -1;
            float dist;
            // __syncthreads();
            // if (lid == 0) printf("free_index: %d, %d | tree_idx: %d\n", free_index[wid][0], free_index[wid][1], tree_idx);
            // __syncthreads();
            for (unsigned int i = lid; i < free_index[wid][t_tree_id] && i < MAX_TREE_SIZE; i += 32) {
                // printf("i: %d, tree_idx: %d\n", i, tree_idx);
                // __syncthreads();
                dist = device_utils::sq_l2_dist(&t_nodes[i * dim], config[wid], dim);
                if (dist < min_dist) {
                    min_dist = dist;
                    min_index = i;
                }
            }
            __syncwarp();
            // __syncthreads();
            // if (lid == 0) printf("here3, tree_idx: %d\n", tree_idx);
            // __syncthreads();
            for (unsigned int offset = 16; offset > 0; offset /= 2) {
                float other_dist = __shfl_down_sync(FULL_MASK, min_dist, offset);
                int other_index = __shfl_down_sync(FULL_MASK, min_index, offset);
                if (other_dist < min_dist) {
                    min_dist = other_dist;
                    min_index = other_index;
                }
            }
            // __syncthreads();
            // if (lid == 0) printf("min_dist: %f, tree_idx: %d\n", min_dist, tree_idx);
            // __syncthreads();
            __syncwarp();
            min_dist = __shfl_sync(FULL_MASK, min_dist, 0);
            min_index = __shfl_sync(FULL_MASK, min_index, 0);
            // __syncwarp();
            min_dist = sqrt(min_dist);
            // printf("min_dist: %f\n", min_dist);
            // printf("min_index: %d\n", min_index);
            float scale = min(1.0f, d_settings.range / min_dist);
            float *nearest_node = &t_nodes[min_index * dim];
            // if (lid == 0) {
            //     printf("nearest node: ");
            //     print_config(nearest_node, dim);
            // }
            // __syncthreads();
            if (lid < dim) {
                // printf("setting config and delta for lid: %d, tree_idx: %d\n", lid, tree_idx);
                // __syncthreads();
                config[wid][lid] = (1 - scale) * nearest_node[lid] + scale * config[wid][lid];
                // printf("here4");
                // __syncthreads();
                delta[wid][lid] = (config[wid][lid] - nearest_node[lid]) / d_settings.granularity;
            }
            __syncwarp();
            // if (lid == 0) {
            //     printf("here4.5, tree_idx: %d\n", tree_idx);
            // }
            // __syncthreads();
            /* validate edge to config*/
            float interp_cfg[dim];
            bool config_in_collision = false;
            for (unsigned int i = lid; i < d_settings.granularity; i+=32) {
                // if (lid == 0) {
                //     printf("i: %d\n", i);
                // }
                // printf("lid: %d, wid: %d\n", lid, wid);
                for (unsigned int j = 0; j < dim; j++) {
                    interp_cfg[j] = nearest_node[j] + delta[wid][j] * (i + 1);
                }
                // volatile unsigned int local_cc_result = 0;
                // if (lid == 0) printf("here5, tree_idx: %d\n", tree_idx);
                // __syncthreads();
                config_in_collision = not ppln::collision::fkcc<Robot>(interp_cfg, env, threadIdx.x);
                // __syncthreads();
                // if (lid == 0) printf("here6, tree_idx: %d\n", tree_idx);
                // printf("tree_idx: %d, lid: %d", tree_idx, lid);
                // __syncthreads();
                __syncwarp();
                for (unsigned int offset = 16; offset > 0; offset /= 2) {
                    // if (lid == 0) printf("here7\n");
                    bool other_config_in_collision = __shfl_down_sync(FULL_MASK, config_in_collision, offset);
                    // printf("other_config_in_collision: %d\n", other_config_in_collision);
                    config_in_collision = config_in_collision || other_config_in_collision;
                }
                __syncwarp();
                // if (lid == 0) printf("config_in_collision: %d\n", config_in_collision);
                config_in_collision = __shfl_sync(FULL_MASK, config_in_collision, 0);
                if (config_in_collision) break;
            }
            // if (lid == 0) printf("here8\n");
            /* add new config to this tree */
            if (!config_in_collision) {
                // printf("lid: %d\n", lid);
                unsigned int index = free_index[wid][t_tree_id];
                if (lid == 0) {
                    if (index >= MAX_TREE_SIZE) {
                        atomicCAS((int *)&solved, 0, -1);
                    }
                    else {
                        free_index[wid][t_tree_id] = index + 1;
                        t_parents[index] = min_index;
                    }
                }
                // if (lid == 0) printf("here9\n");
                if (lid < dim) {
                    t_nodes[index * dim + lid] = config[wid][lid];
                }
                __syncwarp();
                // if (lid == 0) {
                //     printf("Added new config: ");
                //     print_config(&t_nodes[index * dim], dim);
                // }
                /* connect (find nearest node in other tree and attempt connection) */
                float min_dist = FLT_MAX;
                int min_index = -1;
                for (unsigned int i = lid; i < free_index[wid][o_tree_id]; i += 32) {
                    dist = device_utils::sq_l2_dist(&o_nodes[i * dim], config[wid], dim);
                    if (dist < min_dist) {
                        min_dist = dist;
                        min_index = i;
                    }
                }
                __syncwarp();
                for (unsigned int offset = 16; offset > 0; offset /= 2) {
                    float other_dist = __shfl_down_sync(FULL_MASK, min_dist, offset);
                    int other_index = __shfl_down_sync(FULL_MASK, min_index, offset);
                    if (other_dist < min_dist) {
                        min_dist = other_dist;
                        min_index = other_index;
                    }
                }
                min_dist = __shfl_sync(FULL_MASK, min_dist, 0);
                min_index = __shfl_sync(FULL_MASK, min_index, 0);
                min_dist = sqrt(min_dist);
                int n_extensions = ceil(min_dist / d_settings.range);
                // if (lid == 0) printf("here11\n");
                if (lid < dim) {
                    delta[wid][lid] = (o_nodes[min_index * dim + lid] - config[wid][lid]) / (float) n_extensions;
                }
                __syncwarp();
                // if (lid == 0) {
                //     printf("n_extensions: %d\n", n_extensions);
                //     printf("delta: ");
                //     print_config(delta[wid], dim);
                // }
                /* extend as far as we can to NN in opposite tree */
                int i_extensions = 0;
                int extension_parent_idx = index;
                while (i_extensions < n_extensions) {
                    // if (lid == 0) printf("i_extensions: %d\n", i_extensions);
                    /* collision check the extension */
                    bool config_in_collision = false;
                    for (unsigned int i = lid; i < d_settings.granularity; i+=32) {
                        // if (lid == 0) {
                        //     printf("i: %d\n", i);
                        // }
                        // printf("lid: %d, wid: %d\n", lid, wid);
                        for (unsigned int j = 0; j < dim; j++) {
                            interp_cfg[j] = config[wid][j] + ((delta[wid][j] / d_settings.granularity) * (i + 1));
                        }
                        // volatile unsigned int local_cc_result = 0;
                        // if (lid == 0) printf("here13\n");
                        config_in_collision = not ppln::collision::fkcc<Robot>(interp_cfg, env, threadIdx.x);
                        // if (lid == 0) printf("here14\n");
                        __syncwarp();
                        for (unsigned int offset = 16; offset > 0; offset /= 2) {
                            // if (lid == 0) printf("here15\n");
                            bool other_config_in_collision = __shfl_down_sync(FULL_MASK, config_in_collision, offset);
                            config_in_collision = config_in_collision || other_config_in_collision;
                        }
                        __syncwarp();
                        // if (lid == 0) printf("config_in_collision (ext): %d\n", config_in_collision);
                        config_in_collision = __shfl_sync(FULL_MASK, config_in_collision, 0);
                        if (config_in_collision) break;
                    }
                    if (config_in_collision) break;
                    /* add extended config to tree */
                    index = free_index[wid][t_tree_id];
                    if (lid == 0) {
                        // printf("extending: %d\n", i_extensions);
                        if (index >= MAX_TREE_SIZE) {
                            atomicCAS((int *)&solved, 0, -1);
                        }
                        else {
                            free_index[wid][t_tree_id] = index + 1;
                            t_parents[index] = extension_parent_idx;
                            extension_parent_idx = index;
                        }
                    }
                    if (lid < dim) {
                        config[wid][lid] = config[wid][lid] + delta[wid][lid];
                        t_nodes[index * dim + lid] = config[wid][lid];
                    }
                    __syncwarp();
                    // if (lid == 0) {
                    //     printf("Added config: ");
                    //     print_config(&t_nodes[index * dim], dim);
                    // }
                    // __syncwarp();
                    i_extensions ++;
                }
                /* check if we reached the goal */
                if (i_extensions == n_extensions) {
                    if (lid == 0 && atomicCAS((int *)&solved, 0, 1) == 0) {
                        // printf("ENTERED SOLUTION CODE\n");
                        // printf("tree_idx: %d\n", tree_idx);
                        // trace back to the start and goal.
                        int current = index;
                        int parent;
                        int t_path_size = 0;
                        int o_path_size = 0;
                        while (t_parents[current] != current) {
                            parent = t_parents[current];
                            cost += device_utils::l2_dist(&t_nodes[current * dim], &t_nodes[parent * dim], dim);
                            for (int i = 0; i < dim; i++) path[t_tree_id][t_path_size * dim + i] = t_nodes[current * dim + i];
                            t_path_size++;
                            current = parent;
                            
                        }
                        if (t_tree_id == 1) reached_goal_idx = current;
                        current = min_index;
                        while (o_parents[current] != current) {
                            parent = o_parents[current];
                            cost += device_utils::l2_dist(&o_nodes[current * dim], &o_nodes[parent * dim], dim);
                            for (int i = 0; i < dim; i++) path[o_tree_id][o_path_size * dim + i] = o_nodes[current * dim + i];
                            o_path_size++;
                            current = parent;
                        }
                        if (t_tree_id == 0) reached_goal_idx = current;
                        // printf("t_path_size: %d, o_path_size: %d\n", t_path_size, o_path_size);
                        path_size[t_tree_id] = t_path_size;
                        path_size[o_tree_id] = o_path_size;
                        solved_iters = iter;
                        solved_free_index[0] = free_index[wid][0];
                        solved_free_index[1] = free_index[wid][1];
                    }
                    __syncthreads();
                }
            }
            if (solved != 0) break;
        }
    }



    template <typename Robot>
    PlannerResult<Robot> solve(
        typename Robot::Configuration &start,
        std::vector<typename Robot::Configuration> &goals,
        ppln::collision::Environment<float> &h_environment,
        pRRTC_settings &settings
    ) 
    {
        // std::cout << "here" << std::endl;
        settings.max_samples = MAX_TREE_SIZE * NUM_TREES;
        cudaSetDevice(1);
        // std::cout << "here1" << std::endl;
        auto start_time = std::chrono::steady_clock::now();
        static constexpr auto dim = Robot::dimension;
        std::size_t start_index = 0;

        
        PlannerResult<Robot> res;
        // copy data to GPU
        cudaMemcpyToSymbol(d_settings, &settings, sizeof(settings));

        // std::cout << "here2" << std::endl;
        int num_goals = goals.size();
        float *nodes[2];
        int *parents[2];
        float *radii[2];
        float **d_nodes;
        int **d_parents;
        float **d_radii;
        cudaMalloc(&d_nodes, 2 * sizeof(float*));
        cudaMalloc(&d_parents, 2 * sizeof(int*));
        // cudaMalloc(&d_radii, 2 * sizeof(float*));
        const std::size_t config_size = dim * sizeof(float);

        for (int i = 0; i < 2; i++) {
            cudaMalloc(&nodes[i], settings.max_samples * config_size);
            cudaMalloc(&parents[i], settings.max_samples * sizeof(int));
            // cudaMalloc(&radii[i], settings.max_samples * sizeof(float));
        }
        cudaMemcpy(d_nodes, nodes, 2 * sizeof(float*), cudaMemcpyHostToDevice);
        cudaMemcpy(d_parents, parents, 2 * sizeof(int*), cudaMemcpyHostToDevice);
        // cudaMemcpy(d_radii, radii, 2 * sizeof(float*), cudaMemcpyHostToDevice);

        // std::cout << "here3" << std::endl;

        // add start and goal configurations to trees
        for (int i = 0; i < NUM_TREES; i++) {
            int global_idx = i * MAX_TREE_SIZE;
            cudaMemcpy(nodes[0] + (global_idx * dim), start.data(), config_size, cudaMemcpyHostToDevice);
            cudaMemcpy(parents[0] + global_idx, &start_index, sizeof(int), cudaMemcpyHostToDevice);

            cudaMemcpy(nodes[1] + (global_idx * dim), goals.data(), config_size * num_goals, cudaMemcpyHostToDevice);
            std::vector<int> parents_b_init(num_goals);
            iota(parents_b_init.begin(), parents_b_init.end(), 0); // consecutive integers from 0 ... num_goals - 1
            cudaMemcpy(parents[1] + global_idx, parents_b_init.data(), sizeof(int) * num_goals, cudaMemcpyHostToDevice);

            // std::vector<float> radii_init(num_goals, FLT_MAX);
            // cudaMemcpy(radii[0] + global_idx, radii_init.data(), sizeof(float), cudaMemcpyHostToDevice);
            // cudaMemcpy(radii[1] + global_idx, radii_init.data(), sizeof(float) * num_goals, cudaMemcpyHostToDevice);
        }
        cudaCheckError(cudaGetLastError());
        
        // create a curandState for each thread -> holds state of RNG for each thread seperately
        // For growing the tree we will create NUM_NEW_CONFIGS threads
        curandState *rng_states;
        int num_rng_states = NUM_TREES * 32;
        cudaMalloc(&rng_states, num_rng_states * sizeof(curandState));
        int numBlocks = (num_rng_states + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_rng<<<numBlocks, BLOCK_SIZE>>>(rng_states, 1, num_rng_states);

        HaltonState<Robot> *halton_states;
        cudaMalloc(&halton_states, NUM_TREES * sizeof(HaltonState<Robot>));
        int numBlocks1 = (NUM_TREES + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_halton<Robot><<<numBlocks1, BLOCK_SIZE>>>(halton_states, rng_states);

        
        // std::cout << "here5" << std::endl;
        // allocate for obstacles
        ppln::collision::Environment<float> *env;
        setup_environment_on_device(env, h_environment);
        // std::cout << "here6" << std::endl;
        cudaCheckError(cudaGetLastError());
        // Setup pinned memory for signaling
        int *h_solved;
        int current_samples[2];
        int h_solved_iters = -1;
        cudaMallocHost(&h_solved, sizeof(int));  // Pinned memory
        *h_solved = -1;
        // std::cout << "here7" << std::endl;
        auto kernel_start_time = std::chrono::steady_clock::now();
        rrtc<Robot><<<64, 128>>> (
            d_nodes,
            d_parents,
            // d_radii,
            halton_states,
            rng_states,
            env,
            num_goals
        );
        cudaDeviceSynchronize();
        res.kernel_ns = get_elapsed_nanoseconds(kernel_start_time);
        cudaCheckError(cudaGetLastError());
        
        // std::cout << "here8" << std::endl;
        

        cudaMemcpyFromSymbol(h_solved, solved, sizeof(int), 0, cudaMemcpyDeviceToHost);
        cudaMemcpyFromSymbol(&h_solved_iters, solved_iters, sizeof(int), 0, cudaMemcpyDeviceToHost);
        cudaMemcpyFromSymbol(current_samples, solved_free_index, sizeof(int) * 2, 0, cudaMemcpyDeviceToHost);
        // currently, iteration count is not copied because each block may have different iteration count
        if (*h_solved!=1) *h_solved=0;
        std::cout << "current_samples: start: " << current_samples[0] << ", goal: " << current_samples[1] << "\n";
        std::cout << "solved iters: " << h_solved_iters << "\n";
        // res.start_tree_size = current_samples[0];
        // res.goal_tree_size = current_samples[1];
        Robot::print_robot_config(start);
        Robot::print_robot_config(goals[0]);
        if (*h_solved) {
            std::cout << "solved!\n";
            int h_path_size[2];
            float h_paths[2][500];
            float h_cost;
            int h_reached_goal_idx;
            cudaMemcpyFromSymbol(h_path_size, path_size, sizeof(int) * 2, 0, cudaMemcpyDeviceToHost);
            cudaMemcpyFromSymbol(h_paths, path, sizeof(float) * 2 * 500, 0, cudaMemcpyDeviceToHost);
            cudaMemcpyFromSymbol(&h_cost, cost, sizeof(float), 0, cudaMemcpyDeviceToHost);
            cudaMemcpyFromSymbol(&h_reached_goal_idx, reached_goal_idx, sizeof(int), 0, cudaMemcpyDeviceToHost);
            cudaCheckError(cudaGetLastError());
            // Robot::print_robot_config(goals[h_reached_goal_idx]);
            res.path.emplace_back(goals[h_reached_goal_idx]);
            typename Robot::Configuration config;
            for (int i = h_path_size[1] - 1; i >= 0; i--) {
                std::copy_n(h_paths[1] + i * dim, dim, config.begin());
                res.path.emplace_back(config);
                // print_cfg_ptr<Robot>(&h_paths[1][i * dim]);
            }
            for (int i = 0; i < h_path_size[0]; i++) {
                std::copy_n(h_paths[0] + i * dim, dim, config.begin());
                res.path.emplace_back(config);
                // print_cfg_ptr<Robot>(&h_paths[0][i * dim]);
            }
            // Robot::print_robot_config(start);
            res.path.emplace_back(start);
            res.cost = h_cost;
            res.path_length = (h_path_size[0] + h_path_size[1]);
            std::cout << "cost: " << res.cost << "\n";
        }
        res.solved = (*h_solved) != 0;
        res.iters = h_solved_iters;
        
        cleanup_environment_on_device(env, h_environment);
        reset_device_variables();
        cudaFree(nodes[0]);
        cudaFree(nodes[1]);
        cudaFree(parents[0]);
        cudaFree(parents[1]);
        // cudaFree(radii[0]);
        // cudaFree(radii[1]);
        cudaFree(rng_states);
        cudaFree(halton_states);
        cudaFree(d_nodes);
        cudaFree(d_parents);
        // cudaFree(d_radii);
        cudaFreeHost(h_solved);
        cudaCheckError(cudaGetLastError());
        res.wall_ns = get_elapsed_nanoseconds(start_time);
        cudaDeviceReset();
        return res;
    }

    template PlannerResult<typename ppln::robots::Sphere> solve<ppln::robots::Sphere>(std::array<float, 3>&, std::vector<std::array<float, 3>>&, ppln::collision::Environment<float>&, pRRTC_settings&);
    template PlannerResult<typename ppln::robots::Panda> solve<ppln::robots::Panda>(std::array<float, 7>&, std::vector<std::array<float, 7>>&, ppln::collision::Environment<float>&, pRRTC_settings&);
    template PlannerResult<typename ppln::robots::Fetch> solve<ppln::robots::Fetch>(std::array<float, 8>&, std::vector<std::array<float, 8>>&, ppln::collision::Environment<float>&, pRRTC_settings&);
}
