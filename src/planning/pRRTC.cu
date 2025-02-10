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

#include <cooperative_groups.h>
namespace cg = cooperative_groups;


/*
Parallelized RRTC: Each block works to add a config to the tree (either start or goal depending on balance)
*/


namespace pRRTC {
    using namespace ppln;
    __device__ volatile int solved = 0;
    __device__ volatile int atomic_free_index[2]; // separate for tree_a and tree_b
    __device__ volatile int nodes_size[2];
    __device__ float path[2][500]; // solution path segments for tree_a, and tree_b
    __device__ int path_size[2] = {0, 0};
    __device__ float cost = 0.0;
    __device__ int reached_goal_idx = 0;
    __device__ int solved_iters = 0; // value of iters in the block that solves the problem
    __constant__ pRRTC_settings d_settings;
    __device__ int print_id = 0;
    // constexpr int MAX_SAMPLES = 1000000;
    // constexpr int MAX_ITERS = 1000000;
    // constexpr int NUM_NEW_CONFIGS = 600;
    constexpr int MAX_GRANULARITY = 256;
    // constexpr float RRT_RADIUS = 0.5;
    // constexpr float TREE_RATIO = 0.5;
    // constexpr bool balance = true;
    // constexpr bool dynamic_domain = true;
    // constexpr float ALPHA = 0.0001;
    // constexpr float dd_RADIUS = 4.0;
    // constexpr float dd_MIN_RADIUS = 1.0;
    // constexpr int NUM_SAMPLE_RETRY = 3;

    constexpr int BLOCK_SIZE = 64;
    constexpr float UNWRITTEN_VAL = -9999.0f;

    // Constants
    // __constant__ float primes[16] = {
    //     3.f, 5.f, 7.f, 11.f, 13.f, 17.f, 19.f, 23.f,
    //     29.f, 31.f, 37.f, 41.f, 43.f, 47.f, 53.f, 59.f
    // };

    template<typename Robot>
    struct HaltonState {
        float b[Robot::dimension];   // bases
        float n[Robot::dimension];   // numerators
        float d[Robot::dimension];   // denominators
    };

    void __device__ shuffle_array(float *array, int n, curandState &state) {
        for (int i = n - 1; i > 0; i--) {
            int j = curand(&state) % (i + 1);
            float temp = array[i];
            array[i] = array[j];
            array[j] = temp;
        }
    }

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
        // int skip = (curand_uniform(&cr_states[idx]) * 50000.0f);
        int skip = 0;
        if (idx == 0) skip = 0;
        halton_initialize(states[idx], skip, cr_states[idx], idx);
    }

    __device__ inline void print_config(volatile float *config, int dim) {
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
        
        atomic_free_index[0] = 0;
        atomic_free_index[1] = 0;
        nodes_size[0] = 0;
        nodes_size[1] = 0;
        
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

    __device__ __forceinline__ bool blockAnyTrue(bool localValue, int tid, int warpId, int laneId) {
        __shared__ bool shared[16];
        bool warpResult = __any_sync(0xffffffff, localValue);
        if (laneId == 0) shared[warpId] = warpResult;
        __syncthreads();
        if (warpId == 0) {
            bool blockResult = false;
            if (laneId < (blockDim.x + warpSize - 1) / warpSize) blockResult = shared[laneId];
            blockResult = __any_sync(0xffffffff, blockResult);
            if (laneId == 0) shared[0] = blockResult;
        }
        __syncthreads();
        return shared[0];
    }

    __device__ __forceinline__ bool check_partially_written(volatile float *node, int dim) {
        #pragma unroll
        for (int i = 0; i < dim; i++) {
            if (node[i] == UNWRITTEN_VAL) return true;
        }
        return false;
    }
    
    template <typename Robot>
    __global__ void
    __launch_bounds__(128, 8)
    rrtc(
        float **nodes,
        int **parents,
        float **radii,
        HaltonState<Robot> *halton_states,
        curandState *rng_states,
        ppln::collision::Environment<float> *env
    )
    {
        cg::grid_group grid = cg::this_grid();
        static constexpr auto dim = Robot::dimension;
        const int tid = threadIdx.x;
        const int bid = blockIdx.x; // 0 ... NUM_NEW_CONFIGS
        // const int lid = threadIdx.x%32;
        // const int wid = threadIdx.x/32;
        __shared__ int t_tree_id; // this tree
        __shared__ int o_tree_id; // the other tree
        __shared__ volatile float config[dim];
        __shared__ volatile float sdata[MAX_GRANULARITY];
        __shared__ volatile int sindex[MAX_GRANULARITY];
        __shared__ volatile unsigned int local_cc_result[1];
        __shared__ volatile float *t_nodes;
        __shared__ volatile float *o_nodes;
        __shared__ volatile int *t_parents;
        __shared__ volatile int *o_parents;
        __shared__ float scale;
        __shared__ volatile float *nearest_node;
        __shared__ float delta[dim];
        // __shared__ float var_cache[MAX_GRANULARITY][10];
        __shared__ volatile int index;
        __shared__ volatile float vec[dim];
        __shared__ unsigned int n_extensions;
        __shared__ bool should_skip;

        int iter = 0;

        while (true) {
            // if (solved != 0) return;
            // if (tid < dim) {
            //     config[tid] = curand_uniform(&rng_states[bid * dim + tid]);
            // }
            // __syncthreads();
            if (tid == 0) {
                // printf("iter: %d, bid: %d\n", iter, bid);
                iter++;
                if (iter > d_settings.max_iters) {
                    // printf("max iters reached from bid: %d\n", bid);
                    atomicCAS((int *)&solved, 0, -1);
                }

                if (d_settings.balance == 0 || iter == 1) {
                    t_tree_id = (bid < (d_settings.num_new_configs / 2))? 0 : 1;
                    o_tree_id = 1 - t_tree_id;
                }
                else if (d_settings.balance == 1 && abs(atomic_free_index[0]-atomic_free_index[1]) < 1.5 * d_settings.num_new_configs){
                    float ratio = atomic_free_index[0] / (float)(atomic_free_index[0]+atomic_free_index[1]);
                    float balance_factor = 1 - ratio;
                    t_tree_id = (bid < (d_settings.num_new_configs * balance_factor))? 0 : 1;
                    o_tree_id = 1 - t_tree_id;
                }
                else if (d_settings.balance == 1) {
                    float ratio = atomic_free_index[0] / (float)(atomic_free_index[0] + atomic_free_index[1]);
                    if (ratio < d_settings.tree_ratio) t_tree_id = 0;
                    else t_tree_id = 1;
                    o_tree_id = 1 - t_tree_id;
                }
                else if (d_settings.balance == 2) { // vamp balance algo
                    float ratio = abs(atomic_free_index[t_tree_id] - atomic_free_index[o_tree_id]) / (float) atomic_free_index[t_tree_id];
                    if (ratio < d_settings.tree_ratio)
                    {
                        t_tree_id = 1 - t_tree_id;
                        o_tree_id = 1 - t_tree_id;
                    }
                }

                t_nodes = nodes[t_tree_id];
                o_nodes = nodes[o_tree_id];
                t_parents = parents[t_tree_id];
                o_parents = parents[o_tree_id];
                
                halton_next(halton_states[bid], (float *)config);
                Robot::scale_cfg((float *)config);
                local_cc_result[0] = 0;
            }
            __syncthreads();
            // grid.sync();

            // if (solved != 0) {
            //     // if (tid == 0) printf("Exiting because solved = %d\n, bid: %d, iter: %d", solved, bid, iter);
            //     return;
            // }

            // divide up the work of finding nearest neighbor among the threads
            float local_min_dist = FLT_MAX;
            int local_near_idx = 0;
            float dist;
            // int size = nodes_size[t_tree_id];
            int size = atomic_free_index[t_tree_id];
            for (int i = tid; i < size; i += blockDim.x) {
                while (check_partially_written(&t_nodes[i * dim], dim)) {};
                dist = device_utils::sq_l2_dist((float *)&t_nodes[i * dim], (float *) config, dim);
                if (dist < local_min_dist) {
                    local_min_dist = dist;
                    local_near_idx = i;
                }
            }
            sdata[tid] = local_min_dist;
            sindex[tid] = local_near_idx;
            __syncthreads();

            for (unsigned int s = blockDim.x/2; s > 0; s >>= 1) {
                if (tid < s){
                    if (sdata[tid + s] < sdata[tid]) {
                        sdata[tid] = sdata[tid + s];
                        sindex[tid] = sindex[tid + s];
                    }
                }
                __syncthreads();
            }

            // by this point NN dist = sdata[0], NN index = sindex[0]
            // now calculate the extension
            volatile float nearest_node_copy[dim];
            if (tid == 0) {
                sdata[0] = sqrt(sdata[0]);
                scale = min(1.0f, d_settings.range / (sdata[0]));
                nearest_node = &t_nodes[sindex[0] * dim];
                for (int i = 0; i < dim; i++) {
                    nearest_node_copy[i] = nearest_node[i];
                }
                // printf("nearest node copy: %f %f %f %f %f %f %f %f\n sindex[0]: %d, t_tree_id: %d\n",
                //     nearest_node_copy[0], nearest_node_copy[1], nearest_node_copy[2], nearest_node_copy[3], nearest_node_copy[4], nearest_node_copy[5], nearest_node_copy[6], nearest_node_copy[7]
                //     , sindex[0], t_tree_id
                // );
                should_skip = (d_settings.dynamic_domain && radii[t_tree_id][sindex[0]] < sdata[0]);
                // printf("radius: %f, dist: %f\n, should_skip: %d", radii[t_tree_id][sindex[0]], sdata[0], should_skip);
                // printf("calculate extension\n");
            }
            __syncthreads();

            if (should_skip) {
                // if (tid == 0) printf("skipping iter %d\n", iter);
                continue;
            }
            __syncthreads();

            if (tid < dim) {
                config[tid] = nearest_node[tid] + ((config[tid] - nearest_node[tid]) * scale);
                delta[tid] = (config[tid] - nearest_node[tid]) / (float) d_settings.granularity;
            }
            __syncthreads();

            // if (solved != 0) return;
            
            /* validate_edges */
            float interp_cfg[dim];
            for (int i = 0; i < dim; i++) {
                interp_cfg[i] = nearest_node[i] + ((tid + 1) * delta[i]);
            }
            // __shared__ bool each_thread_result[128];
            bool config_in_collision = not ppln::collision::fkcc<Robot>(interp_cfg, env, tid);
            // each_thread_result[tid] = config_in_collision;
            // __syncthreads();
            // bool edge_good = not blockAnyTrue(config_in_collision, tid, wid, lid);
            atomicOr((unsigned int *)&local_cc_result[0], config_in_collision ? 1u : 0u);
            __syncthreads();
            bool edge_good = local_cc_result[0] == 0;
            // bool repeated_node = (sdata[0]==0);
            // grid.sync();

            // if (tid == 0) {
            //     // check edge with loop to verify
            //     // printf("env debug: num_spheres: %d, num_capsules: %d, num_cylinders: %d, num_cuboids: %d\n", env->num_spheres, env->num_capsules, env->num_cylinders, env->num_cuboids);
            //     bool debug_collision = false;
            //     float debug_delta[dim];
            //     float debug_interp_cfg[dim];
            //     for (int i = 0; i < dim; i++) {
            //         debug_delta[i] = (config[i] - nearest_node[i]) / (float) d_settings.granularity;
            //     }
            //     for (int i = 0; i < d_settings.granularity; i++) {
            //         for (int j = 0; j < dim; j++) {
            //             debug_interp_cfg[j] = nearest_node[j] + ((i + 1) * debug_delta[j]);
            //         }
            //         config_in_collision = not ppln::collision::fkcc<Robot>(debug_interp_cfg, env, tid);
            //         if (config_in_collision) {
            //             debug_collision = true;
            //             break;
            //         }
            //     }

            //     // bool debug_collision_2 = false;
            //     // for (int i = 0; i < 128; i++) {
            //     //     if (each_thread_result[i]) {
            //     //         debug_collision_2 = true;
            //     //         break;
            //     //     }
            //     // }

            //     // int l_print_id = atomicAdd(&print_id, 1);
            //     // printf("%d checked edge between: %f %f %f %f %f %f %f %f\n and %f %f %f %f %f %f %f %f. Result: %d, Debug1: %d, Debug2: %d, sindex[0]: %d\n, Nearest Node copy: %f %f %f %f %f %f %f %f\n", 
            //     //     l_print_id,
            //     //     nearest_node[0], nearest_node[1], nearest_node[2], nearest_node[3], nearest_node[4], nearest_node[5], nearest_node[6], nearest_node[7],
            //     //     config[0], config[1], config[2], config[3], config[4], config[5], config[6], config[7], edge_good, !debug_collision, !debug_collision_2, sindex[0],
            //     //     nearest_node_copy[0], nearest_node_copy[1], nearest_node_copy[2], nearest_node_copy[3], nearest_node_copy[4], nearest_node_copy[5], nearest_node_copy[6], nearest_node_copy[7]
            //     // );
                
            //     // printf("%d Debug delta: %f %f %f %f %f %f %f %f\n %f %f %f %f %f %f %f %f\n",
            //     // l_print_id,
            //     // delta[0], delta[1], delta[2], delta[3], delta[4], delta[5], delta[6], delta[7],
            //     // debug_delta[0], debug_delta[1], debug_delta[2], debug_delta[3], debug_delta[4], debug_delta[5], debug_delta[6], debug_delta[7]
            //     // );
                
            // }
            __syncthreads();
            if (edge_good) {
                /* grow tree */
                if (tid == 0) {
                    
                    index = atomicAdd((int *)&atomic_free_index[t_tree_id], 1);
                    if (index >= d_settings.max_samples) solved = -1;
                    
                    t_parents[index] = sindex[0];
                    
                    if (d_settings.dynamic_domain) {
                        radii[t_tree_id][index] = FLT_MAX;
                        volatile float *radius_ptr = &radii[t_tree_id][sindex[0]];
                        float old_radius, new_radius;
                        int expected, desired;
                        do {
                            // printf("dynamic domain loop 1\n");
                            old_radius = *radius_ptr;
                            if (old_radius == FLT_MAX) break;
                            new_radius = old_radius * (1 + d_settings.dd_alpha);
                            expected = __float_as_int(old_radius);
                            desired = __float_as_int(new_radius);
                        } while (atomicCAS((int *)radius_ptr, expected, desired) != expected);
                    }
                }
                __syncthreads();

                if (tid < dim) {
                    t_nodes[index * dim + tid] = config[tid];
                }
                __syncthreads();
                __threadfence_system();
            }
            // grid.sync();
            if (edge_good) {
                // if (tid == 0) {
                //     atomicAdd((int *)&nodes_size[t_tree_id], 1);
                //     // printf("added config to tree %d at index %d (bid %d, parent %d): %f %f %f %f %f %f %f %f\n parent: %f %f %f %f %f %f %f %f\n", 
                //     //         t_tree_id, index, bid, sindex[0], config[0], config[1], config[2], config[3], config[4], config[5], config[6], config[7],
                //     //         nearest_node[0], nearest_node[1], nearest_node[2], nearest_node[3], nearest_node[4], nearest_node[5], nearest_node[6], nearest_node[7]
                //     //     );
                // }
                // __syncthreads();
                /* connect */
                local_min_dist = INFINITY;
                local_near_idx = 0;
                // int size = nodes_size[o_tree_id];
                int size = atomic_free_index[o_tree_id];
                for (unsigned int i = tid; i < size; i += blockDim.x) {
                    while (check_partially_written(&o_nodes[i * dim], dim)) {};
                    dist = device_utils::sq_l2_dist((float *)&o_nodes[i * dim], (float *)config, dim);
                    if (dist < local_min_dist) {
                        local_min_dist = dist;
                        local_near_idx = i;
                    }
                }
                sdata[tid] = local_min_dist;
                sindex[tid] = local_near_idx;
                __syncthreads();
                
                for (unsigned int s = blockDim.x/2; s > 0; s >>= 1) {
                    if (tid < s) {
                        if (sdata[tid + s] < sdata[tid]) {
                            sdata[tid] = sdata[tid + s];
                            sindex[tid] = sindex[tid + s];
                        }
                    }
                    __syncthreads();
                }
                
                
                if (tid == 0) {
                    sdata[0] = sqrt(sdata[0]);
                    nearest_node = &o_nodes[sindex[0] * dim];
                    n_extensions = ceil(sdata[0] / d_settings.range);
                    local_cc_result[0] = 0;
                    // printf("found closest in other tree\n");
                }
                __syncthreads();

                if (tid < dim) {
                    vec[tid] = (nearest_node[tid] - config[tid]) / (float) n_extensions;
                }
                __syncthreads();
            }
            // grid.sync();
            if (edge_good) {
                // validate the edge to the nearest neighbor in opposite tree, go as far as we can
                int i_extensions = 0;
                int extension_parent_idx = index;
                // printf("here6\n");
                while (i_extensions < n_extensions) {
                    /* each thread checking an interpolated config along the extension vector*/
                    for (int i = 0; i < dim; i++) {
                        interp_cfg[i] = config[i] + ((tid + 1) * (vec[i] / (float) d_settings.granularity));
                    }
                    __syncthreads();
                    bool config_in_collision = not ppln::collision::fkcc<Robot>(interp_cfg, env, tid);
                    // bool edge_good = not blockAnyTrue(config_in_collision, tid, wid, lid);
                    atomicOr((unsigned int *)&local_cc_result[0], config_in_collision ? 1u : 0u);
                    __syncthreads();
                    bool ext_edge_good = local_cc_result[0] == 0;
                    if (!ext_edge_good) break;
                    // if (local_cc_result[0] != 0) break;
                    /* add extension to tree */
                    if (tid == 0) {
                        index = atomicAdd((int *)&atomic_free_index[t_tree_id], 1);
                        if (index >= d_settings.max_samples) solved = -1;
                        t_parents[index] = extension_parent_idx;
                        radii[t_tree_id][index] = FLT_MAX;
                        extension_parent_idx = index;
                        local_cc_result[0] = 0;
                        // printf("in extension loop\n");
                    }
                    __syncthreads();
                    if (tid < dim) {
                        config[tid] = config[tid] + vec[tid];
                        t_nodes[index * dim + tid] = config[tid];
                    }
                    __syncthreads();
                    __threadfence_system();
                    // if (tid == 0) {
                    //     atomicAdd((int *)&nodes_size[t_tree_id], 1);
                    //     // printf("added config from extension to tree %d at index %d (bid %d, parent %d): %f %f %f %f %f %f %f %f\n", t_tree_id, index, bid, t_parents[index], config[0], config[1], config[2], config[3], config[4], config[5], config[6], config[7]);
                    //     // print_config(config, dim);
                    // }
                    i_extensions++;
                    __syncthreads();
                }
                if (i_extensions == n_extensions) { // connected
                    if (tid == 0 && atomicCAS((int *)&solved, 0, 1) == 0) {
                        // printf("in connected\n");
                        // printf("config at connection: %f %f %f %f %f %f %f %f\n nearest node: %f %f %f %f %f %f %f %f\n", 
                        //     config[0], config[1], config[2], config[3], config[4], config[5], config[6], config[7],
                        //     nearest_node[0], nearest_node[1], nearest_node[2], nearest_node[3], nearest_node[4], nearest_node[5], nearest_node[6], nearest_node[7]
                        // );
                        // trace back to the start and goal.
                        int current = index;
                        int parent;
                        int t_path_size = 0;
                        int o_path_size = 0;
                        while (t_parents[current] != current) {
                            parent = t_parents[current];
                            cost += device_utils::l2_dist((float *)&t_nodes[current * dim], (float *)&t_nodes[parent * dim], dim);
                            for (int i = 0; i < dim; i++) path[t_tree_id][t_path_size * dim + i] = t_nodes[current * dim + i];
                            // printf("added to path[%d]: %f %f %f %f %f %f %f %f\n", t_tree_id, path[t_tree_id][t_path_size * dim], path[t_tree_id][t_path_size * dim + 1], path[t_tree_id][t_path_size * dim + 2], path[t_tree_id][t_path_size * dim + 3], path[t_tree_id][t_path_size * dim + 4], path[t_tree_id][t_path_size * dim + 5], path[t_tree_id][t_path_size * dim + 6], path[t_tree_id][t_path_size * dim + 7]);
                            //print_config(&t_nodes[current * dim], dim);
                            t_path_size++;
                            current = parent;
                            
                        }
                        //printf("GPU path above");
                        if (t_tree_id == 1) reached_goal_idx = current;
                        current = sindex[0];
                        // printf("sindex[0]: %d\n", sindex[0]);
                        // printf("entered here2\n");
                        while(o_parents[current] != current) {
                            parent = o_parents[current];
                            cost += device_utils::l2_dist((float *)&o_nodes[current * dim], (float *)&o_nodes[parent * dim], dim);
                            for (int i = 0; i < dim; i++) path[o_tree_id][o_path_size * dim + i] = o_nodes[current * dim + i];
                            // printf("added to path[%d]: %f %f %f %f %f %f %f %f\n", o_tree_id, path[o_tree_id][o_path_size * dim], path[o_tree_id][o_path_size * dim + 1], path[o_tree_id][o_path_size * dim + 2], path[o_tree_id][o_path_size * dim + 3], path[o_tree_id][o_path_size * dim + 4], path[o_tree_id][o_path_size * dim + 5], path[o_tree_id][o_path_size * dim + 6], path[o_tree_id][o_path_size * dim + 7]);
                            //print_config(&o_nodes[current * dim], dim);
                            o_path_size++;
                            current = parent;
                        }
                        //printf("GPU path above 2");
                        if (t_tree_id == 0) reached_goal_idx = current;
                        path_size[t_tree_id] = t_path_size;
                        path_size[o_tree_id] = o_path_size;
                        solved_iters = iter;
                        
                        // printf("t_tree_id %d, t_path_size: %d, o_path_size: %d\n", t_tree_id, t_path_size, o_path_size);
                        // return;
                    }
                    __syncthreads();
                }
                // printf("here8\n");
            }
            else if (d_settings.dynamic_domain && tid == 0) {         
                volatile float *radius_ptr = &radii[t_tree_id][sindex[0]];
                float old_radius, new_radius;
                int expected, desired;
                do {
                    // printf("in d settings end\n");
                    old_radius = *radius_ptr;
                    if (old_radius == FLT_MAX) {
                        new_radius = d_settings.dd_radius;
                    } else {
                        new_radius = fmaxf(old_radius * (1.f - d_settings.dd_alpha), d_settings.dd_min_radius);
                    }
                    expected = __float_as_int(old_radius);
                    desired = __float_as_int(new_radius);
                } while (atomicCAS((int *)radius_ptr, expected, desired) != expected);
            }
            __syncthreads();
            // grid.sync();
            if (solved != 0) return;
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
        cudaSetDevice(1);
        // std::cout << "here1" << std::endl;
        auto start_time = std::chrono::steady_clock::now();
        static constexpr auto dim = Robot::dimension;
        // std::size_t iter = 0;
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
        cudaMalloc(&d_radii, 2 * sizeof(float*));
        const std::size_t config_size = dim * sizeof(float);

        for (int i = 0; i < 2; i++) {
            cudaMalloc(&nodes[i], settings.max_samples * config_size);
            cudaMalloc(&parents[i], settings.max_samples * sizeof(int));
            cudaMalloc(&radii[i], settings.max_samples * sizeof(float));
        }
        cudaMemcpy(d_nodes, nodes, 2 * sizeof(float*), cudaMemcpyHostToDevice);
        cudaMemcpy(d_parents, parents, 2 * sizeof(int*), cudaMemcpyHostToDevice);
        cudaMemcpy(d_radii, radii, 2 * sizeof(float*), cudaMemcpyHostToDevice);

        // set nodes to unitialized
        std::vector<float> nodes_init(settings.max_samples * dim, UNWRITTEN_VAL);
        cudaMemcpy((void *)nodes[0], nodes_init.data(), config_size * settings.max_samples, cudaMemcpyHostToDevice);
        cudaMemcpy((void *)nodes[1], nodes_init.data(), config_size * settings.max_samples, cudaMemcpyHostToDevice);
        // add start to tree_a and goals to tree_b
        cudaMemcpy((void *)nodes[0], start.data(), config_size, cudaMemcpyHostToDevice);
        cudaMemcpy((void *)parents[0], &start_index, sizeof(int), cudaMemcpyHostToDevice);

        cudaMemcpy((void *)nodes[1], goals.data(), config_size * num_goals, cudaMemcpyHostToDevice);
        std::vector<int> parents_b_init(num_goals);
        iota(parents_b_init.begin(), parents_b_init.end(), 0); // consecutive integers from 0 ... num_goals - 1
        cudaMemcpy((void *)parents[1], parents_b_init.data(), sizeof(int) * num_goals, cudaMemcpyHostToDevice);

        // initialize radii
        std::vector<float> radii_init(num_goals, FLT_MAX);
        cudaMemcpy((void *)radii[0], radii_init.data(), sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy((void *)radii[1], radii_init.data(), sizeof(float) * num_goals, cudaMemcpyHostToDevice);
        // std::cout << "here4" << std::endl;
        // create a curandState for each thread -> holds state of RNG for each thread seperately
        // For growing the tree we will create NUM_NEW_CONFIGS threads
        curandState *rng_states;
        int num_rng_states = settings.num_new_configs * dim;
        cudaMalloc(&rng_states, num_rng_states * sizeof(curandState));
        int numBlocks = (num_rng_states + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_rng<<<numBlocks, BLOCK_SIZE>>>(rng_states, 1, num_rng_states);

        HaltonState<Robot> *halton_states;
        cudaMalloc(&halton_states, settings.num_new_configs * sizeof(HaltonState<Robot>));
        int numBlocks1 = (settings.num_new_configs + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_halton<Robot><<<numBlocks1, BLOCK_SIZE>>>(halton_states, rng_states);

        // free index for next available position in tree_a and tree_b
        int h_free_index[2] = {1, num_goals};
        cudaMemcpyToSymbol(atomic_free_index, &h_free_index, sizeof(int) * 2);
        cudaMemcpyToSymbol(nodes_size, &h_free_index, sizeof(int) * 2);
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


        auto kernel_start_time = std::chrono::steady_clock::now();
        rrtc<Robot><<<settings.num_new_configs, settings.granularity>>> (
            d_nodes,
            d_parents,
            d_radii,
            halton_states,
            rng_states,
            env
        );
        cudaDeviceSynchronize();
        res.kernel_ns = get_elapsed_nanoseconds(kernel_start_time);
        cudaCheckError(cudaGetLastError());

        // void* kernelArgs[] = {
        //     (void*)&d_nodes,
        //     (void*)&d_parents,
        //     (void*)&d_radii,
        //     (void*)&halton_states,
        //     (void*)&rng_states,
        //     (void*)&env
        // };
        // auto kernel_start_time = std::chrono::steady_clock::now();
        // cudaError_t err = cudaLaunchCooperativeKernel(
        //     (void*)rrtc<Robot>,  // Kernel function
        //     settings.num_new_configs, settings.granularity,  // Grid and block dimensions
        //     kernelArgs,  // Kernel arguments
        //     0  // Shared memory per block (set to 0 so the compiler and auto compute)
        // );
        // if (err != cudaSuccess) {
        //     std::cerr << "CUDA Kernel launch failed: " << cudaGetErrorString(err) << "\n";
        // }
        // cudaDeviceSynchronize();
        // cudaCheckError(cudaGetLastError());
        // res.kernel_ns = get_elapsed_nanoseconds(kernel_start_time);
        
        
        cudaMemcpyFromSymbol(current_samples, atomic_free_index, sizeof(int) * 2, 0, cudaMemcpyDeviceToHost);
        // cudaMemcpyFromSymbol(current_samples, nodes_size, sizeof(int) * 2, 0, cudaMemcpyDeviceToHost);
        cudaMemcpyFromSymbol(h_solved, solved, sizeof(int), 0, cudaMemcpyDeviceToHost);
        cudaMemcpyFromSymbol(&h_solved_iters, solved_iters, sizeof(int), 0, cudaMemcpyDeviceToHost);

        // currently, iteration count is not copied because each block may have different iteration count
        if (*h_solved!=1) *h_solved=0;
        std::cout << "current_samples: start: " << current_samples[0] << ", goal: " << current_samples[1] << "\n";
        std::cout << "solved iters: " << h_solved_iters << "\n";
        res.start_tree_size = current_samples[0];
        res.goal_tree_size = current_samples[1];
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
        cudaFree((void *)nodes[0]);
        cudaFree((void *)nodes[1]);
        cudaFree((void *)parents[0]);
        cudaFree((void *)parents[1]);
        cudaFree((void *)radii[0]);
        cudaFree((void *)radii[1]);
        cudaFree(rng_states);
        cudaFree(halton_states);
        cudaFree(d_nodes);
        cudaFree(d_parents);
        cudaFree(d_radii);
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
