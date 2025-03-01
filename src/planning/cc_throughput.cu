#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <numeric>

#include "src/collision/environment.hh"
#include "src/collision/factory.hh"
#include "src/planning/Planners.hh"
#include "src/planning/pRRTC_settings.hh"
#include "src/planning/utils.cuh"
#include "src/planning/cc_throughput.hh"

#include <curand.h>
#include <curand_kernel.h>
#include <float.h>




// Script to check throughput of edge collision checking
namespace bench_cc {
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
    __global__ void init_halton(HaltonState<Robot>* states, curandState* cr_states, int num_halton_states) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= num_halton_states) return;
        // int skip = (curand_uniform(&cr_states[idx]) * 50000.0f);
        int skip = 0;
        if (idx == 0) skip = 0;
        halton_initialize(states[idx], skip, cr_states[idx], idx);
    }

    template <typename Robot>
    __global__ void
    __launch_bounds__(32, 4)
    run_cc(ppln::collision::Environment<float> *env, HaltonState<Robot> *halton_states, int edges_per_block) 
    {
        // constexpr auto dim = Robot::dimension;
        // const int tid = threadIdx.x;
        // const int bid = blockIdx.x;
        // // sample 
        // __shared__ float edge_start[dim];
        // __shared__ float edge_end[dim];
        // __shared__ unsigned int local_cc_result[1];
        // __shared__ float delta[dim];
        // __shared__ int n;
        // float config[dim];
        // for (int i = 0; i < edges_per_block; i++) {
        //     if (tid == 0) {
        //         halton_next(halton_states[bid], (float *)edge_start);
        //         halton_next(halton_states[bid], (float *)edge_end);
        //         Robot::scale_cfg((float *)edge_start);
        //         Robot::scale_cfg((float *)edge_end);
        //         n = max(ceil(sqrt(device_utils::sq_l2_dist(edge_start, edge_end, dim))), 1.0f);
        //         local_cc_result[0] = 0;
        //     }
        //     __syncthreads();
        //     if (tid < dim) {
        //         delta[tid] = edge_end[tid] - edge_start[tid] / (float) (blockDim.x * n);
        //     }
        //     __syncthreads();
        //     # pragma unroll
        //     for (int j = 0; j < dim; j++) {
        //         config[j] = edge_start[j] + delta[j] * (tid * n);
        //     }
        //     for (int i = 0; i < n; i++) {
        //         bool config_in_collision = not ppln::collision::fkcc<Robot>(config, env, tid);
        //         atomicOr((unsigned int *)&local_cc_result[0], config_in_collision ? 1u : 0u);
        //         __syncthreads();
        //         if (local_cc_result[0]) break;
        //         # pragma unroll
        //         for (int j = 0; j < dim; j++) {
        //             config[j] += delta[j];
        //         }
        //     }
        // }

        constexpr auto dim = Robot::dimension;
        const int tid = threadIdx.x;
        const int bid = blockIdx.x;
        // sample 
        __shared__ float edge_start[dim];
        __shared__ float edge_end[dim];
        __shared__ unsigned int local_cc_result[1];
        __shared__ float delta[dim];
        __shared__ float dist;
        float config[dim];
        for (int i = 0; i < edges_per_block; i++) {
            if (tid == 0) {
                halton_next(halton_states[bid], (float *)edge_start);
                halton_next(halton_states[bid], (float *)edge_end);
                Robot::scale_cfg((float *)edge_start);
                Robot::scale_cfg((float *)edge_end);
                dist = sqrt(device_utils::sq_l2_dist(edge_start, edge_end, dim));

            }
            __syncthreads();
            if (tid < dim) {
                delta[tid] = (edge_end[tid] - edge_start[tid]) / (dist * blockDim.x);
            }
            __syncthreads();
            #pragma unroll
            for (int j = 0; j < dim; j++) {
                config[j] = edge_start[j] + delta[j] * (tid);
            }
            bool config_in_collision = not ppln::collision::fkcc<Robot>(config, env, tid);
            atomicOr((unsigned int *)&local_cc_result[0], config_in_collision ? 1u : 0u);
            __syncthreads();
        }
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






    template <typename Robot>
    size_t test_cc_throughput(ppln::collision::Environment<float> &h_environment, int num_blocks, int granularity, int edges_per_block) {
        ppln::collision::Environment<float> *d_env;
        setup_environment_on_device(d_env, h_environment);

        // Initialize random number generator
        const int BLOCK_SIZE = 256;
        curandState *rng_states;
        int num_rng_states = num_blocks;
        cudaMalloc(&rng_states, num_rng_states * sizeof(curandState));
        int numBlocks = (num_rng_states + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_rng<<<numBlocks, BLOCK_SIZE>>>(rng_states, 1, num_rng_states);

        HaltonState<Robot> *halton_states;
        int num_halton_states = num_blocks;
        cudaMalloc(&halton_states, num_halton_states * sizeof(HaltonState<Robot>));
        int numBlocks1 = (num_halton_states + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_halton<Robot><<<numBlocks1, BLOCK_SIZE>>>(halton_states, rng_states, num_halton_states);

        // warm up
        run_cc<Robot><<<num_blocks, granularity>>>(d_env, halton_states, edges_per_block);

        int iterations = 10;
        auto start_time = std::chrono::steady_clock::now();
        for (int i = 0; i < iterations; i++)
            run_cc<Robot><<<num_blocks, granularity>>>(d_env, halton_states, edges_per_block);
        cudaDeviceSynchronize();
        auto nanoseconds = get_elapsed_nanoseconds(start_time) / iterations;
        cudaCheckError(cudaGetLastError());
        
        cleanup_environment_on_device(d_env, h_environment);
        cudaFree(rng_states);
        cudaFree(halton_states);
        return nanoseconds;
    }

    template size_t test_cc_throughput<typename ppln::robots::Panda>(ppln::collision::Environment<float> &h_environment, int num_blocks, int granularity, int edges_per_block);
    template size_t test_cc_throughput<typename ppln::robots::Fetch>(ppln::collision::Environment<float> &h_environment, int num_blocks, int granularity, int edges_per_block);
    template size_t test_cc_throughput<typename ppln::robots::Baxter>(ppln::collision::Environment<float> &h_environment, int num_blocks, int granularity, int edges_per_block);
} // namespace bench_cc