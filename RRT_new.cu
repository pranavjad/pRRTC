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
    // list of configs sampled by cpu rrt when solving table_pick 3
    __device__ float set_configs[] = {
        0.333333, 0.2, 0.142857, 0.0909091, 0.0769231, 0.0588235, 0.0526316,
        0.666667, 0.4, 0.285714, 0.181818, 0.153846, 0.117647, 0.105263,
        0.111111, 0.6, 0.428571, 0.272727, 0.230769, 0.176471, 0.157895,
        0.444444, 0.8, 0.571429, 0.363636, 0.307692, 0.235294, 0.210526,
        0.777778, 0.04, 0.714286, 0.454545, 0.384615, 0.294118, 0.263158,
        0.222222, 0.24, 0.857143, 0.545455, 0.461538, 0.352941, 0.315789,
        0.555556, 0.44, 0.0204082, 0.636364, 0.538462, 0.411765, 0.368421,
        0.888889, 0.64, 0.163265, 0.727273, 0.615385, 0.470588, 0.421053,
        0.037037, 0.84, 0.306122, 0.818182, 0.692308, 0.529412, 0.473684,
        0.37037, 0.08, 0.44898, 0.909091, 0.769231, 0.588235, 0.526316,
        0.703704, 0.28, 0.591837, 0.00826446, 0.846154, 0.647059, 0.578947,
        0.148148, 0.48, 0.734694, 0.0991736, 0.923077, 0.705882, 0.631579,
        0.481481, 0.68, 0.877551, 0.190083, 0.00591716, 0.764706, 0.684211,
        0.814815, 0.88, 0.0408163, 0.280992, 0.0828402, 0.823529, 0.736842,
        0.259259, 0.12, 0.183673, 0.371901, 0.159763, 0.882353, 0.789474,
        0.592593, 0.32, 0.326531, 0.46281, 0.236686, 0.941176, 0.842105,
        0.925926, 0.52, 0.469388, 0.553719, 0.313609, 0.00346021, 0.894737,
        0.0740741, 0.72, 0.612245, 0.644628, 0.390533, 0.0622837, 0.947368,
        0.407407, 0.92, 0.755102, 0.735537, 0.467456, 0.121107, 0.00277008,
        0.740741, 0.16, 0.897959, 0.826446, 0.544379, 0.179931, 0.0554017,
        0.185185, 0.36, 0.0612245, 0.917355, 0.621302, 0.238754, 0.108033,
        0.518519, 0.56, 0.204082, 0.0165289, 0.698225, 0.297578, 0.160665,
        0.851852, 0.76, 0.346939, 0.107438, 0.775148, 0.356401, 0.213296,
        0.296296, 0.96, 0.489796, 0.198347, 0.852071, 0.415225, 0.265928,
        0.62963, 0.008, 0.632653, 0.289256, 0.928994, 0.474048, 0.31856,
        0.962963, 0.208, 0.77551, 0.380165, 0.0118343, 0.532872, 0.371191,
        0.0123457, 0.408, 0.918367, 0.471074, 0.0887574, 0.591695, 0.423823,
        0.345679, 0.608, 0.0816327, 0.561983, 0.16568, 0.650519, 0.476454,
        0.679012, 0.808, 0.22449, 0.652893, 0.242604, 0.709343, 0.529086,
        0.123457, 0.048, 0.367347, 0.743802, 0.319527, 0.768166, 0.581717,
        0.45679, 0.248, 0.510204, 0.834711, 0.39645, 0.82699, 0.634349,
        0.790123, 0.448, 0.653061, 0.92562, 0.473373, 0.885813, 0.686981,
        0.234568, 0.648, 0.795918, 0.0247934, 0.550296, 0.944637, 0.739612,
        0.567901, 0.848, 0.938776, 0.115702, 0.627219, 0.00692042, 0.792244,
        0.901235, 0.088, 0.102041, 0.206612, 0.704142, 0.0657439, 0.844875,
        0.0493827, 0.288, 0.244898, 0.297521, 0.781065, 0.124567, 0.897507,
        0.382716, 0.488, 0.387755, 0.38843, 0.857988, 0.183391, 0.950139,
        0.716049, 0.688, 0.530612, 0.479339, 0.934911, 0.242215, 0.00554017,
        0.160494, 0.888, 0.673469, 0.570248, 0.0177515, 0.301038, 0.0581717,
        0.493827, 0.128, 0.816327, 0.661157, 0.0946746, 0.359862, 0.110803,
        0.82716, 0.328, 0.959184, 0.752066, 0.171598, 0.418685, 0.163435,
        0.271605, 0.528, 0.122449, 0.842975, 0.248521, 0.477509, 0.216066,
        0.604938, 0.728, 0.265306, 0.933884, 0.325444, 0.536332, 0.268698,
        0.938272, 0.928, 0.408163, 0.0330578, 0.402367, 0.595156, 0.32133,
        0.0864198, 0.168, 0.55102, 0.123967, 0.47929, 0.653979, 0.373961,
        0.419753, 0.368, 0.693878, 0.214876, 0.556213, 0.712803, 0.426593,
        0.753086, 0.568, 0.836735, 0.305785, 0.633136, 0.771626, 0.479224,
        0.197531, 0.768, 0.979592, 0.396694, 0.710059, 0.83045, 0.531856,
        0.530864, 0.968, 0.00291545, 0.487603, 0.786982, 0.889273, 0.584488,
        0.864198, 0.016, 0.145773, 0.578512, 0.863905, 0.948097, 0.637119,
        0.308642, 0.216, 0.28863, 0.669421, 0.940828, 0.0103806, 0.689751,
        0.641975, 0.416, 0.431487, 0.760331, 0.0236686, 0.0692042, 0.742382,
        0.975309, 0.616, 0.574344, 0.85124, 0.100592, 0.128028, 0.795014,
        0.0246914, 0.816, 0.717201, 0.942149, 0.177515, 0.186851, 0.847645,
        0.358025, 0.056, 0.860058, 0.0413223, 0.254438, 0.245675, 0.900277,
        0.691358, 0.256, 0.0233236, 0.132231, 0.331361, 0.304498, 0.952909
    };
    __device__ int set_cfg_idx = 0;

    __device__ int atomic_free_index;
    __device__ int reached_goal = 0;
    __device__ int reached_goal_idx = -1;
    __device__ int goal_parent_idx = -1;

    constexpr int MAX_SAMPLES = 1000000;
    constexpr int MAX_ITERS = 1000000;
    constexpr int NUM_NEW_CONFIGS = 1;
    constexpr int GRANULARITY = 256;
    constexpr float RRT_RADIUS = 1.0;

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
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        int goal_idx = idx / NUM_NEW_CONFIGS;
        int config_idx = idx % NUM_NEW_CONFIGS; 
        if (goal_idx >= num_goals || config_idx >= NUM_NEW_CONFIGS) return;
        if (cc_result[config_idx] != 0) return;

        float delta[dim];
        float config[dim];
        float *new_config = &new_configs[config_idx * dim];
        float *goal = &goal_configs[goal_idx * dim];
        for (int i = 0; i < dim; i++) {
            delta[i] = (goal[i] - new_config[i]) / (float) GRANULARITY;
            config[i] = new_config[i] + delta[i];
        }

        bool edge_valid = true;
        for (int i = 0; i < GRANULARITY; i++) {
            edge_valid &= ppln::collision::fkcc<Robot>(config, env);
            for (int i = 0; i < dim; i++) {
                config[i] += delta[i];
            }
        }
        // if (edge_valid) {
        //     printf("edge valid for new_config %d and goal %d\n", new_config_idxs[config_idx], goal_idx);
        // }
        atomicCAS(&reached_goal, 0, edge_valid);
        atomicCAS(&reached_goal_idx, -1, goal_idx * edge_valid - (!edge_valid));
        atomicCAS(&goal_parent_idx, -1, new_config_idxs[config_idx] * edge_valid - (!edge_valid));
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
        int totalPairs = num_goals * NUM_NEW_CONFIGS;
        int threadsPerBlock1 = 256;
        int numBlocks1 = (totalPairs + threadsPerBlock1 - 1) / threadsPerBlock1;
        // std::cout << "numBlocks: " << numBlocks << "\n" << "BLOCK_SIZE: " << BLOCK_SIZE << "\n";
        // std::cout << "numBlocks1: " << numBlocks1 << "\n" << "threadsPerBlock1: " << threadsPerBlock1 << "\n";


        // main RRT loop
        while (iter++ < MAX_ITERS && free_index < MAX_SAMPLES) {
            std::cout << "iter: " << iter << std::endl;
            // sample configurations and get edges to be checked
            sample_edges<Robot><<<numBlocks, BLOCK_SIZE>>>(new_configs, new_config_parents, nodes, goal_configs, num_goals, rng_states, halton_states);
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();
            // collision check all the edges
            cudaMemset(cc_result, 0, NUM_NEW_CONFIGS * sizeof(unsigned int));
            // std::cout << "validate\n";
            validate_edges<Robot><<<NUM_NEW_CONFIGS, GRANULARITY>>>(new_configs, new_config_parents, cc_result, num_colliding_edges, env, nodes);
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();
            // add all the new edges to the tree
            // std::cout << "grow\n";
            grow_tree<Robot><<<numBlocks, BLOCK_SIZE>>>(new_configs, new_config_parents, cc_result, nodes, parents, num_colliding_edges, goal_configs, num_goals, new_config_idxs);
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();

            // check whether each new configuration added to the tree can reach the goal
            check_goal<Robot><<<numBlocks1, threadsPerBlock1>>>(new_configs, goal_configs, num_goals, env, nodes, parents, cc_result, new_config_idxs);
            cudaCheckError(cudaGetLastError());
            cudaDeviceSynchronize();
            // std::cout << "end loop\n";
            // update free index
            cudaMemcpyFromSymbol(&free_index, atomic_free_index, sizeof(int), 0, cudaMemcpyDeviceToHost);
            cudaCheckError(cudaGetLastError());
            cudaMemcpyFromSymbol(&done, reached_goal, sizeof(int), 0, cudaMemcpyDeviceToHost);
            // cudaCheckError(cudaGetLastError());
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
                std::cout << parent_idx << std::endl;
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



