#include "Planners.hh"
#include "Robots.hh"
#include "utils.cuh"
#include "src/collision/environment.hh"

#include <curand.h>
#include <curand_kernel.h>

#include <vector>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <numeric>

/*
New attempt to rewrite RRTC from scratch.
I realize now that everything can be organized into one kernel with launch parameters <<<NUM_NEW_CONFIGS, GRANULARITY>>>
I am going to try this approach and see how it goes.
*/


namespace pRRTC {
    using namespace ppln;
    __device__ volatile int solved = 0;
    __device__ volatile int atomic_free_index[2]; // separate for tree_a and tree_b
    __device__ float path[2][500]; // solution path segments for tree_a, and tree_b
    __device__ int path_size[2] = {0, 0};
    __device__ float cost = 0.0;
    __device__ int reached_goal_idx = 0;

    constexpr int MAX_SAMPLES = 1000000;
    constexpr int MAX_ITERS = 1000000;
    constexpr int NUM_NEW_CONFIGS = 400;
    constexpr int GRANULARITY = 256;
    constexpr float RRT_RADIUS = 1.0;
    constexpr float TREE_RATIO = 0.5;
    constexpr bool balance = true;

    // threads per block for sample_edges and grow_tree
    constexpr int BLOCK_SIZE = 256;

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
        if (idx >= NUM_NEW_CONFIGS) return;
        int skip = (curand_uniform(&cr_states[idx]) * 500000.0f);
        if (idx == 0) skip = 0;
        if (idx == 1) skip = 100000;
        halton_initialize(states[idx], skip);
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
        
        atomic_free_index[0] = 0;
        atomic_free_index[1] = 0;
        
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
    
    template <typename Robot>
    __global__ void rrtc(
        float **nodes,
        int **parents,
        HaltonState<Robot> *halton_states,
        curandState *rng_states,
        ppln::collision::Environment<float> *env
    )
    {
        
        // printf("rrtc\n");
        static constexpr auto dim = Robot::dimension;
        const int tid = threadIdx.x;
        const int bid = blockIdx.x; // 0 ... NUM_NEW_CONFIGS
        __shared__ int t_tree_id; // this tree
        __shared__ int o_tree_id; // the other tree
        __shared__ volatile float config[dim];
        __shared__ volatile float sdata[GRANULARITY];
        __shared__ volatile unsigned int sindex[GRANULARITY];
        __shared__ volatile unsigned int local_cc_result;
        __shared__ float *t_nodes;
        __shared__ float *o_nodes;
        __shared__ int *t_parents;
        __shared__ int *o_parents;
        __shared__ float scale;
        __shared__ volatile float *nearest_node;
        __shared__ volatile float delta[dim];
        __shared__ volatile float var_cache[GRANULARITY][10];
        __shared__ volatile int index;
        __shared__ volatile float vec[dim];
        __shared__ unsigned int n_extensions;

        // printf("here1\n");
        /* sample_edges */
        // if (tid < dim) {
        //     config[tid] = curand_uniform(&rng_states[bid * dim + tid]);
        // }
        // __syncthreads();

        int iter=0;

        while (true){

            
            if (tid == 0) {
                iter++;
                if (iter>MAX_ITERS) atomicCAS((int *)&solved, 0, -1);
                // t_tree_id = 0;

                if (!balance || iter==1){
                    t_tree_id = (bid < (NUM_NEW_CONFIGS / 2))? 0 : 1;
                    o_tree_id = 1 - t_tree_id;
                }
                else if (balance){
                    float ratio = atomic_free_index[0]/(float)(atomic_free_index[0]+atomic_free_index[1]);
                    
                    float balance_factor = 1-ratio;
                    t_tree_id = (bid < (NUM_NEW_CONFIGS * balance_factor))? 0 : 1;
                    o_tree_id = 1 - t_tree_id;
                    
                   /*
                    if (ratio<TREE_RATIO) t_tree_id=0;
                    else t_tree_id=1;
                    o_tree_id = 1 - t_tree_id;
                    */
                }


                // if (bid == 0) printf("t_tree_id: %d\n", t_tree_id);
                t_nodes = nodes[t_tree_id];
                o_nodes = nodes[o_tree_id];
                t_parents = parents[t_tree_id];
                o_parents = parents[o_tree_id];
                // printf("here2\n");
                
                halton_next(halton_states[bid], (float *)config);
                Robot::scale_cfg((float *)config);
                //printf("iter %d\n", iter);
                //printf("config x %f\n", config[0]);
                local_cc_result = 0;
            }
            __syncthreads();

            

            // if (tid == 0 && bid == 1) {
            //     printf("sample: ");
            //     print_config(config, dim);
            // }
            // __syncthreads();

            // divide up the work of finding nearest neighbor among the threads
            float local_min_dist = INFINITY;
            unsigned int local_near_idx = 0;
            float dist;
            for (unsigned int i = 0; i < atomic_free_index[t_tree_id]; i += blockDim.x) {
                dist = device_utils::sq_l2_dist(&t_nodes[i * dim], (float *) config, dim);
                if (dist < local_min_dist) {
                    local_min_dist = dist;
                    local_near_idx = i;
                }
            }
            sdata[tid] = local_min_dist;
            sindex[tid] = local_near_idx;
            __syncthreads();

            for (unsigned int s = blockDim.x/2; s > 0; s >>= 1) {
                
                float sdata_tid_s = sdata[tid + s];
                float sdata_tid = sdata[tid];
                
                __syncthreads();
                if (tid < s && (tid+s)<atomic_free_index[t_tree_id]){
                    if (sdata_tid_s < sdata_tid) {
                        sdata[tid] = sdata[tid + s];
                        sindex[tid] = sindex[tid + s];
                    }
                }
                    
                
                __syncthreads();
            }



            // by this point NN dist = sdata[0], NN index = sindex[0]
            // if (tid == 0 && bid == 0) {
            //     printf("NN dist, idx: %f, %d\n", sqrt(sdata[0]), sindex[0]);
            //     print_config(&t_nodes[sindex[0] * dim], dim);
            //     print_config(&nodes[1][0], dim);
            // }
            // __syncthreads();
            // now calculate the extension
            
            if (tid == 0) {
                scale = min(1.0f, RRT_RADIUS / sqrt(sdata[0]));
                nearest_node = &t_nodes[sindex[0] * dim];
            }
            __syncthreads();

            if (tid < dim) {
                config[tid] = nearest_node[tid] + ((config[tid] - nearest_node[tid]) * scale);
                delta[tid] = (config[tid] - nearest_node[tid]) / (float) GRANULARITY;
            }
            __syncthreads();

            //if (tid==0) printf("sample %f %f %f %f %f %f %f neighbor %f %f %f %f %f %f %f\n", config[0], config[1], config[2], config[3], config[4], config[5], config[6], nearest_node[0], nearest_node[1], nearest_node[2], nearest_node[3], nearest_node[4], nearest_node[5], nearest_node[6]);
            
            /* validate_edges */
            float interp_cfg[dim];
            for (int i = 0; i < dim; i++) {
                interp_cfg[i] = nearest_node[i] + ((tid + 1) * delta[i]);

            }
            
            
            bool config_in_collision = not ppln::collision::fkcc<Robot>(interp_cfg, env, var_cache, tid);
            // if (tid == 200 && bid == 1) {
            //     printf("device num spheres, capsules, cuboids: %d, %d, %d\n", env->num_spheres, env->num_capsules, env->num_cuboids);
            //     printf("iterp_cfg: ");
            //     print_config(interp_cfg, dim);
            //     printf("config_in_collision: %d\n", config_in_collision);
            // }
            // __syncthreads();
            atomicOr((unsigned int *)&local_cc_result, config_in_collision ? 1u : 0u);
            __syncthreads();
            // printf("here3\n");
            if (local_cc_result == 0) {
                // if (tid == 2 && bid == 0) printf("entered local_cc_result\n");
                // printf("entered local_cc_result %d %d\n", tid, bid);
                /* grow tree */
                
                if (tid == 0) {
                    index = atomicAdd((int *)&atomic_free_index[t_tree_id], 1);
                    if (index >= MAX_SAMPLES) solved=-1;
                    t_parents[index] = sindex[0];
                }
                __syncthreads();

                if (tid < dim) {
                    t_nodes[index * dim + tid] = config[tid];
                }
                __syncthreads();

                // if (tid == 0 && bid == 1) {
                //     printf("added to tree: ");
                //     print_config(config, dim);
                // }
                // __syncthreads();
                // printf("here4\n");
                /* connect */
                // find nearest neighbor in opposite tree
                local_min_dist = INFINITY;
                local_near_idx = 0;
                for (unsigned int i = 0; i < atomic_free_index[o_tree_id]; i += blockDim.x) {
                    dist = device_utils::sq_l2_dist(&o_nodes[i * dim], (float *)config, dim);
                    if (dist < local_min_dist) {
                        local_min_dist = dist;
                        local_near_idx = i;
                    }
                }
                sdata[tid] = local_min_dist;
                sindex[tid] = local_near_idx;
                __syncthreads();
                // printf("here5\n");
                for (unsigned int s = blockDim.x/2; s > 0; s >>= 1) {
                    if (tid < s) {
                        if (sdata[tid + s] < sdata[tid]) {
                            sdata[tid] = sdata[tid + s];
                            sindex[tid] = sindex[tid + s];
                        }
                    }
                    __syncthreads();
                }
                // if (tid == 0 && bid == 1) {
                // printf("NN in opposite tree dist, idx: %f, %d\n", sqrt(sdata[0]), sindex[0]);
                // printf("NN o tree: ");
                // print_config(&o_nodes[sindex[0] * dim], dim);
                // }
                __syncthreads();
                
                if (tid == 0) {
                    sdata[0] = sqrt(sdata[0]);
                    // scale = min(1.0f, RRT_RADIUS / sdata[0]);
                    nearest_node = &o_nodes[sindex[0] * dim];
                    n_extensions = ceil(sdata[0] / RRT_RADIUS);
                    local_cc_result = 0;
                }
                __syncthreads();

                if (tid < dim) {
                    // vec[tid] = (nearest_node[tid] - config[tid]) * scale;
                    vec[tid] = (nearest_node[tid] - config[tid]) / (float) n_extensions;
                }
                __syncthreads();


                // if (tid == 0 && bid == 1) {
                //     printf("vec:");
                //     print_config(vec, dim);
                // }
                // __syncthreads();

                // validate the edge to the nearest neighbor in opposite tree, go as far as we can
                int i_extensions = 0;
                int extension_parent_idx = index;
                // printf("here6\n");
                while (i_extensions < n_extensions) {
                    /* each thread checking an interpolated config along the extension vector*/
                    for (int i = 0; i < dim; i++) {
                        interp_cfg[i] = config[i] + ((tid + 1) * (vec[i] / GRANULARITY));
                    }
                    bool config_in_collision = not ppln::collision::fkcc<Robot>(interp_cfg, env, var_cache, tid);
                    atomicOr((unsigned int *)&local_cc_result, config_in_collision ? 1u : 0u);
                    __syncthreads();
                    if (local_cc_result != 0) break;
                    /* add extension to tree */
                    if (tid == 0) {
                        index = atomicAdd((int *)&atomic_free_index[t_tree_id], 1);
                        if (index >= MAX_SAMPLES) solved=-1;
                        t_parents[index] = extension_parent_idx;
                        extension_parent_idx = index;
                        local_cc_result = 0;
                    }
                    __syncthreads();
                    if (tid < dim) {
                        config[tid] = config[tid] + vec[tid];
                        t_nodes[index * dim + tid] = config[tid];
                    }
                    __syncthreads();
                    // if (tid == 0 && bid == 1) {
                    //     printf("added to tree: ");
                    //     print_config(config, dim);
                    // }
                    i_extensions++;
                    __syncthreads();
                }
                // if (tid == 0) {
                //     printf("n_extensions: %d\n", n_extensions);
                //     printf("i_extensions: %d\n", i_extensions);
                // }
                // printf("here7\n");
                if (i_extensions == n_extensions) { // connected
                    if (tid == 0 && atomicCAS((int *)&solved, 0, 1) == 0) {
                        //printf("entered here %d %d\n", tid, bid);
                        //printf("n_extensions: %d\n", n_extensions);
                        // trace back to the start and goal.
                        // int current = atomic_free_index[t_tree_id] - 1;
                        int current = index;
                        int parent;
                        int t_path_size = 0;
                        int o_path_size = 0;
                        while (t_parents[current] != current) {
                            // printf("entered here1\n");
                            // printf("path config: ");
                            // print_config(&t_nodes[current*dim], dim);
                            parent = t_parents[current];
                            cost += device_utils::l2_dist(&t_nodes[current * dim], &t_nodes[parent * dim], dim);
                            for (int i = 0; i < dim; i++) path[t_tree_id][t_path_size * dim + i] = t_nodes[current * dim + i];
                            
                            print_config(&t_nodes[current * dim], dim);
                            t_path_size++;
                            current = parent;
                            
                        }
                        printf("GPU path above");
                        if (t_tree_id == 1) reached_goal_idx = current;
                        current = sindex[0];
                        // printf("entered here2\n");
                        while(o_parents[current] != current) {
                            parent = o_parents[current];
                            cost += device_utils::l2_dist(&o_nodes[current * dim], &o_nodes[parent * dim], dim);
                            for (int i = 0; i < dim; i++) path[o_tree_id][o_path_size * dim + i] = o_nodes[current * dim + i];
                            print_config(&o_nodes[current * dim], dim);
                            o_path_size++;
                            current = parent;
                        }
                        printf("GPU path above 2");
                        if (t_tree_id == 0) reached_goal_idx = current;
                        path_size[t_tree_id] = t_path_size;
                        path_size[o_tree_id] = o_path_size;
                        //printf("path_size: {%d, %d}; cost: %f\n", path_size[0], path_size[1], cost);
                        // printf("entered here3\n");
                        return;
                    }
                    __syncthreads();
                }
                // printf("here8\n");
            }
            __syncthreads();
            if (solved!=0) return;
        }
        
        
    }




    template <typename Robot>
    PlannerResult<Robot> solve(typename Robot::Configuration &start, std::vector<typename Robot::Configuration> &goals, ppln::collision::Environment<float> &h_environment) {
        static constexpr auto dim = Robot::dimension;
        std::size_t iter = 0;
        std::size_t start_index = 0;

        
        PlannerResult<Robot> res;

        // copy data to GPU
        float *start_config;
        float *goal_configs;
        int num_goals = goals.size();
        float *nodes[2];
        int *parents[2];
        float **d_nodes;
        int **d_parents;
        cudaMalloc(&d_nodes, 2 * sizeof(float*));
        cudaMalloc(&d_parents, 2 * sizeof(int*));
        const std::size_t config_size = dim * sizeof(float);
        cudaMalloc(&start_config, config_size);
        cudaMalloc(&goal_configs, config_size * num_goals);
        for (int i = 0; i < 2; i++) {
            cudaMalloc(&nodes[i], MAX_SAMPLES * config_size);
            cudaMalloc(&parents[i], MAX_SAMPLES * sizeof(int));
        }
        cudaMemcpy(d_nodes, nodes, 2 * sizeof(float*), cudaMemcpyHostToDevice);
        cudaMemcpy(d_parents, parents, 2 * sizeof(int*), cudaMemcpyHostToDevice);

        cudaMemcpy(start_config, start.data(), config_size, cudaMemcpyHostToDevice);
        cudaMemcpy(goal_configs, goals.data(), config_size, cudaMemcpyHostToDevice);

        // add start to tree_a and goals to tree_b
        cudaMemcpy(nodes[0], start.data(), config_size, cudaMemcpyHostToDevice);
        cudaMemcpy(parents[0], &start_index, sizeof(int), cudaMemcpyHostToDevice);

        cudaMemcpy(nodes[1], goals.data(), config_size * num_goals, cudaMemcpyHostToDevice);
        std::vector<int> parents_b_init(num_goals);
        iota(parents_b_init.begin(), parents_b_init.end(), 0); // consecutive integers from 0 ... num_goals - 1
        cudaMemcpy(parents[1], parents_b_init.data(), sizeof(int) * num_goals, cudaMemcpyHostToDevice);

        // create a curandState for each thread -> holds state of RNG for each thread seperately
        // For growing the tree we will create NUM_NEW_CONFIGS threads
        curandState *rng_states;
        int num_rng_states = NUM_NEW_CONFIGS * dim;
        cudaMalloc(&rng_states, num_rng_states * sizeof(curandState));
        int numBlocks = (num_rng_states + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_rng<<<numBlocks, BLOCK_SIZE>>>(rng_states, 1, num_rng_states);

        HaltonState<Robot> *halton_states;
        cudaMalloc(&halton_states, NUM_NEW_CONFIGS * sizeof(HaltonState<Robot>));
        int numBlocks1 = (NUM_NEW_CONFIGS + BLOCK_SIZE - 1) / BLOCK_SIZE;
        init_halton<Robot><<<numBlocks1, BLOCK_SIZE>>>(halton_states, rng_states);

        // free index for next available position in tree_a and tree_b
        int h_free_index[2] = {1, num_goals};
        cudaMemcpyToSymbol(atomic_free_index, &h_free_index, sizeof(int) * 2);

        // allocate for obstacles
        ppln::collision::Environment<float> *env;
        setup_environment_on_device(env, h_environment);

        cudaCheckError(cudaGetLastError());
        // Setup pinned memory for signaling
        int *h_solved;
        int current_samples[2];
        cudaMallocHost(&h_solved, sizeof(int));  // Pinned memory
        *h_solved = 0;
        
        auto start_time = std::chrono::steady_clock::now();
        
        // std::cout << "iter: " << iter << "\n";
        rrtc<Robot><<<NUM_NEW_CONFIGS, GRANULARITY>>> (
            d_nodes,
            d_parents,
            halton_states,
            rng_states,
            env
        );
        cudaDeviceSynchronize();
        // cudaCheckError(cudaGetLastError());

        res.nanoseconds = get_elapsed_nanoseconds(start_time);

        // int current_samples[2];
        cudaMemcpyFromSymbol(current_samples, atomic_free_index, sizeof(int) * 2, 0, cudaMemcpyDeviceToHost);
        

        cudaMemcpyFromSymbol(h_solved, solved, sizeof(int), 0, cudaMemcpyDeviceToHost);

        // currently, iteration count is not copied because each block may have different iteration count

        if (*h_solved!=1) *h_solved=0;
        
        std::cout << "current_samples: start: " << current_samples[0] << ", goal: " << current_samples[1] << "\n";
        // printf("current_samples: %d, %d\n", current_samples[0], current_samples[1]);
        std::cout << "iters: " << iter << "\n";
        std::cout << "exited loop\n";
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
            Robot::print_robot_config(goals[h_reached_goal_idx]);
            for (int i = h_path_size[1] - 1; i >= 0; i--) print_cfg<Robot>(&h_paths[1][i * dim]);
            for (int i = 0; i < h_path_size[0]; i++) print_cfg<Robot>(&h_paths[0][i * dim]);
            Robot::print_robot_config(start);
            res.cost = h_cost;
            std::cout << "cost: " << res.cost << "\n";
        }
        // cudaCheckError(cudaGetLastError());
        res.solved = (*h_solved) != 0;
        res.iters = iter;
        
        cleanup_environment_on_device(env, h_environment);
        reset_device_variables();
        // cudaCheckError(cudaGetLastError());
        cudaFree(start_config);
        cudaFree(goal_configs);
        // cudaCheckError(cudaGetLastError());
        cudaFree(nodes[0]);
        cudaFree(nodes[1]);
        // cudaCheckError(cudaGetLastError());
        cudaFree(parents[0]);
        cudaFree(parents[1]);
        // cudaCheckError(cudaGetLastError());
        cudaFree(rng_states);
        cudaFree(halton_states);
        cudaFree(d_nodes);
        cudaFree(d_parents);
        // cudaFree(h_solved);
        cudaCheckError(cudaGetLastError());
        return res;
    }

    template PlannerResult<typename ppln::robots::Sphere> solve<ppln::robots::Sphere>(std::array<float, 3>&, std::vector<std::array<float, 3>>&, ppln::collision::Environment<float>&);
    template PlannerResult<typename ppln::robots::Panda> solve<ppln::robots::Panda>(std::array<float, 7>&, std::vector<std::array<float, 7>>&, ppln::collision::Environment<float>&);
    template PlannerResult<typename ppln::robots::Fetch> solve<ppln::robots::Fetch>(std::array<float, 8>&, std::vector<std::array<float, 8>>&, ppln::collision::Environment<float>&);
}


