#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <nlohmann/json.hpp>

#include "src/collision/environment.hh"
#include "src/collision/factory.hh"
#include "src/planning/Planners.hh"
#include "src/planning/pRRTC_settings.hh"
#include "../src/planning/utils.cuh"


using json = nlohmann::json;
using namespace ppln::collision;


// Script to check a single edge for a robot using fkcc.

template <typename Robot>
__global__ void check_edge(ppln::collision::Environment<float> *env, float *config1, float *config2, bool* result, int granularity) {
    constexpr auto dim = Robot::dimension;
    const int tid = threadIdx.x;
    __shared__ volatile unsigned int local_cc_result[1];
    __shared__ volatile float delta[dim];
    // check with threads in parallel
    if (tid == 0) {
        local_cc_result[0] = 0;
    }
    __syncthreads();
    if (tid < dim) {
        delta[tid] = (config2[tid] - config1[tid]) / (float) granularity;
    }
    __syncthreads();
    float interp_cfg[dim];
    for (int j = 0; j < dim; j++) {
        interp_cfg[j] = config1[j] +  (delta[j] * (tid + 1));
    }
    bool config_in_collision = not ppln::collision::fkcc<Robot>(interp_cfg, env, tid);
    atomicOr((unsigned int *)&local_cc_result[0], config_in_collision ? 1u : 0u);
    if (config_in_collision) {
        printf("Collision detected at %d\n", tid);
    }
    __syncthreads();
    if (tid == 0) {
        *result = (local_cc_result[0] == 0);
        return;
    }
    // if (not ppln::collision::fkcc<Robot>(config, env, tid)) {
    //     printf("Collision detected at %d\n", i);
    //     *result = false;
    //     return;
    // }

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


Environment<float> problem_dict_to_env(const json& problem, const std::string& name) {
    Environment<float> env{};
    
    std::vector<Sphere<float>> spheres;
    std::vector<Capsule<float>> capsules;
    std::vector<Cuboid<float>> cuboids;
    // Fill spheres
    for (const auto& obj : problem["sphere"]) {
        const json& position = obj["position"];
        Sphere<float> sphere(position[0], position[1], position[2], obj["radius"]);
        sphere.name = obj["name"];
        spheres.push_back(sphere);
    }
    // Handle cylinders based on name
    if (name == "box") {
        for (const auto& obj : problem["cylinder"]) {
            const json& position = obj["position"];
            const json& orientation = obj["orientation_euler_xyz"];
            const float radius = obj["radius"];
            const std::array<float, 3> dims = {radius, radius, radius/2.0f};
            auto cuboid = factory::cuboid::array(
                position, orientation,
                dims
            );
            cuboid.name = obj["name"];
            cuboids.push_back(cuboid);
        }
    } else {
        for (const auto& obj : problem["cylinder"]) {
            const json& position = obj["position"];
            const json& orientation = obj["orientation_euler_xyz"];
            const float radius = obj["radius"];
            const float length = obj["length"];
            auto cylinder = factory::cylinder::center::array(
                position, orientation,
                radius, length
            );
            cylinder.name = obj["name"];
            capsules.push_back(cylinder);
        }
    }

    // Fill boxes
    for (const auto& obj : problem["box"]) {
        const json& position = obj["position"];
        const json& orientation = obj["orientation_euler_xyz"];
        const json& half_extents = obj["half_extents"];
        auto cuboid = factory::cuboid::array(
            position, orientation, half_extents
        );
        cuboid.name = obj["name"];
        cuboids.push_back(cuboid);
    }

    // Allocate memory on the heap for the arrays
    if (!spheres.empty()) {
        env.spheres = new Sphere<float>[spheres.size()];
        std::copy(spheres.begin(), spheres.end(), env.spheres);
        env.num_spheres = spheres.size();
    }

    if (!capsules.empty()) {
        env.capsules = new Capsule<float>[capsules.size()];
        std::copy(capsules.begin(), capsules.end(), env.capsules);
        env.num_capsules = capsules.size();
    }

    if (!cuboids.empty()) {
        env.cuboids = new Cuboid<float>[cuboids.size()];
        std::copy(cuboids.begin(), cuboids.end(), env.cuboids);
        env.num_cuboids = cuboids.size();
    }

    return env;
}

std::vector<float> parse_float_string(const std::string& str) {
    std::vector<float> result;
    std::stringstream ss(str);
    float value;
    
    while (ss >> value) {
        result.push_back(value);
    }
    
    return result;
}

int main(int argc, char* argv[]) {
    std::string robot_name = "panda";
    std::string name = "cage";
    int granularity = 128;
    int problem_idx = 1;
    if (argc == 5) {
        robot_name = argv[1];
        name = argv[2];
        problem_idx = std::stoi(argv[3]);
        granularity = std::stoi(argv[4]);
    }
    else {
        std::cout << "Usage: ./edge_check <robot_name> <problem_name> <problem_idx> <granularity>\n";
        return 1;
    }
    std::string path = "scripts/" + robot_name + "_problems.json";
    std::ifstream f(path);
    json all_data = json::parse(f);
    json problems = all_data["problems"];
    auto pset = problems[name];
    json data = pset[problem_idx - 1];
    if (not data["valid"]) {
        return -1;
    }
    auto env = problem_dict_to_env(data, name);
    std::string config1;
    std::string config2;
    std::getline(std::cin, config1);
    std::getline(std::cin, config2);
    auto vec1 = parse_float_string(config1);
    auto vec2 = parse_float_string(config2);

    float *config1_ptr;
    float *config2_ptr;
    bool *result;
    ppln::collision::Environment<float> *d_env;
    setup_environment_on_device(d_env, env);
    cudaMalloc(&config1_ptr, vec1.size() * sizeof(float));
    cudaMalloc(&config2_ptr, vec2.size() * sizeof(float));
    cudaMalloc(&result, sizeof(bool));
    cudaMemcpy(config1_ptr, vec1.data(), vec1.size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(config2_ptr, vec2.data(), vec2.size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemset(result, 1, sizeof(bool));
    if (robot_name == "panda") {
        check_edge<robots::Panda><<<1,granularity>>>(d_env, config1_ptr, config2_ptr, result, granularity);
    }
    else if (robot_name == "fetch") {
        check_edge<robots::Fetch><<<1,granularity>>>(d_env, config1_ptr, config2_ptr, result, granularity);
    }
    cudaDeviceSynchronize();
    bool host_result = true;
    cudaMemcpy(&host_result, result, sizeof(bool), cudaMemcpyDeviceToHost);
    std::cout << "Edge collision free: " << host_result << std::endl;
    cleanup_environment_on_device(d_env, env);
    cudaFree(config1_ptr);
    cudaFree(config2_ptr);
    cudaFree(result);
    return 0;
}