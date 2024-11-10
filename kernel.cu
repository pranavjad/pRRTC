#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "kernel_types.hh"
#include "collision_checker.hh"
// #include <cmath>
#include <new>
#include <iostream>

using namespace kernel_types;

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

__device__ inline constexpr auto sphere_sphere_sql2(
    const Sphere &a,
    const float bx,
    const float by,
    const float bz,
    const float br) -> float
{
    return sphere_sphere_sql2(a.x, a.y, a.z, a.r, bx, by, bz, br);
}


__device__ inline constexpr auto sphere_sphere_self_collision(
    float ax_,
    float ay_,
    float az_,
    float ar_,
    float bx_,
    float by_,
    float bz_,
    float br_) -> bool
{
    return sphere_sphere_sql2(ax_, ay_, az_, ar_, bx_, by_, bz_, br_) < 0;
}

__device__ inline bool sphere_environment_in_collision(
    const Sphere *spheres,
    const CollisionKernelArgs *args,
    float sx_,
    float sy_,
    float sz_,
    float sr_)
{
    const float max_extent = std::sqrt(dot3(sx_, sy_, sz_, sx_, sy_, sz_)) + sr_;
    for (unsigned int i = 0; i < args->num_spheres_in_environment; i++)
    {
        if (spheres[i].min_distance > max_extent)
        {
            break;
        }

        if (sphere_sphere_sql2(spheres[i], sx_, sy_, sz_, sr_) < 0)
        {
            return true;
        }
    }
    return false;
}

__device__ bool gpu_cc_internal(
    const Sphere *spheres,
    const float *configurations,
    const CollisionKernelArgs *args,
    unsigned int id)
{
    auto x = configurations[id * 3];
    auto y = configurations[id * 3 + 1];
    auto z = configurations[id * 3 + 2];
    sphere_environment_in_collision(spheres, args, x, y, z, args->robot_radius);
}


__global__ void gpu_cc(
    const Sphere *spheres,
    const float* configurations,
    const CollisionKernelArgs *args,
    bool *out
)
{
    unsigned int id = blockIdx.x * blockDim.x + threadIdx.x;
    *out = *out | gpu_cc_internal(spheres, configurations, args, id);
}

gpu_collision_checker::gpu_collision_checker(std::vector<Sphere> &obstacles, float robot_radius) : obstacles(obstacles)
{
    size_t obstacles_size = sizeof(Sphere) * obstacles.size();
    size_t out_size = sizeof(bool);
    size_t args_size = sizeof(CollisionKernelArgs);
    cudaMalloc(&device_obstacles, obstacles_size);
    cudaMalloc(&device_out, out_size);
    cudaMalloc(&device_args, args_size);
    cudaMemcpy(device_obstacles, obstacles.data(), obstacles_size, cudaMemcpyHostToDevice);
    kernel_types::CollisionKernelArgs host_args = {obstacles.size(), robot_radius};
    cudaMemcpy(device_args, &host_args, sizeof(host_args), cudaMemcpyHostToDevice);
    bool host_out = false;
    cudaMemcpy(device_out, &host_out, sizeof(host_out), cudaMemcpyHostToDevice);
}

// returns true if all of the configs have no collisions, returns false if at least one of the configs has a collision
bool gpu_collision_checker::fkcc_gpu(const std::vector<std::vector<float>> &configs)
{
    auto num_cfgs = configs.size();

    // copy configurations from host to device
    float *device_configurations, *host_configurations;

    size_t configs_size = sizeof(float) * num_cfgs * 3;
    cudaMallocHost(&host_configurations, configs_size);
    cudaMalloc(&device_configurations, configs_size);

    for (auto i = 0U; i < num_cfgs; i++) {
        host_configurations[i * 3] = configs[i][0];
        host_configurations[i * 3 + 1] = configs[i][1];
        host_configurations[i * 3 + 2] = configs[i][2];
    }

    cudaMemcpy(device_configurations, host_configurations, configs_size, cudaMemcpyHostToDevice);
    cudaFreeHost(host_configurations);

    // run kernel
    int block_size = 1024;
    if (num_cfgs < block_size) {
        block_size = num_cfgs;
    }
    int num_blocks = (num_cfgs + block_size - 1) / block_size;
    bool host_out = false;
    cudaMemcpy(device_out, &host_out, sizeof(host_out), cudaMemcpyHostToDevice);
    gpu_cc <<< num_blocks, block_size >>> (device_obstacles, device_configurations, device_args, device_out);
    cudaDeviceSynchronize();
    cudaFree(device_configurations);
    cudaMemcpy(&host_out, device_out, sizeof(host_out), cudaMemcpyDeviceToHost);
    return host_out;
}

bool gpu_collision_checker::validate_motion(double x1, double y1, double z1, double x2, double y2, double z2, unsigned long resolution)
{
    double dx = (x2 - x1) / resolution;
    double dy = (y2 - y1) / resolution;
    double dz = (z2 - z1) / resolution;
    std::vector<std::vector<float>> configs(resolution, std::vector<float>(3));

    for (int i = 0; i < resolution; i++) {
        configs[i][0] = x1;
        configs[i][1] = y1;
        configs[i][2] = z1;
        x1 += dx;
        y1 += dy;
        z1 += dz;
    }
    return fkcc_gpu(configs);
}

gpu_collision_checker::~gpu_collision_checker()
{
    cudaFree(device_obstacles);
    cudaFree(device_args);
    cudaFree(device_out);
}