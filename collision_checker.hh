#pragma once
#include "kernel_types.hh"
#include <vector>

class gpu_collision_checker {
    private:
        std::vector<kernel_types::Sphere> &obstacles;
        kernel_types::Sphere *device_obstacles;
        bool *device_out;
        kernel_types::CollisionKernelArgs *device_args;

    public:
        gpu_collision_checker(std::vector<kernel_types::Sphere> &obstacles, float robot_radius);
        ~gpu_collision_checker();

        bool fkcc_gpu(const std::vector<std::vector<float>> &configs);

        bool validate_motion(double x1, double y1, double z1, double x2, double y2, double z2, unsigned long resolution);
};