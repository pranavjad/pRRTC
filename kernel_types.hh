#pragma once
#include <cmath>



namespace kernel_types
{
    struct CollisionKernelArgs {
        unsigned int num_spheres_in_environment;
        float robot_radius;
    };

    struct Sphere
    {
        float x;
        float y;
        float z;
        float r;
        float min_distance;

        Sphere() = default;

        constexpr explicit Sphere(float x_, float y_, float z_, float r_) : x(x_), y(y_), z(z_), r(r_), min_distance(std::sqrt(x_ * x_ + y_ * y_ + z_ * z_) - r) {}
    };
}  // namespace kernel_types