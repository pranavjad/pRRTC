#include "collision_checker.hh"
#include "kernel_types.hh"
#include <random>
#include <vector>
#include <iostream>



int main() {
    std::mt19937 gen(0);
    std::uniform_real_distribution<float> center_dist(0.0, 1.0);
    std::uniform_real_distribution<float> radius_dist(0.0, 0.05);

    // generate obstacles
    std::vector<kernel_types::Sphere> obstacles(20);
    for (auto& sphere : obstacles) {
        sphere = kernel_types::Sphere(
            center_dist(gen),
            center_dist(gen),
            center_dist(gen),
            radius_dist(gen)
        );
    }

    // generate edges (num_edges x 6)
    std::vector<std::vector<float>> edges(100);
    for (auto& edge : edges) {
        float start_x = center_dist(gen);
        float start_y = center_dist(gen);
        float start_z = center_dist(gen);
        float end_x = center_dist(gen);
        float end_y = center_dist(gen);
        float end_z = center_dist(gen);
        edge = {start_x, start_y, start_z, end_x, end_y, end_z};
    }

    gpu_collision_checker cc(obstacles, 1.0);

    for (auto& edge : edges) {
        std::cout << cc.validate_motion(edge[0], edge[1], edge[2], edge[3], edge[4], edge[5], 100) << std::endl;
    }
}