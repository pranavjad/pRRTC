#include "RRT_interleaved.hh"
#include "Robots.hh"
#include "collision/environment.hh"
#include "collision/shapes.hh"

using namespace ppln;


void sphere_test() {
    std::vector<collision::Sphere<float>> obstacles = {
        collision::Sphere<float>{0.0,  0.0,  0.0,  3.0},
    };

    ppln::collision::Environment<float> env;
    env.spheres = new collision::Sphere<float>[obstacles.size()];
    std::copy(obstacles.begin(), obstacles.end(), env.spheres);
    env.num_spheres = (int)(obstacles.size());
    env.capsules = nullptr;
    env.num_capsules = 0;
    env.cuboids = nullptr;
    env.num_cuboids = 0;
    env.z_aligned_capsules = nullptr;
    env.num_z_aligned_capsules = 0;
    env.cylinders = nullptr;
    env.num_cylinders = 0;
    env.z_aligned_cuboids = nullptr;
    env.num_z_aligned_cuboids = 0;

    robots::Sphere::Configuration start = {-4.0, -4.0, -4.0};
    std::vector<robots::Sphere::Configuration> goals = {{4.0, 4.0, 4.0}};

    auto res = pRRT::solve<robots::Sphere>(start, goals, env);
}

void panda_test() {
    // obstacles for sphere cage
    std::vector<collision::Sphere<float>> obstacles = {
        collision::Sphere<float>{0.55,  0.00,  0.25,  0.2},
        collision::Sphere<float>{0.35,  0.35,  0.25,  0.2},
        collision::Sphere<float>{0.00,  0.55,  0.25,  0.2},
        collision::Sphere<float>{-0.55,  0.00,  0.25,  0.2},
        collision::Sphere<float>{-0.35, -0.35,  0.25,  0.2},
        collision::Sphere<float>{0.00, -0.55,  0.25,  0.2},
        collision::Sphere<float>{0.35, -0.35,  0.25,  0.2},
        collision::Sphere<float>{0.35,  0.35,  0.80,  0.2},
        collision::Sphere<float>{0.00,  0.55,  0.80,  0.2},
        collision::Sphere<float>{-0.35,  0.35,  0.80,  0.2},
        collision::Sphere<float>{-0.55,  0.00,  0.80,  0.2},
        collision::Sphere<float>{-0.35, -0.35,  0.80,  0.2},
        collision::Sphere<float>{0.00, -0.55,  0.80,  0.2},
        collision::Sphere<float>{0.35, -0.35,  0.80,  0.2}
    };

    ppln::collision::Environment<float> env;
    env.spheres = new collision::Sphere<float>[obstacles.size()];
    std::copy(obstacles.begin(), obstacles.end(), env.spheres);
    env.num_spheres = (int)(obstacles.size());
    env.capsules = nullptr;
    env.num_capsules = 0;
    env.z_aligned_capsules = nullptr;
    env.num_z_aligned_capsules = 0;
    env.cylinders = nullptr;
    env.num_cylinders = 0;
    env.cuboids = nullptr;
    env.num_cuboids = 0;
    env.z_aligned_cuboids = nullptr;
    env.num_z_aligned_cuboids = 0;

    robots::Panda::Configuration start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    std::vector<robots::Panda::Configuration> goals = {{2.35, 1., 0., -0.8, 0, 2.5, 0.785}};

    auto res = RRT::solve<robots::Panda>(start, goals, env);
}

int main() {
    sphere_test();
    // panda_test();
}

