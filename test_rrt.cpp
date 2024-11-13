#include "RRT_interleaved.hh"
#include "Robots.hh"

using namespace ppln;


void sphere_test() {
    std::vector<float> obstacles = {
         5.0,   5.0,   5.0,   1.0,
         50.0,  50.0,  50.0,  0.1,
        -10.0, -25.0, -40.0,  0.1
    };

    robots::Sphere::Configuration start = {0.1, 0.2, 0.1};
    robots::Sphere::Configuration goal = {10.0, 10.0, 10.0};

    solve<robots::Sphere>(start, goal, obstacles);
}

void panda_test() {
    // obstacles for sphere cage
    std::vector<float> obstacles = {
        0.55,  0.00,  0.25,  0.2,
        0.35,  0.35,  0.25,  0.2,
        0.00,  0.55,  0.25,  0.2,
       -0.55,  0.00,  0.25,  0.2,
       -0.35, -0.35,  0.25,  0.2,
        0.00, -0.55,  0.25,  0.2,
        0.35, -0.35,  0.25,  0.2,
        0.35,  0.35,  0.80,  0.2,
        0.00,  0.55,  0.80,  0.2,
       -0.35,  0.35,  0.80,  0.2,
       -0.55,  0.00,  0.80,  0.2,
       -0.35, -0.35,  0.80,  0.2,
        0.00, -0.55,  0.80,  0.2,
        0.35, -0.35,  0.80,  0.2
    };

    robots::Panda::Configuration start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    robots::Panda::Configuration goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};

    solve<robots::Panda>(start, goal, obstacles);
}

int main() {
    // sphere_test();
    panda_test();
}

