#include "RRT_interleaved.hh"
#include "Robots.hh"

using namespace ppln;

int main() {
    std::vector<float> obstacles = {
        5.0, 5.0, 5.0, 1.0,
        50.0, 50.0, 50.0, 0.1,
        -10.0, -25.0, -40.0, 0.1
    };

    robots::Sphere::Configuration start = {0.1, 0.2, 0.1};
    robots::Sphere::Configuration goal = {10.0, 10.0, 10.0};

    solve<robots::Sphere>(start, goal, obstacles);
}

