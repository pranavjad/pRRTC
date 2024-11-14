#include "RRT_interleaved.hh"
#include "Robots.hh"
#include "collision/environment.hh"
#include "collision/shapes.hh"

using namespace ppln;


// void sphere_test() {
//     std::vector<float> obstacles = {
//          5.0,   5.0,   5.0,   1.0,
//          50.0,  50.0,  50.0,  0.1,
//         -10.0, -25.0, -40.0,  0.1
//     };

//     ppln::collision::Environment env;
//     env.spheres = obstacles.data();
//     env.num_spheres = (int)(obstacles.size() / 4);

//     robots::Sphere::Configuration start = {0.1, 0.2, 0.1};
//     std::vector<robots::Sphere::Configuration> goals = {{10.0, 10.0, 10.0}};

//     solve<robots::Sphere>(start, goals, env);
// }

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
    env.spheres = obstacles.data();
    env.num_spheres = (int)(obstacles.size() / 4);

    robots::Panda::Configuration start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    std::vector<robots::Panda::Configuration> goals = {{2.35, 1., 0., -0.8, 0, 2.5, 0.785}};

    solve<robots::Panda>(start, goals, env);
}

int main() {
    // sphere_test();
    panda_test();
}

