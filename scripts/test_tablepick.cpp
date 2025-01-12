#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#include "src/collision/environment.hh"
#include "src/collision/factory.hh"
#include "src/planning/Planners.hh"

using json = nlohmann::json;

using namespace ppln::collision;

std::ifstream f("scripts/fetch_problems.json");

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

int main() {
    json all_data = json::parse(f);
    json problems = all_data["problems"];
    using Robot = robots::Fetch;
    using Configuration = Robot::Configuration;
    int failed = 0;
    std::map<std::string, std::vector<PlannerResult<Robot>>> results;
    // cage 13 - 14842 iterations for RRT w Halton on CPU
    // cage 100 - ~200,000 iterations for RRT w Halton on CPU
    // cage 70 - 342,092 iters on CPU
    std::string name = "table_pick";
    int problem_idx = 1;
    auto pset = problems[name];
    json data = pset[problem_idx - 1];
    if (not data["valid"]) {
        return -1;
    }
    auto env = problem_dict_to_env(data, name);
    printf("num spheres, capsules, cuboids: %d, %d, %d\n", env.num_spheres, env.num_capsules, env.num_cuboids);
    Configuration start = data["start"];
    std::vector<Configuration> goals = data["goals"];
    auto result = pRRT::solve<Robot>(start, goals, env);
    if (not result.solved) {
        failed ++;
        std::cout << "failed " << name << std::endl;
    }
    std::cout << "cost: " << result.cost << "\n";
    results[name].emplace_back(result);
}