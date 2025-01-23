#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#include "src/collision/environment.hh"
#include "src/collision/factory.hh"
#include "src/planning/Planners.hh"


using json = nlohmann::json;

using namespace ppln::collision;

std::ifstream f("scripts/panda_problems.json");

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
    using Configuration = robots::Panda::Configuration;
    int failed = 0;
    std::map<std::string, std::vector<PlannerResult<robots::Panda>>> results;
    std::vector<std::string> prob_names = {
        "bookshelf_small",
        "bookshelf_tall",
        "bookshelf_thin",
        "box",
        "cage",
        "table_pick",
        "table_under_pick",
    };
    for (auto& name : prob_names) {
        std::cout << name << "\n";
        auto pset = problems[name];
        for (int i = 0; i < pset.size(); i++) {
            std::cout << "idx: " << i << "\n";
            json data = pset[i];
            if (not data["valid"]) {
                continue;
            }
            auto env = problem_dict_to_env(data, name);
            printf("num spheres, capsules, cuboids: %d, %d, %d\n", env.num_spheres, env.num_capsules, env.num_cuboids);
            Configuration start = data["start"];
            std::vector<Configuration> goals = data["goals"];
            auto result = pRRTC::solve<robots::Panda>(start, goals, env);
            if (not result.solved) {
                failed ++;
                std::cout << "failed " << name << std::endl;
            }
            std::cout << "cost: " << result.cost << "\n";
            results[name].emplace_back(result);
        }
    }

    for (auto& name : prob_names) {
        std::cout << name << std::endl;

        // calculate stats for cost 
        float avg_cost = 0;
        float min_cost = 1e9;
        float max_cost = 0;
        for (auto &result : results[name]) {
            avg_cost += result.cost;
            min_cost = std::min(min_cost, result.cost);
            max_cost = std::max(max_cost, result.cost);
        }
        avg_cost /= results[name].size();
        std::cout << "avg cost: " << avg_cost << std::endl;
        std::cout << "min cost: " << min_cost << std::endl;
        std::cout << "max cost: " << max_cost << std::endl;
        std::cout << std::endl;

        // calculate stats for cost 
        std::size_t net_time = 0;
        std::size_t min_time = 1e9;
        std::size_t max_time = 0;
        for (auto &result : results[name]) {
            net_time += result.nanoseconds;
            min_time = std::min(min_time, result.nanoseconds);
            max_time = std::max(max_time, result.nanoseconds);
        }
        float avg_time = net_time / results[name].size();
        std::cout << "avg time (μs): " << avg_time/1000 << std::endl;
        std::cout << "min time (μs): " << min_time/1000 << std::endl;
        std::cout << "max time (μs): " << max_time/1000 << std::endl;

        // calculate stats for time per iter 
        float avg_time_per_iter = 0.0;
        float avg_time_per_attempted_iter = 0.0;
        for (auto &result : results[name]) {
            float time_per_attempted_iter = (float)result.nanoseconds / result.attempted_tree_size;
            float time_per_iter = (float)result.nanoseconds / result.tree_size;
            avg_time_per_iter += time_per_iter;
            avg_time_per_attempted_iter += time_per_attempted_iter;
        }
        avg_time_per_iter = avg_time_per_iter / results[name].size();
        avg_time_per_attempted_iter = avg_time_per_attempted_iter / results[name].size();
        std::cout << "avg time per node added to tree (μs): " << avg_time_per_iter/1000 << std::endl;
        std::cout << "avg time per attempted addition of node (μs): " << avg_time_per_attempted_iter/1000 << std::endl;
        std::cout << "----" << std::endl;
    }
}