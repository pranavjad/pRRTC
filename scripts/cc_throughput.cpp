#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#include "src/collision/environment.hh"
#include "src/collision/factory.hh"
#include "src/planning/Planners.hh"
#include "src/planning/pRRTC_settings.hh"

#include "src/planning/cc_throughput.hh"

using json = nlohmann::json;

using namespace ppln::collision;


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

int main(int argc, char* argv[]) {
    std::string robot_name = "panda";
    std::string name = "cage";
    int granularity = 128;
    int problem_idx = 1;
    int edges_per_block = 1;
    int num_blocks = 1;
    if (argc == 7) {
        robot_name = argv[1];
        name = argv[2];
        problem_idx = std::stoi(argv[3]);
        num_blocks = std::stoi(argv[4]);
        granularity = std::stoi(argv[5]);
        edges_per_block = std::stoi(argv[6]);
    }
    else {
        std::cout << "Usage: ./cc_throughput <robot_name> <problem_name> <problem_idx> <num blocks> <granularity> <edges per block>\n";
        return 1;
    }

    std::string path = "scripts/" + robot_name + "_problems.json";
    std::ifstream f(path);
    json all_data = json::parse(f);
    json problems = all_data["problems"];
    auto pset = problems[name];
    json data = pset[problem_idx - 1];
    if (not data["valid"]) {
        return -1;
    }
    auto env = problem_dict_to_env(data, name);
    size_t nanoseconds = -1;
    if (robot_name == "panda") {
        nanoseconds = bench_cc::test_cc_throughput<robots::Panda>(env, num_blocks, granularity, edges_per_block);
    }
    else if (robot_name == "fetch") {
        nanoseconds = bench_cc::test_cc_throughput<robots::Fetch>(env, num_blocks, granularity, edges_per_block);
    }
    else {
        std::cout << "Unknown robot name: " << robot_name << "\n";
        return -1;
    }
    std::cout << "Time taken: " << nanoseconds << " ns\n";
    std::cout << "Edges Checked: " << num_blocks * edges_per_block << "\n";
    double throughput = (num_blocks * edges_per_block) / (nanoseconds / 1e9);
    std::cout << "Throughput: " << throughput << " edges/s\n";
    return 0;
}