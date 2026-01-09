#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#include "src/collision/environment.hh"
#include "src/collision/factory.hh"
#include "src/planning/Planners.hh"
#include "src/planning/pRRTC_settings.hh"

#include <vamp/planning/validate.hh>
#include <vamp/collision/factory.hh>
#include <vamp/collision/environment.hh>
#include <vamp/robots/fetch.hh> 
#include <vamp/robots/panda.hh>
#include <vamp/robots/baxter.hh>



using json = nlohmann::json;

using namespace ppln::collision;

static constexpr const std::size_t rake = vamp::FloatVectorWidth;

using VampEnv = vamp::collision::Environment<vamp::FloatVector<rake>>; 

VampEnv problem_dict_vamp(const json& problem, const std::string &name) {
    auto env = vamp::collision::Environment<float>();
    
    // Handle spheres
    for (const auto& obj : problem["sphere"]) {
        const json& position = obj["position"];
        auto sphere = vamp::collision::factory::sphere::array(
            {position[0], position[1], position[2]},
            obj["radius"]
        );
        env.spheres.emplace_back(sphere);
    }

    // Handle cylinders
    if (name == "box") {
        for (const auto& obj : problem["cylinder"]) {
            const json& position = obj["position"];
            const json& orientation = obj["orientation_euler_xyz"];
            const float radius = obj["radius"];
            const std::array<float, 3> dims = {radius, radius, radius/2.0f};
            auto cuboid = vamp::collision::factory::cuboid::array(
                {position[0], position[1], position[2]},
                {orientation[0], orientation[1], orientation[2]},
                dims
            );
            env.cuboids.emplace_back(cuboid);
        }
    } else {
        for (const auto& obj : problem["cylinder"]) {
            const json& position = obj["position"];
            const json& orientation = obj["orientation_euler_xyz"];
            const float radius = obj["radius"];
            const float length = obj["length"];
            auto capsule = vamp::collision::factory::capsule::center::array(
                {position[0], position[1], position[2]},
                {orientation[0], orientation[1], orientation[2]},
                radius, length
            );
            env.capsules.emplace_back(capsule);
        }
    }

    // Handle boxes
    for (const auto& obj : problem["box"]) {
        const json& position = obj["position"];
        const json& orientation = obj["orientation_euler_xyz"];
        const json& half_extents = obj["half_extents"];
        auto cuboid = vamp::collision::factory::cuboid::array(
            {position[0], position[1], position[2]},
            {orientation[0], orientation[1], orientation[2]},
            {half_extents[0], half_extents[1], half_extents[2]}
        );
        env.cuboids.emplace_back(cuboid);
    }
    env.sort();
    auto env_v = vamp::collision::Environment<vamp::FloatVector<rake>>(env);
    return env_v;
}

Environment<float> problem_dict_to_env(const json& problem, const std::string& name, bool use_pointcloud) {
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

    if (!problem["filtered_pointcloud"].empty() && use_pointcloud) {
        const auto& pointcloud = problem["filtered_pointcloud"];
        const auto num_points = pointcloud.size();
        env.pointcloud = new float[num_points * 3];
        env.num_points = static_cast<unsigned int>(num_points);
        for (size_t i = 0; i < num_points; ++i) {
            const auto& point = pointcloud[i];
            env.pointcloud[i * 3 + 0] = point[0].get<float>();
            env.pointcloud[i * 3 + 1] = point[1].get<float>();
            env.pointcloud[i * 3 + 2] = point[2].get<float>();
        }
    } else {
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
    }
    return env;
}


template <typename Robot, typename vampRobot>
void run_planner(json &data, Environment<float> &env, VampEnv &vamp_env, struct pRRTC_settings &settings) {
    using Configuration = typename Robot::Configuration;
    Configuration start = data["start"];
    std::vector<Configuration> goals = data["goals"];
    auto result = pRRTC::solve<Robot>(start, goals, env, settings);
    for (auto& cfg: result.path) {
        print_cfg<Robot>(cfg);
    }
    if (not result.solved) {
        std::cout << "failed!" << std::endl;
    }
    std::cout << "cost: " << result.cost << "\n";
    std::cout << "time (us): " << result.kernel_ns/1000.0f << "\n";

    // Validate the result
    bool vamp_valid = true;
    for (auto i = 1ul; i < result.path.size(); i++) {
        auto cfg1 = result.path[i-1];
        auto cfg2 = result.path[i];
        typename vampRobot::Configuration vamp_cfg1(cfg1);
        typename vampRobot::Configuration vamp_cfg2(cfg2);
        if (not vamp::planning::validate_motion<vampRobot, rake, 1>(vamp_cfg1, vamp_cfg2, vamp_env)) {
            int index1 = result.path.size() - i - 1;
            int index2 = result.path.size() - (i-1) - 1;
            std::cout << "Vamp found collision in solution path between " << index1 << " and " << index2 << std::endl;
            vamp_valid = false;
            break;
        }
    }
}

int main(int argc, char* argv[]) {
    std::string robot_name = "panda";
    std::string name = "cage";
    int problem_idx = 1;
    bool use_pointcloud = false;
    if (argc == 5) {
        robot_name = argv[1];
        name = argv[2];
        problem_idx = std::stoi(argv[3]);
        use_pointcloud = std::stoi(argv[4]);
    }
    else {
        std::cout << "Usage: ./single_mbm <robot_name> <problem_name> <problem_idx> <use_pointcloud>\n";
        return 1;
    }
    std::string path = "scripts/" + robot_name + "_problems_pointcloud.json";
    std::ifstream f(path);
    json all_data = json::parse(f);
    json problems = all_data["problems"];
    std::vector<std::string> problem_names;
    for (auto& [name, pset] : problems.items()) problem_names.push_back(name);
    auto pset = problems[name];
    if (pset.empty()) {
        std::cerr << "Problem " << name << " not found\n";
        std::cout << "Available problems:\n";
        for (auto& name : problem_names) std::cout << name << "\n";
        return 1;
    }
    json data = pset[problem_idx - 1];
    if (not data["valid"]) {
        std::cerr << "Problem " << name << " is invalid\n";
        return -1;
    }
    auto env = problem_dict_to_env(data, name, use_pointcloud);
    auto vamp_env = problem_dict_vamp(data, name);
    struct pRRTC_settings settings;
    settings.num_new_configs = 512; //usually:512
    settings.max_iters = 100000000;
    settings.granularity = 16;
    settings.range = 0.25;
    settings.balance = 2;
    settings.tree_ratio = 1.0;
    settings.dynamic_domain = true;
    settings.dd_radius = 4.0;
    settings.dd_min_radius = 1.0;
    settings.dd_alpha = 0.0001;
    if (robot_name == "fetch") {
        run_planner<robots::Fetch, vamp::robots::Fetch>(data, env, vamp_env, settings);
    } else if (robot_name == "panda") {
        run_planner<robots::Panda, vamp::robots::Panda>(data, env, vamp_env, settings);
    } else if (robot_name == "baxter") {
        run_planner<robots::Baxter, vamp::robots::Baxter>(data, env, vamp_env, settings);
    } else {
        std::cerr << "Unsupported robot type: " << robot_name << "\n";
        return 1;
    }
}
