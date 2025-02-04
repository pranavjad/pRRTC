#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#include "src/collision/environment.hh"
#include "src/collision/factory.hh"
#include "src/planning/Planners.hh"
#include "src/planning/pRRTC_settings.hh"



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

void print_csv_header(std::ofstream &outfile) {
    outfile << "problem_name,problem_idx,solved,cost,path_length,start_tree_size,goal_tree_size,iters,wall_ns,kernel_ns,";
    outfile << "num_new_configs,granularity,range,balance,tree_ratio,dynamic_domain,dd_alpha,dd_radius,dd_min_radius\n";
}
template<typename Robot>
void print_planner_result_to_file(PlannerResult<Robot> &result, pRRTC_settings &settings, std::string problem_name, int problem_idx, std::ofstream &outfile) {
    outfile << problem_name << ", ";
    outfile << problem_idx << ", ";
    outfile << result.solved << ", ";
    outfile << result.cost << ", ";
    outfile << result.path_length << ", ";
    outfile << result.start_tree_size << ", ";
    outfile << result.goal_tree_size << ", ";
    outfile << result.iters << ", ";
    outfile << result.wall_ns << ", ";
    outfile << result.kernel_ns << ", ";
    outfile << settings.num_new_configs << ", ";
    outfile << settings.granularity << ", ";
    outfile << settings.range << ", ";
    outfile << settings.balance << ", ";
    outfile << settings.tree_ratio << ", ";
    outfile << settings.dynamic_domain << ", ";
    outfile << settings.dd_alpha << ", ";
    outfile << settings.dd_radius << ", ";
    outfile << settings.dd_min_radius;
    outfile << "\n";
}


template<typename Robot>
void run_planning(const json &problems, pRRTC_settings &settings, std::string run_name, std::string robot_name) {
    using Configuration = typename Robot::Configuration;
    std::ofstream outfile("test_output/"+robot_name+"_"+run_name+".csv");
    print_csv_header(outfile);

    int failed = 0;
    std::map<std::string, std::vector<PlannerResult<Robot>>> results;
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
            auto result = pRRTC::solve<Robot>(start, goals, env, settings);
            for (auto& cfg: result.path) {
                print_cfg<Robot>(cfg);
            }
            std::cout << "kernel_ns: " << result.kernel_ns << "\n";
            if (not result.solved) {
                failed ++;
                std::cout << "failed " << name << std::endl;
            }
            std::cout << "cost: " << result.cost << "\n";
            results[name].emplace_back(result);
            print_planner_result_to_file(result, settings, name, i+1, outfile);
        }
    }

    // for (auto& name : prob_names) {
    //     std::cout << name << std::endl;

    //     // calculate stats for cost 
    //     float avg_cost = 0;
    //     float min_cost = 1e9;
    //     float max_cost = 0;
    //     for (auto &result : results[name]) {
    //         avg_cost += result.cost;
    //         min_cost = std::min(min_cost, result.cost);
    //         max_cost = std::max(max_cost, result.cost);
    //     }
    //     avg_cost /= results[name].size();
    //     std::cout << "avg cost: " << avg_cost << std::endl;
    //     std::cout << "min cost: " << min_cost << std::endl;
    //     std::cout << "max cost: " << max_cost << std::endl;
    //     std::cout << std::endl;

    //     // calculate stats for cost 
    //     std::size_t net_time = 0;
    //     std::size_t min_time = 1e9;
    //     std::size_t max_time = 0;
    //     for (auto &result : results[name]) {
    //         net_time += result.nanoseconds;
    //         min_time = std::min(min_time, result.nanoseconds);
    //         max_time = std::max(max_time, result.nanoseconds);
    //     }
    //     float avg_time = net_time / results[name].size();
    //     std::cout << "avg time (μs): " << avg_time/1000 << std::endl;
    //     std::cout << "min time (μs): " << min_time/1000 << std::endl;
    //     std::cout << "max time (μs): " << max_time/1000 << std::endl;
    //     std::cout << "----" << std::endl;
    // }
}

int main(int argc, char* argv[]) {
    std::string robot_name = "panda";
    std::string run_name;
    pRRTC_settings settings;
    // 2, 128, 0.5, 1, 0
    settings.num_new_configs = 512;
    settings.granularity = 128;
    settings.range = 0.25;
    settings.balance = 1;
    settings.tree_ratio = 0.5;
    settings.dynamic_domain = true;
    // settings.dd_min_radius = settings.range / 2.0f;
    

    if (argc == 3) {
        robot_name = argv[1];
        run_name = argv[2];
    }
    else {
        std::cout << "Usage: evaluate_mbm <robot_name> <run_name>\n";
        return -1;
    }

    std::string path = "scripts/" + robot_name + "_problems.json";
    std::ifstream f(path);
    json all_data = json::parse(f);
    json problems = all_data["problems"];
    if (robot_name == "fetch") {
        run_planning<robots::Fetch>(problems, settings, run_name, robot_name);
    } else if (robot_name == "panda") {
        run_planning<robots::Panda>(problems, settings, run_name, robot_name);
    } else {
        std::cerr << "Unsupported robot type: " << robot_name << "\n";
        return 1;
    }
}
// 2, 128, 1, 2, 1