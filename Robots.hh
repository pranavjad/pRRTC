#pragma once

#include <array>
#include <iostream>

namespace ppln::robots {
    struct Panda
    {
        static constexpr auto name = "panda";
        static constexpr auto dimension = 7;
        using Configuration = std::array<float, dimension>;

        static constexpr Configuration s_m{
            5.9342f,
            3.6652f,
            5.9342f,
            3.2289f,
            5.9342f,
            3.9095999999999997f,
            5.9342f
        };
        
        static constexpr Configuration s_a{
            -2.9671f,
            -1.8326f,
            -2.9671f,
            -3.1416f,
            -2.9671f,
            -0.0873f,
            -2.9671f
        };

        inline static void print_robot_config(Configuration &cfg) {
            for (int i = 0; i < dimension; i++) {
                std::cout << cfg[i] << ' ';
            }
            std::cout << '\n';
        };
    };

    struct Sphere
    {
        static constexpr auto name = "sphere";
        static constexpr auto dimension = 3;
        static constexpr float radius = 1.0;
        using Configuration = std::array<float, dimension>;

        static constexpr Configuration s_m{
            10.0f,
            10.0f,
            10.0f
        };
        
        static constexpr Configuration s_a{
            -5.0f,
            -5.0f,
            -5.0f
        };

        inline static void print_robot_config(Configuration &cfg) {
            for (int i = 0; i < dimension; i++) {
                std::cout << cfg[i] << ' ';
            }
            std::cout << '\n';
        };
    };
}