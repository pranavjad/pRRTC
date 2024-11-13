#pragma once

#include <array>

namespace ppln::robots {
    struct Panda
    {
        static constexpr auto name = "panda";
        static constexpr auto dimension = 7;
        using Configuration = std::array<float, dimension>;
    };

    struct Sphere
    {
        static constexpr auto name = "sphere";
        static constexpr auto dimension = 3;
        using Configuration = std::array<float, dimension>;
    };
}