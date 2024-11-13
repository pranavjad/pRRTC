#pragma once

namespace ppln::robots {
    struct Panda
    {
        static constexpr auto name = "panda";
        static constexpr auto dim = 7;
        using Configuration = std::array<float, dim>;
    }

    struct Sphere
    {
        static constexpr auto name = "sphere";
        static constexpr auto dim = 3;
        using Configuration = std::array<float, dim>;
    }
}