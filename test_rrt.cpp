#include "RRT1.h"

int main() {
    std::vector<float> obstacles = {
        5.0, 5.0, 5.0, 1.0,
        50.0, 50.0, 50.0, 0.1,
        -10.0, -25.0, -40.0, 0.1
    };

    Configuration start = {0.1, 0.2, 0.1};
    Configuration goal = {10.0, 10.0, 10.0};

    solve<3>(start, goal, obstacles);

}

