#pragma once

struct pRRTC_settings {
    int max_samples = 1000000;
    int max_iters = 1000000;
    int num_new_configs = 600;
    int granularity = 64;
    float range = 0.5;

    bool balance = true;
    float tree_ratio = 0.5;

    bool dynamic_domain = true;
    float dd_alpha = 0.0001;
    float dd_radius = 4.0;
    float dd_min_radius = 1.0;
};