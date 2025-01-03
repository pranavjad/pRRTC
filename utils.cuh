#pragma once

#include "Robots.hh"
#include "collision/environment.hh"
#include "collision/shapes.hh"
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>

#include <iostream>
#include <cassert>



namespace ppln::device_utils {
    using namespace collision;

    /* math utils */
    __device__ inline constexpr auto dot_2(const float &ax, const float &ay, const float &bx, const float &by) -> float
    {
        return (ax * bx) + (ay * by);
    }

     __device__ inline constexpr auto dot_3(
    const float ax,
    const float ay,
    const float az,
    const float bx,
    const float by,
    const float bz) -> float
    {
        return ax * bx + ay * by + az * bz;
    }

    __device__ inline constexpr auto sql2_3(
        const float &ax,
        const float &ay,
        const float &az,
        const float &bx,
        const float &by,
        const float &bz) -> float
    {
        const auto xs = (ax - bx);
        const auto ys = (ay - by);
        const auto zs = (az - bz);

        return dot_3(xs, ys, zs, xs, ys, zs);
    }

    __device__ inline constexpr auto clamp(const float &v, const float &lower, const float &upper) -> float
    {
        return fmaxf(fminf(v, upper), lower);
    }
    /* end math utils */


    /* Sphere collision utils*/

    __device__ inline constexpr auto sphere_sphere_sql2(
        const float ax,
        const float ay,
        const float az,
        const float ar,
        const float bx,
        const float by,
        const float bz,
        const float br) -> float
    {
        auto sum = sql2_3(ax, ay, az, bx, by, bz);
        auto rs = ar + br;
        return sum - rs * rs;
    }

    __device__ inline constexpr auto sphere_sphere_sql2(
        const Sphere<float> &a,
        const float &x,
        const float &y,
        const float &z,
        const float &r) -> float
    {
        return sphere_sphere_sql2(a.x, a.y, a.z, a.r, x, y, z, r);
    }

    __device__ inline constexpr auto sphere_sphere_self_collision(float ax, float ay, float az, float ar, float bx, float by, float bz, float br)
    {
        return (sphere_sphere_sql2(ax, ay, az, ar, bx, by, bz, br) < 0);
    }

    // returns squared l2 distance between two configs
    __device__ inline float l2_dist(float *config_a, float *config_b, const int dim) {
        float ans = 0;
        float diff;
        for (int i = 0; i < dim; i++) {
            diff = config_a[i] - config_b[i];
            ans += diff * diff;
        }
        return sqrt(ans);
    }
    /* End Sphere collision utils*/

    /* Capsule collision utils */
    __device__ inline constexpr auto sphere_capsule(
        const Capsule<float> &c,
        const float &x,
        const float &y,
        const float &z,
        const float &r) noexcept -> float
    {
        auto dot = dot_3(x - c.x1, y - c.y1, z - c.z1, c.xv, c.yv, c.zv);
        auto cdf = clamp((dot * c.rdv), 0.F, 1.F);

        auto sum = sql2_3(x, y, z, c.x1 + c.xv * cdf, c.y1 + c.yv * cdf, c.z1 + c.zv * cdf);
        auto rs = r + c.r;
        return sum - rs * rs;
    }

    
    __device__ inline constexpr auto sphere_capsule(const Capsule<float> &c, const Sphere<float> &s) noexcept -> float
    {
        return sphere_capsule(c, s.x, s.y, s.z, s.r);
    }

    
    __device__ inline constexpr auto sphere_z_aligned_capsule(
        const Capsule<float> &c,
        const float &x,
        const float &y,
        const float &z,
        const float &r) noexcept -> float
    {
        auto dot = (z - c.z1) * c.zv;
        auto cdf = clamp((dot * c.rdv), 0.F, 1.F);

        auto sum = sql2_3(x, y, z, c.x1, c.y1, c.z1 + c.zv * cdf);
        auto rs = r + c.r;
        return sum - rs * rs;
    }

    __device__ inline constexpr auto sphere_z_aligned_capsule(const Capsule<float> &c, const Sphere<float> &s) noexcept
        -> float
    {
        return sphere_z_aligned_capsule(c, s.x, s.y, s.z, s.r);
    }
    /* End Capsule collision functions*/

    /* Cuboid collision functions*/
    __device__ inline constexpr auto sphere_cuboid(
        const Cuboid<float> &c,
        const float &x,
        const float &y,
        const float &z,
        const float &rsq) noexcept -> float
    {
        auto xs = x - c.x;
        auto ys = y - c.y;
        auto zs = z - c.z;

        auto a1 = fmaxf(0., abs(dot_3(c.axis_1_x, c.axis_1_y, c.axis_1_z, xs, ys, zs)) - c.axis_1_r);
        auto a2 = fmaxf(0., abs(dot_3(c.axis_2_x, c.axis_2_y, c.axis_2_z, xs, ys, zs)) - c.axis_2_r);
        auto a3 = fmaxf(0., abs(dot_3(c.axis_3_x, c.axis_3_y, c.axis_3_z, xs, ys, zs)) - c.axis_3_r);

        auto sum = dot_3(a1, a2, a3, a1, a2, a3);
        return sum - rsq;
    }

    
    __device__ inline constexpr auto sphere_cuboid(const Cuboid<float> &c, const Sphere<float> &s) noexcept -> float
    {
        return sphere_cuboid(c, s.x, s.y, s.z, s.r * s.r);
    }

    
    __device__ inline constexpr auto sphere_z_aligned_cuboid(
        const Cuboid<float> &c,
        const float &x,
        const float &y,
        const float &z,
        const float &rsq) noexcept -> float
    {
        auto xs = x - c.x;
        auto ys = y - c.y;
        auto zs = z - c.z;

        auto a1 = fmaxf(0., (abs(dot_2(c.axis_1_x, c.axis_1_y, xs, ys)) - c.axis_1_r));
        auto a2 = fmaxf(0., (abs(dot_2(c.axis_2_x, c.axis_2_y, xs, ys)) - c.axis_2_r));
        auto a3 = fmaxf(0, (abs(zs) - c.axis_3_r));

        auto sum = dot_3(a1, a2, a3, a1, a2, a3);
        return sum - rsq;
    }

    
    __device__ inline constexpr auto sphere_z_aligned_cuboid(const Cuboid<float> &c, const Sphere<float> &s) noexcept
        -> float
    {
        return sphere_z_aligned_cuboid(c, s.x, s.y, s.z, s.r * s.r);
    }
    /* End Cuboid collision functions*/




    __device__ inline bool sphere_environment_in_collision(ppln::collision::Environment<float> *env, float sx_, float sy_, float sz_, float sr_)
    {
        const float rsq = sr_ * sr_;
        bool in_collision = false;

        for (unsigned int i = 0; i < env->num_spheres && !in_collision; i++)
        {
            in_collision |= (sphere_sphere_sql2(env->spheres[i], sx_, sy_, sz_, sr_) < 0);
        }

        for (unsigned int i = 0; i < env->num_capsules && !in_collision; i++)
        {
            in_collision |= (sphere_capsule(env->capsules[i], sx_, sy_, sz_, sr_) < 0);
        }

        for (unsigned int i = 0; i < env->num_z_aligned_capsules && !in_collision; i++)
        {
            in_collision |= (sphere_z_aligned_capsule(env->z_aligned_capsules[i], sx_, sy_, sz_, sr_) < 0);
        }

        for (unsigned int i = 0; i < env->num_cuboids && !in_collision; i++)
        {
            in_collision |= (sphere_cuboid(env->cuboids[i], sx_, sy_, sz_, rsq) < 0);
        }

        for (unsigned int i = 0; i < env->num_z_aligned_cuboids && !in_collision; i++)
        {
            in_collision |= (sphere_z_aligned_cuboid(env->z_aligned_cuboids[i], sx_, sy_, sz_, rsq) < 0);
        }

        return in_collision;
    }

    __global__ void init_rng(curandState* states, unsigned long seed);

    /* Robot constants */
    // TODO: have one source of truth for robot constants between device and host
    template<typename Robot>
    struct DeviceRobotConstants {
        static __device__ __host__ constexpr float get_s_m(int idx) {
            float result = 0.0f;
            if constexpr (std::is_same_v<Robot, robots::Panda>) {
                constexpr float values[] = {
                    5.9342f, 3.6652f, 5.9342f, 3.2289f,
                    5.9342f, 3.9095999999999997f, 5.9342f
                };
                result = values[idx];
            }
            else if constexpr (std::is_same_v<Robot, robots::Sphere>) {
                constexpr float values[] = {10.0f, 10.0f, 10.0f};
                result = values[idx];
            }
            return result;
        }

        static __device__ __host__ constexpr float get_s_a(int idx) {
            float result = 0.0f;
            if constexpr (std::is_same_v<Robot, robots::Panda>) {
                constexpr float values[] = {
                    -2.9671f, -1.8326f, -2.9671f, -3.1416f,
                    -2.9671f, -0.0873f, -2.9671f
                };
                result = values[idx];
            }
            else if constexpr (std::is_same_v<Robot, robots::Sphere>) {
                constexpr float values[] = {-5.0f, -5.0f, -5.0f};
                result = values[idx];
            }
            return result;
        }
    };

    // maps [0, 1] --> robot joint ranges
    template <typename Robot>
    __device__ inline void scale_configuration(float *q)
    {
        for (int i = 0; i < Robot::dimension; i++) {
            q[i] = q[i] * DeviceRobotConstants<Robot>::get_s_m(i) + DeviceRobotConstants<Robot>::get_s_a(i);
        }
    }
}


namespace ppln::collision {
    using namespace device_utils;
    
    // fkcc -> checks if the config is "good"
    // returns true if the config does collide with an obstacle, returns false if the config does not collide
    template <typename Robot>
    __device__ inline bool fkcc(float *config, ppln::collision::Environment<float> *env);

    // template <>
    // __device__ bool fkcc<ppln::robots::Sphere>(float *config, ppln::collision::Environment<float> *env);

    // template <>
    // __device__ bool fkcc<ppln::robots::Panda>(float *config, ppln::collision::Environment<float> *env);

}

// Error checking macro
#define cudaCheckError(ans) { cudaAssert((ans), __FILE__, __LINE__); }
inline void cudaAssert(cudaError_t code, const char *file, int line) {
    if (code != cudaSuccess) {
        fprintf(stderr, "CUDA Error: %s %s %d\n", cudaGetErrorString(code), file, line);
        exit(code);
    }
}

inline void printCUDADeviceInfo() {
    int deviceCount;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    
    if (error != cudaSuccess) {
        std::cerr << "Error getting device count: " << cudaGetErrorString(error) << std::endl;
        return;
    }
    
    std::cout << "Found " << deviceCount << " CUDA device(s)\n\n";
    
    for (int device = 0; device < deviceCount; ++device) {
        cudaDeviceProp prop;
        error = cudaGetDeviceProperties(&prop, device);
        
        if (error != cudaSuccess) {
            std::cerr << "Error getting device properties: " << cudaGetErrorString(error) << std::endl;
            continue;
        }
        
        std::cout << "Device " << device << ": " << prop.name << "\n";
        std::cout << "============================================\n";
        std::cout << "Compute Capability: " << prop.major << "." << prop.minor << "\n";
        
        // Memory Information
        std::cout << "Total Global Memory: " << prop.totalGlobalMem / (1024.0 * 1024.0) << " MB\n";
        std::cout << "Total Constant Memory: " << prop.totalConstMem / 1024.0 << " KB\n";
        std::cout << "Shared Memory Per Block: " << prop.sharedMemPerBlock / 1024.0 << " KB\n";
        std::cout << "L2 Cache Size: " << prop.l2CacheSize / 1024.0 << " KB\n";
        
        // Thread Information
        std::cout << "Max Threads Per Block: " << prop.maxThreadsPerBlock << "\n";
        std::cout << "Max Threads Per Multiprocessor: " << prop.maxThreadsPerMultiProcessor << "\n";
        std::cout << "Number of Multiprocessors: " << prop.multiProcessorCount << "\n";
        std::cout << "Warp Size: " << prop.warpSize << "\n";
        
        // Block Dimensions
        std::cout << "Max Block Dimensions: " 
                << prop.maxThreadsDim[0] << " x "
                << prop.maxThreadsDim[1] << " x "
                << prop.maxThreadsDim[2] << "\n";
        
        // Grid Dimensions
        std::cout << "Max Grid Dimensions: "
                << prop.maxGridSize[0] << " x "
                << prop.maxGridSize[1] << " x "
                << prop.maxGridSize[2] << "\n";
        
        // Memory Clock and Bus Width
        std::cout << "Memory Clock Rate: " << prop.memoryClockRate / 1000.0 << " MHz\n";
        std::cout << "Memory Bus Width: " << prop.memoryBusWidth << " bits\n";
        
        // Additional Features
        std::cout << "Unified Addressing: " << (prop.unifiedAddressing ? "Yes" : "No") << "\n";
        std::cout << "Concurrent Kernels: " << (prop.concurrentKernels ? "Yes" : "No") << "\n";
        std::cout << "ECC Enabled: " << (prop.ECCEnabled ? "Yes" : "No") << "\n";
        
        std::cout << "\n";
    }
}

/* testing making collision stuff inline temporarily*/


/* Collision checking backend implementations for different robots */
namespace ppln::collision {
    using namespace device_utils;
    
    // fkcc -> checks if the config is "good"
    // returns false if the config does collide with an obstacle, returns true if the config does not collide
    // template <typename Robot>
    // __device__ bool fkcc(float *config, ppln::collision::Environment<float> *env);


    template <>
    __device__ inline bool fkcc<ppln::robots::Sphere>(float *config, ppln::collision::Environment<float> *env)
    {
        return not sphere_environment_in_collision(env, config[0], config[1], config[2], ppln::robots::Sphere::radius);
    }

    template <>
    __device__ inline bool fkcc<ppln::robots::Panda>(float *q, ppln::collision::Environment<float> *environment)
    {
        // Ignore static frame collisions - needed for some evaluation problems
        // if (/*panda_link0*/ sphere_environment_in_collision(environment, 0.0, 0.0, 0.05, 0.08))
        // {
        //     return false;
        // }  // (0, 0)
        auto INPUT_0 = q[0];
        auto DIV_8 = INPUT_0 * 0.5;
        auto SIN_9 = sin(DIV_8);
        auto COS_15 = cos(DIV_8);
        auto MUL_1575 = COS_15 * SIN_9;
        auto MUL_1594 = MUL_1575 * 2.0;
        auto MUL_1625 = MUL_1594 * 0.039;
        auto MUL_1574 = SIN_9 * SIN_9;
        auto MUL_1584 = MUL_1574 * 2.0;
        auto SUB_1587 = 1.0 - MUL_1584;
        auto MUL_1614 = SUB_1587 * 0.001;
        auto SUB_1641 = MUL_1625 - MUL_1614;
        auto MUL_1628 = SUB_1587 * 0.039;
        auto MUL_1618 = MUL_1594 * 0.001;
        auto ADD_1642 = MUL_1618 + MUL_1628;
        auto NEGATE_1643 = -ADD_1642;
        auto MUL_1656 = MUL_1594 * 0.08;
        auto MUL_1659 = SUB_1587 * 0.08;
        auto NEGATE_1660 = -MUL_1659;
        auto MUL_1680 = MUL_1594 * 0.03;
        auto MUL_1683 = SUB_1587 * 0.03;
        auto NEGATE_1684 = -MUL_1683;
        if (/*panda_link1*/ sphere_environment_in_collision(environment, SUB_1641, NEGATE_1643, 0.248, 0.154))
        {
            if (sphere_environment_in_collision(environment, MUL_1656, NEGATE_1660, 0.333, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1680, NEGATE_1684, 0.333, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.0, 0.0, 0.213, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.0, 0.0, 0.163, 0.06))
            {
                return false;
            }
        }  // (0, 22)
        auto MUL_74 = COS_15 * 0.7071068;
        auto MUL_72 = SIN_9 * 0.7071068;
        auto INPUT_1 = q[1];
        auto DIV_117 = INPUT_1 * 0.5;
        auto SIN_118 = sin(DIV_117);
        auto COS_124 = cos(DIV_117);
        auto MUL_143 = MUL_72 * COS_124;
        auto MUL_128 = MUL_72 * SIN_118;
        auto MUL_126 = MUL_74 * COS_124;
        auto SUB_149 = MUL_126 - MUL_128;
        auto ADD_130 = MUL_126 + MUL_128;
        auto MUL_140 = MUL_74 * SIN_118;
        auto SUB_138 = MUL_140 - MUL_143;
        auto ADD_144 = MUL_140 + MUL_143;
        auto MUL_1756 = SUB_149 * ADD_144;
        auto MUL_1757 = SUB_149 * SUB_138;
        auto MUL_1763 = ADD_130 * ADD_144;
        auto SUB_1796 = MUL_1757 - MUL_1763;
        auto MUL_1798 = SUB_1796 * 2.0;
        auto MUL_1827 = MUL_1798 * 0.04;
        auto MUL_1761 = ADD_130 * SUB_138;
        auto ADD_1781 = MUL_1761 + MUL_1756;
        auto MUL_1784 = ADD_1781 * 2.0;
        auto MUL_1817 = MUL_1784 * 0.085;
        auto ADD_1832 = MUL_1817 + MUL_1827;
        auto MUL_1759 = SUB_149 * ADD_130;
        auto MUL_1755 = ADD_144 * ADD_144;
        auto MUL_1765 = SUB_138 * ADD_144;
        auto ADD_1799 = MUL_1765 + MUL_1759;
        auto MUL_1801 = ADD_1799 * 2.0;
        auto MUL_1829 = MUL_1801 * 0.04;
        auto MUL_1758 = ADD_130 * ADD_130;
        auto ADD_1786 = MUL_1755 + MUL_1758;
        auto MUL_1789 = ADD_1786 * 2.0;
        auto SUB_1792 = 1.0 - MUL_1789;
        auto MUL_1820 = SUB_1792 * 0.085;
        auto SUB_1833 = MUL_1829 - MUL_1820;
        auto SUB_1793 = MUL_1765 - MUL_1759;
        auto MUL_1795 = SUB_1793 * 2.0;
        auto MUL_1824 = MUL_1795 * 0.085;
        auto MUL_1754 = SUB_138 * SUB_138;
        auto ADD_1802 = MUL_1754 + MUL_1758;
        auto MUL_1805 = ADD_1802 * 2.0;
        auto SUB_1808 = 1.0 - MUL_1805;
        auto MUL_1831 = SUB_1808 * 0.04;
        auto SUB_1834 = MUL_1831 - MUL_1824;
        auto ADD_1835 = 0.333 + SUB_1834;
        auto MUL_1849 = MUL_1798 * 0.03;
        auto MUL_1851 = MUL_1801 * 0.03;
        auto MUL_1853 = SUB_1808 * 0.03;
        auto ADD_1854 = 0.333 + MUL_1853;
        auto MUL_1868 = MUL_1798 * 0.08;
        auto MUL_1870 = MUL_1801 * 0.08;
        auto MUL_1872 = SUB_1808 * 0.08;
        auto ADD_1873 = 0.333 + MUL_1872;
        auto MUL_1882 = MUL_1784 * 0.12;
        auto MUL_1885 = SUB_1792 * 0.12;
        auto NEGATE_1886 = -MUL_1885;
        auto MUL_1889 = MUL_1795 * 0.12;
        auto SUB_1897 = 0.333 - MUL_1889;
        auto MUL_1906 = MUL_1784 * 0.17;
        auto MUL_1909 = SUB_1792 * 0.17;
        auto NEGATE_1910 = -MUL_1909;
        auto MUL_1913 = MUL_1795 * 0.17;
        auto SUB_1921 = 0.333 - MUL_1913;
        if (/*panda_link2*/ sphere_environment_in_collision(environment, ADD_1832, SUB_1833, ADD_1835, 0.154))
        {
            if (sphere_environment_in_collision(environment, MUL_1849, MUL_1851, ADD_1854, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1868, MUL_1870, ADD_1873, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1882, NEGATE_1886, SUB_1897, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1906, NEGATE_1910, SUB_1921, 0.06))
            {
                return false;
            }
        }  // (22, 87)
        auto MUL_182 = SUB_149 * 0.7071068;
        auto MUL_198 = ADD_144 * 0.7071068;
        auto MUL_196 = SUB_138 * 0.7071068;
        auto SUB_209 = MUL_198 - MUL_196;
        auto ADD_199 = MUL_196 + MUL_198;
        auto MUL_184 = ADD_130 * 0.7071068;
        auto SUB_186 = MUL_182 - MUL_184;
        auto ADD_215 = MUL_182 + MUL_184;
        auto MUL_224 = ADD_144 * 0.316;
        auto MUL_235 = SUB_149 * MUL_224;
        auto MUL_228 = ADD_130 * 0.316;
        auto MUL_236 = SUB_138 * MUL_228;
        auto ADD_237 = MUL_235 + MUL_236;
        auto MUL_240 = ADD_237 * 2.0;
        auto INPUT_2 = q[2];
        auto DIV_262 = INPUT_2 * 0.5;
        auto SIN_263 = sin(DIV_262);
        auto COS_269 = cos(DIV_262);
        auto MUL_286 = ADD_215 * COS_269;
        auto MUL_281 = ADD_215 * SIN_263;
        auto MUL_284 = SUB_209 * COS_269;
        auto ADD_285 = MUL_281 + MUL_284;
        auto MUL_1933 = ADD_285 * ADD_285;
        auto MUL_289 = SUB_209 * SIN_263;
        auto SUB_290 = MUL_286 - MUL_289;
        auto MUL_1934 = SUB_290 * ADD_285;
        auto MUL_271 = SUB_186 * COS_269;
        auto MUL_276 = SUB_186 * SIN_263;
        auto MUL_278 = ADD_199 * COS_269;
        auto SUB_279 = MUL_278 - MUL_276;
        auto MUL_1935 = SUB_290 * SUB_279;
        auto MUL_1932 = SUB_279 * SUB_279;
        auto ADD_1941 = MUL_1932 + MUL_1933;
        auto MUL_1944 = ADD_1941 * 2.0;
        auto SUB_1947 = 1.0 - MUL_1944;
        auto MUL_1981 = SUB_1947 * 0.039;
        auto MUL_272 = ADD_199 * SIN_263;
        auto ADD_273 = MUL_271 + MUL_272;
        auto MUL_1939 = ADD_273 * ADD_285;
        auto ADD_1967 = MUL_1939 + MUL_1935;
        auto MUL_1969 = ADD_1967 * 2.0;
        auto MUL_1994 = MUL_1969 * 0.052;
        auto MUL_1938 = ADD_273 * SUB_279;
        auto SUB_1954 = MUL_1938 - MUL_1934;
        auto MUL_1956 = SUB_1954 * 2.0;
        auto MUL_1987 = MUL_1956 * 0.028;
        auto ADD_2004 = MUL_1981 + MUL_1987;
        auto SUB_2007 = ADD_2004 - MUL_1994;
        auto ADD_2010 = MUL_240 + SUB_2007;
        auto ADD_1948 = MUL_1938 + MUL_1934;
        auto MUL_1950 = ADD_1948 * 2.0;
        auto MUL_1983 = MUL_1950 * 0.039;
        auto MUL_1937 = SUB_290 * ADD_273;
        auto MUL_1940 = SUB_279 * ADD_285;
        auto SUB_1970 = MUL_1940 - MUL_1937;
        auto MUL_1972 = SUB_1970 * 2.0;
        auto MUL_1998 = MUL_1972 * 0.052;
        auto MUL_1936 = ADD_273 * ADD_273;
        auto ADD_1957 = MUL_1933 + MUL_1936;
        auto MUL_1960 = ADD_1957 * 2.0;
        auto SUB_1963 = 1.0 - MUL_1960;
        auto MUL_1989 = SUB_1963 * 0.028;
        auto ADD_2005 = MUL_1983 + MUL_1989;
        auto SUB_2008 = ADD_2005 - MUL_1998;
        auto MUL_246 = ADD_144 * MUL_224;
        auto MUL_244 = ADD_130 * MUL_228;
        auto ADD_247 = MUL_244 + MUL_246;
        auto MUL_249 = ADD_247 * 2.0;
        auto SUB_252 = MUL_249 - 0.316;
        auto ADD_2011 = SUB_252 + SUB_2008;
        auto SUB_1951 = MUL_1939 - MUL_1935;
        auto ADD_1964 = MUL_1940 + MUL_1937;
        auto ADD_1973 = MUL_1932 + MUL_1936;
        auto MUL_1976 = ADD_1973 * 2.0;
        auto SUB_1979 = 1.0 - MUL_1976;
        auto MUL_2002 = SUB_1979 * 0.052;
        auto MUL_1966 = ADD_1964 * 2.0;
        auto MUL_1991 = MUL_1966 * 0.028;
        auto MUL_1953 = SUB_1951 * 2.0;
        auto MUL_1985 = MUL_1953 * 0.039;
        auto ADD_2006 = MUL_1985 + MUL_1991;
        auto SUB_2009 = ADD_2006 - MUL_2002;
        auto MUL_253 = SUB_149 * MUL_228;
        auto MUL_255 = SUB_138 * MUL_224;
        auto SUB_256 = MUL_253 - MUL_255;
        auto MUL_258 = SUB_256 * 2.0;
        auto ADD_260 = 0.333 + MUL_258;
        auto ADD_2012 = ADD_260 + SUB_2009;
        auto MUL_2027 = MUL_1969 * 0.1;
        auto SUB_2037 = MUL_240 - MUL_2027;
        auto MUL_2031 = MUL_1972 * 0.1;
        auto SUB_2038 = SUB_252 - MUL_2031;
        auto MUL_2035 = SUB_1979 * 0.1;
        auto SUB_2039 = ADD_260 - MUL_2035;
        auto MUL_2054 = MUL_1969 * 0.06;
        auto SUB_2064 = MUL_240 - MUL_2054;
        auto MUL_2058 = MUL_1972 * 0.06;
        auto SUB_2065 = SUB_252 - MUL_2058;
        auto MUL_2062 = SUB_1979 * 0.06;
        auto SUB_2066 = ADD_260 - MUL_2062;
        auto MUL_2074 = MUL_1956 * 0.06;
        auto MUL_2068 = SUB_1947 * 0.08;
        auto ADD_2085 = MUL_2068 + MUL_2074;
        auto ADD_2088 = MUL_240 + ADD_2085;
        auto MUL_2076 = SUB_1963 * 0.06;
        auto MUL_2070 = MUL_1950 * 0.08;
        auto ADD_2086 = MUL_2070 + MUL_2076;
        auto ADD_2089 = SUB_252 + ADD_2086;
        auto MUL_2078 = MUL_1966 * 0.06;
        auto MUL_2072 = MUL_1953 * 0.08;
        auto ADD_2087 = MUL_2072 + MUL_2078;
        auto ADD_2090 = ADD_260 + ADD_2087;
        auto MUL_2098 = MUL_1956 * 0.02;
        auto ADD_2109 = MUL_2068 + MUL_2098;
        auto ADD_2112 = MUL_240 + ADD_2109;
        auto MUL_2100 = SUB_1963 * 0.02;
        auto ADD_2110 = MUL_2070 + MUL_2100;
        auto ADD_2113 = SUB_252 + ADD_2110;
        auto MUL_2102 = MUL_1966 * 0.02;
        auto ADD_2111 = MUL_2072 + MUL_2102;
        auto ADD_2114 = ADD_260 + ADD_2111;
        if (/*panda_link3*/ sphere_environment_in_collision(environment, ADD_2010, ADD_2011, ADD_2012, 0.128))
        {
            if (sphere_environment_in_collision(environment, SUB_2037, SUB_2038, SUB_2039, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2064, SUB_2065, SUB_2066, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2088, ADD_2089, ADD_2090, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2112, ADD_2113, ADD_2114, 0.055))
            {
                return false;
            }
        }  // (87, 208)
        auto MUL_323 = SUB_290 * 0.7071068;
        auto MUL_338 = ADD_285 * 0.7071068;
        auto MUL_336 = SUB_279 * 0.7071068;
        auto SUB_349 = MUL_338 - MUL_336;
        auto ADD_339 = MUL_336 + MUL_338;
        auto MUL_325 = ADD_273 * 0.7071068;
        auto SUB_354 = MUL_323 - MUL_325;
        auto ADD_326 = MUL_323 + MUL_325;
        auto MUL_371 = ADD_285 * 0.0825;
        auto MUL_376 = ADD_285 * MUL_371;
        auto MUL_366 = SUB_279 * 0.0825;
        auto MUL_374 = SUB_279 * MUL_366;
        auto ADD_378 = MUL_374 + MUL_376;
        auto MUL_381 = ADD_378 * 2.0;
        auto SUB_384 = 0.0825 - MUL_381;
        auto ADD_403 = MUL_240 + SUB_384;
        auto INPUT_3 = q[3];
        auto DIV_407 = INPUT_3 * 0.5;
        auto SIN_408 = sin(DIV_407);
        auto COS_414 = cos(DIV_407);
        auto MUL_431 = SUB_354 * COS_414;
        auto MUL_426 = SUB_354 * SIN_408;
        auto MUL_429 = SUB_349 * COS_414;
        auto ADD_430 = MUL_426 + MUL_429;
        auto MUL_2126 = ADD_430 * ADD_430;
        auto MUL_434 = SUB_349 * SIN_408;
        auto SUB_435 = MUL_431 - MUL_434;
        auto MUL_2127 = SUB_435 * ADD_430;
        auto MUL_416 = ADD_326 * COS_414;
        auto MUL_421 = ADD_326 * SIN_408;
        auto MUL_423 = ADD_339 * COS_414;
        auto SUB_424 = MUL_423 - MUL_421;
        auto MUL_2128 = SUB_435 * SUB_424;
        auto MUL_2125 = SUB_424 * SUB_424;
        auto ADD_2134 = MUL_2125 + MUL_2126;
        auto MUL_2137 = ADD_2134 * 2.0;
        auto SUB_2140 = 1.0 - MUL_2137;
        auto MUL_2175 = SUB_2140 * 0.042;
        auto MUL_417 = ADD_339 * SIN_408;
        auto ADD_418 = MUL_416 + MUL_417;
        auto MUL_2132 = ADD_418 * ADD_430;
        auto ADD_2160 = MUL_2132 + MUL_2128;
        auto MUL_2162 = ADD_2160 * 2.0;
        auto MUL_2192 = MUL_2162 * 0.029;
        auto MUL_2131 = ADD_418 * SUB_424;
        auto SUB_2147 = MUL_2131 - MUL_2127;
        auto MUL_2149 = SUB_2147 * 2.0;
        auto MUL_2186 = MUL_2149 * 0.049;
        auto SUB_2197 = MUL_2186 - MUL_2175;
        auto ADD_2200 = SUB_2197 + MUL_2192;
        auto ADD_2203 = ADD_403 + ADD_2200;
        auto ADD_2141 = MUL_2131 + MUL_2127;
        auto MUL_2143 = ADD_2141 * 2.0;
        auto MUL_2179 = MUL_2143 * 0.042;
        auto MUL_2130 = SUB_435 * ADD_418;
        auto MUL_2133 = SUB_424 * ADD_430;
        auto SUB_2163 = MUL_2133 - MUL_2130;
        auto MUL_2165 = SUB_2163 * 2.0;
        auto MUL_2194 = MUL_2165 * 0.029;
        auto MUL_2129 = ADD_418 * ADD_418;
        auto ADD_2150 = MUL_2126 + MUL_2129;
        auto MUL_2153 = ADD_2150 * 2.0;
        auto SUB_2156 = 1.0 - MUL_2153;
        auto MUL_2188 = SUB_2156 * 0.049;
        auto SUB_2198 = MUL_2188 - MUL_2179;
        auto ADD_2201 = SUB_2198 + MUL_2194;
        auto MUL_386 = SUB_290 * MUL_371;
        auto MUL_387 = ADD_273 * MUL_366;
        auto ADD_389 = MUL_386 + MUL_387;
        auto MUL_392 = ADD_389 * 2.0;
        auto ADD_404 = SUB_252 + MUL_392;
        auto ADD_2204 = ADD_404 + ADD_2201;
        auto SUB_2144 = MUL_2132 - MUL_2128;
        auto ADD_2157 = MUL_2133 + MUL_2130;
        auto ADD_2166 = MUL_2125 + MUL_2129;
        auto MUL_2169 = ADD_2166 * 2.0;
        auto SUB_2172 = 1.0 - MUL_2169;
        auto MUL_2196 = SUB_2172 * 0.029;
        auto MUL_2159 = ADD_2157 * 2.0;
        auto MUL_2190 = MUL_2159 * 0.049;
        auto MUL_2146 = SUB_2144 * 2.0;
        auto MUL_2183 = MUL_2146 * 0.042;
        auto SUB_2199 = MUL_2190 - MUL_2183;
        auto ADD_2202 = SUB_2199 + MUL_2196;
        auto MUL_394 = SUB_290 * MUL_366;
        auto MUL_396 = ADD_273 * MUL_371;
        auto SUB_398 = MUL_396 - MUL_394;
        auto MUL_401 = SUB_398 * 2.0;
        auto ADD_405 = ADD_260 + MUL_401;
        auto ADD_2205 = ADD_405 + ADD_2202;
        auto MUL_2208 = SUB_2140 * 0.08;
        auto MUL_2219 = MUL_2149 * 0.095;
        auto SUB_2230 = MUL_2219 - MUL_2208;
        auto ADD_2233 = ADD_403 + SUB_2230;
        auto MUL_2221 = SUB_2156 * 0.095;
        auto MUL_2212 = MUL_2143 * 0.08;
        auto SUB_2231 = MUL_2221 - MUL_2212;
        auto ADD_2234 = ADD_404 + SUB_2231;
        auto MUL_2223 = MUL_2159 * 0.095;
        auto MUL_2216 = MUL_2146 * 0.08;
        auto SUB_2232 = MUL_2223 - MUL_2216;
        auto ADD_2235 = ADD_405 + SUB_2232;
        auto MUL_2249 = MUL_2162 * 0.02;
        auto ADD_2254 = ADD_403 + MUL_2249;
        auto MUL_2251 = MUL_2165 * 0.02;
        auto ADD_2255 = ADD_404 + MUL_2251;
        auto MUL_2253 = SUB_2172 * 0.02;
        auto ADD_2256 = ADD_405 + MUL_2253;
        auto MUL_2270 = MUL_2162 * 0.06;
        auto ADD_2275 = ADD_403 + MUL_2270;
        auto MUL_2272 = MUL_2165 * 0.06;
        auto ADD_2276 = ADD_404 + MUL_2272;
        auto MUL_2274 = SUB_2172 * 0.06;
        auto ADD_2277 = ADD_405 + MUL_2274;
        auto MUL_2291 = MUL_2149 * 0.06;
        auto SUB_2302 = MUL_2291 - MUL_2208;
        auto ADD_2305 = ADD_403 + SUB_2302;
        auto MUL_2293 = SUB_2156 * 0.06;
        auto SUB_2303 = MUL_2293 - MUL_2212;
        auto ADD_2306 = ADD_404 + SUB_2303;
        auto MUL_2295 = MUL_2159 * 0.06;
        auto SUB_2304 = MUL_2295 - MUL_2216;
        auto ADD_2307 = ADD_405 + SUB_2304;
        if (/*panda_link4*/ sphere_environment_in_collision(environment, ADD_2203, ADD_2204, ADD_2205, 0.126))
        {
            if (sphere_environment_in_collision(environment, ADD_2233, ADD_2234, ADD_2235, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2254, ADD_2255, ADD_2256, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2275, ADD_2276, ADD_2277, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2305, ADD_2306, ADD_2307, 0.055))
            {
                return false;
            }
        }  // (208, 331)
        auto MUL_469 = SUB_435 * 0.7071068;
        auto MUL_486 = ADD_430 * 0.7071068;
        auto MUL_527 = ADD_430 * 0.0825;
        auto MUL_533 = ADD_430 * MUL_527;
        auto MUL_483 = SUB_424 * 0.7071068;
        auto SUB_488 = MUL_483 - MUL_486;
        auto ADD_499 = MUL_483 + MUL_486;
        auto MUL_520 = SUB_424 * 0.0825;
        auto MUL_472 = ADD_418 * 0.7071068;
        auto SUB_473 = MUL_472 - MUL_469;
        auto ADD_506 = MUL_469 + MUL_472;
        auto MUL_514 = ADD_430 * 0.384;
        auto MUL_529 = SUB_435 * MUL_514;
        auto MUL_517 = ADD_418 * 0.384;
        auto ADD_522 = MUL_517 + MUL_520;
        auto MUL_531 = SUB_424 * ADD_522;
        auto SUB_532 = MUL_531 - MUL_529;
        auto ADD_534 = SUB_532 + MUL_533;
        auto MUL_536 = ADD_534 * 2.0;
        auto SUB_539 = MUL_536 - 0.0825;
        auto ADD_564 = ADD_403 + SUB_539;
        auto INPUT_4 = q[4];
        auto DIV_568 = INPUT_4 * 0.5;
        auto SIN_569 = sin(DIV_568);
        auto COS_575 = cos(DIV_568);
        auto MUL_592 = ADD_506 * COS_575;
        auto MUL_587 = ADD_506 * SIN_569;
        auto MUL_590 = ADD_499 * COS_575;
        auto ADD_591 = MUL_587 + MUL_590;
        auto MUL_2319 = ADD_591 * ADD_591;
        auto MUL_595 = ADD_499 * SIN_569;
        auto SUB_596 = MUL_592 - MUL_595;
        auto MUL_2320 = SUB_596 * ADD_591;
        auto MUL_577 = SUB_473 * COS_575;
        auto MUL_582 = SUB_473 * SIN_569;
        auto MUL_584 = SUB_488 * COS_575;
        auto SUB_585 = MUL_584 - MUL_582;
        auto MUL_2321 = SUB_596 * SUB_585;
        auto MUL_2318 = SUB_585 * SUB_585;
        auto ADD_2327 = MUL_2318 + MUL_2319;
        auto MUL_2330 = ADD_2327 * 2.0;
        auto SUB_2333 = 1.0 - MUL_2330;
        auto MUL_2368 = SUB_2333 * 0.001;
        auto MUL_578 = SUB_488 * SIN_569;
        auto ADD_579 = MUL_577 + MUL_578;
        auto MUL_2325 = ADD_579 * ADD_591;
        auto ADD_2353 = MUL_2325 + MUL_2321;
        auto MUL_2355 = ADD_2353 * 2.0;
        auto MUL_2386 = MUL_2355 * 0.11;
        auto MUL_2324 = ADD_579 * SUB_585;
        auto SUB_2340 = MUL_2324 - MUL_2320;
        auto MUL_2342 = SUB_2340 * 2.0;
        auto MUL_2379 = MUL_2342 * 0.037;
        auto SUB_2396 = MUL_2379 - MUL_2368;
        auto SUB_2399 = SUB_2396 - MUL_2386;
        auto ADD_2402 = ADD_564 + SUB_2399;
        auto ADD_2334 = MUL_2324 + MUL_2320;
        auto MUL_2336 = ADD_2334 * 2.0;
        auto MUL_2372 = MUL_2336 * 0.001;
        auto MUL_2323 = SUB_596 * ADD_579;
        auto MUL_2326 = SUB_585 * ADD_591;
        auto SUB_2356 = MUL_2326 - MUL_2323;
        auto MUL_2358 = SUB_2356 * 2.0;
        auto MUL_2390 = MUL_2358 * 0.11;
        auto MUL_2322 = ADD_579 * ADD_579;
        auto ADD_2343 = MUL_2319 + MUL_2322;
        auto MUL_2346 = ADD_2343 * 2.0;
        auto SUB_2349 = 1.0 - MUL_2346;
        auto MUL_2381 = SUB_2349 * 0.037;
        auto SUB_2397 = MUL_2381 - MUL_2372;
        auto SUB_2400 = SUB_2397 - MUL_2390;
        auto MUL_541 = SUB_435 * MUL_527;
        auto MUL_546 = ADD_430 * MUL_514;
        auto MUL_543 = ADD_418 * ADD_522;
        auto ADD_544 = MUL_541 + MUL_543;
        auto ADD_548 = ADD_544 + MUL_546;
        auto MUL_551 = ADD_548 * 2.0;
        auto SUB_554 = 0.384 - MUL_551;
        auto ADD_565 = ADD_404 + SUB_554;
        auto ADD_2403 = ADD_565 + SUB_2400;
        auto SUB_2337 = MUL_2325 - MUL_2321;
        auto ADD_2350 = MUL_2326 + MUL_2323;
        auto ADD_2359 = MUL_2318 + MUL_2322;
        auto MUL_2362 = ADD_2359 * 2.0;
        auto SUB_2365 = 1.0 - MUL_2362;
        auto MUL_2394 = SUB_2365 * 0.11;
        auto MUL_2352 = ADD_2350 * 2.0;
        auto MUL_2383 = MUL_2352 * 0.037;
        auto MUL_2339 = SUB_2337 * 2.0;
        auto MUL_2376 = MUL_2339 * 0.001;
        auto SUB_2398 = MUL_2383 - MUL_2376;
        auto SUB_2401 = SUB_2398 - MUL_2394;
        auto MUL_555 = SUB_435 * ADD_522;
        auto MUL_558 = SUB_424 * MUL_514;
        auto MUL_556 = ADD_418 * MUL_527;
        auto SUB_557 = MUL_555 - MUL_556;
        auto ADD_560 = SUB_557 + MUL_558;
        auto MUL_562 = ADD_560 * 2.0;
        auto ADD_566 = ADD_405 + MUL_562;
        auto ADD_2404 = ADD_566 + SUB_2401;
        auto MUL_2412 = MUL_2342 * 0.055;
        auto ADD_2423 = ADD_564 + MUL_2412;
        auto MUL_2414 = SUB_2349 * 0.055;
        auto ADD_2424 = ADD_565 + MUL_2414;
        auto MUL_2416 = MUL_2352 * 0.055;
        auto ADD_2425 = ADD_566 + MUL_2416;
        auto MUL_2433 = MUL_2342 * 0.075;
        auto ADD_2444 = ADD_564 + MUL_2433;
        auto MUL_2435 = SUB_2349 * 0.075;
        auto ADD_2445 = ADD_565 + MUL_2435;
        auto MUL_2437 = MUL_2352 * 0.075;
        auto ADD_2446 = ADD_566 + MUL_2437;
        auto MUL_2461 = MUL_2355 * 0.22;
        auto SUB_2471 = ADD_564 - MUL_2461;
        auto MUL_2465 = MUL_2358 * 0.22;
        auto SUB_2472 = ADD_565 - MUL_2465;
        auto MUL_2469 = SUB_2365 * 0.22;
        auto SUB_2473 = ADD_566 - MUL_2469;
        auto MUL_2488 = MUL_2355 * 0.18;
        auto MUL_2481 = MUL_2342 * 0.05;
        auto SUB_2498 = MUL_2481 - MUL_2488;
        auto ADD_2501 = ADD_564 + SUB_2498;
        auto MUL_2492 = MUL_2358 * 0.18;
        auto MUL_2483 = SUB_2349 * 0.05;
        auto SUB_2499 = MUL_2483 - MUL_2492;
        auto ADD_2502 = ADD_565 + SUB_2499;
        auto MUL_2496 = SUB_2365 * 0.18;
        auto MUL_2485 = MUL_2352 * 0.05;
        auto SUB_2500 = MUL_2485 - MUL_2496;
        auto ADD_2503 = ADD_566 + SUB_2500;
        auto MUL_2511 = MUL_2342 * 0.08;
        auto MUL_2518 = MUL_2355 * 0.14;
        auto MUL_2505 = SUB_2333 * 0.01;
        auto ADD_2528 = MUL_2505 + MUL_2511;
        auto SUB_2531 = ADD_2528 - MUL_2518;
        auto ADD_2534 = ADD_564 + SUB_2531;
        auto MUL_2522 = MUL_2358 * 0.14;
        auto MUL_2513 = SUB_2349 * 0.08;
        auto MUL_2507 = MUL_2336 * 0.01;
        auto ADD_2529 = MUL_2507 + MUL_2513;
        auto SUB_2532 = ADD_2529 - MUL_2522;
        auto ADD_2535 = ADD_565 + SUB_2532;
        auto MUL_2526 = SUB_2365 * 0.14;
        auto MUL_2515 = MUL_2352 * 0.08;
        auto MUL_2509 = MUL_2339 * 0.01;
        auto ADD_2530 = MUL_2509 + MUL_2515;
        auto SUB_2533 = ADD_2530 - MUL_2526;
        auto ADD_2536 = ADD_566 + SUB_2533;
        auto MUL_2544 = MUL_2342 * 0.085;
        auto ADD_2561 = MUL_2505 + MUL_2544;
        auto SUB_2564 = ADD_2561 - MUL_2386;
        auto ADD_2567 = ADD_564 + SUB_2564;
        auto MUL_2546 = SUB_2349 * 0.085;
        auto ADD_2562 = MUL_2507 + MUL_2546;
        auto SUB_2565 = ADD_2562 - MUL_2390;
        auto ADD_2568 = ADD_565 + SUB_2565;
        auto MUL_2548 = MUL_2352 * 0.085;
        auto ADD_2563 = MUL_2509 + MUL_2548;
        auto SUB_2566 = ADD_2563 - MUL_2394;
        auto ADD_2569 = ADD_566 + SUB_2566;
        auto MUL_2584 = MUL_2355 * 0.08;
        auto MUL_2577 = MUL_2342 * 0.09;
        auto ADD_2594 = MUL_2505 + MUL_2577;
        auto SUB_2597 = ADD_2594 - MUL_2584;
        auto ADD_2600 = ADD_564 + SUB_2597;
        auto MUL_2588 = MUL_2358 * 0.08;
        auto MUL_2579 = SUB_2349 * 0.09;
        auto ADD_2595 = MUL_2507 + MUL_2579;
        auto SUB_2598 = ADD_2595 - MUL_2588;
        auto ADD_2601 = ADD_565 + SUB_2598;
        auto MUL_2592 = SUB_2365 * 0.08;
        auto MUL_2581 = MUL_2352 * 0.09;
        auto ADD_2596 = MUL_2509 + MUL_2581;
        auto SUB_2599 = ADD_2596 - MUL_2592;
        auto ADD_2602 = ADD_566 + SUB_2599;
        auto MUL_2617 = MUL_2355 * 0.05;
        auto MUL_2610 = MUL_2342 * 0.095;
        auto ADD_2627 = MUL_2505 + MUL_2610;
        auto SUB_2630 = ADD_2627 - MUL_2617;
        auto ADD_2633 = ADD_564 + SUB_2630;
        auto MUL_2621 = MUL_2358 * 0.05;
        auto MUL_2612 = SUB_2349 * 0.095;
        auto ADD_2628 = MUL_2507 + MUL_2612;
        auto SUB_2631 = ADD_2628 - MUL_2621;
        auto ADD_2634 = ADD_565 + SUB_2631;
        auto MUL_2625 = SUB_2365 * 0.05;
        auto MUL_2614 = MUL_2352 * 0.095;
        auto ADD_2629 = MUL_2509 + MUL_2614;
        auto SUB_2632 = ADD_2629 - MUL_2625;
        auto ADD_2635 = ADD_566 + SUB_2632;
        auto SUB_2666 = MUL_2511 - MUL_2505;
        auto SUB_2669 = SUB_2666 - MUL_2518;
        auto ADD_2672 = ADD_564 + SUB_2669;
        auto SUB_2667 = MUL_2513 - MUL_2507;
        auto SUB_2670 = SUB_2667 - MUL_2522;
        auto ADD_2673 = ADD_565 + SUB_2670;
        auto SUB_2668 = MUL_2515 - MUL_2509;
        auto SUB_2671 = SUB_2668 - MUL_2526;
        auto ADD_2674 = ADD_566 + SUB_2671;
        auto SUB_2705 = MUL_2544 - MUL_2505;
        auto SUB_2708 = SUB_2705 - MUL_2386;
        auto ADD_2711 = ADD_564 + SUB_2708;
        auto SUB_2706 = MUL_2546 - MUL_2507;
        auto SUB_2709 = SUB_2706 - MUL_2390;
        auto ADD_2712 = ADD_565 + SUB_2709;
        auto SUB_2707 = MUL_2548 - MUL_2509;
        auto SUB_2710 = SUB_2707 - MUL_2394;
        auto ADD_2713 = ADD_566 + SUB_2710;
        auto SUB_2744 = MUL_2577 - MUL_2505;
        auto SUB_2747 = SUB_2744 - MUL_2584;
        auto ADD_2750 = ADD_564 + SUB_2747;
        auto SUB_2745 = MUL_2579 - MUL_2507;
        auto SUB_2748 = SUB_2745 - MUL_2588;
        auto ADD_2751 = ADD_565 + SUB_2748;
        auto SUB_2746 = MUL_2581 - MUL_2509;
        auto SUB_2749 = SUB_2746 - MUL_2592;
        auto ADD_2752 = ADD_566 + SUB_2749;
        auto SUB_2783 = MUL_2610 - MUL_2505;
        auto SUB_2786 = SUB_2783 - MUL_2617;
        auto ADD_2789 = ADD_564 + SUB_2786;
        auto SUB_2784 = MUL_2612 - MUL_2507;
        auto SUB_2787 = SUB_2784 - MUL_2621;
        auto ADD_2790 = ADD_565 + SUB_2787;
        auto SUB_2785 = MUL_2614 - MUL_2509;
        auto SUB_2788 = SUB_2785 - MUL_2625;
        auto ADD_2791 = ADD_566 + SUB_2788;
        if (/*panda_link0 vs. panda_link5*/ sphere_sphere_self_collision(
            0.0, 0.0, 0.05, 0.08, ADD_2402, ADD_2403, ADD_2404, 0.176))
        {
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
        }  // (331, 557)
        if (/*panda_link1 vs. panda_link5*/ sphere_sphere_self_collision(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_2402, ADD_2403, ADD_2404, 0.176))
        {
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
        }  // (557, 557)
        if (/*panda_link2 vs. panda_link5*/ sphere_sphere_self_collision(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_2402, ADD_2403, ADD_2404, 0.176))
        {
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
        }  // (557, 557)
        if (/*panda_link5*/ sphere_environment_in_collision(environment, ADD_2402, ADD_2403, ADD_2404, 0.176))
        {
            if (sphere_environment_in_collision(environment, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
        }  // (557, 557)
        auto MUL_657 = SUB_596 * 0.7071068;
        auto MUL_654 = ADD_591 * 0.7071068;
        auto MUL_651 = SUB_585 * 0.7071068;
        auto SUB_655 = MUL_654 - MUL_651;
        auto ADD_645 = MUL_651 + MUL_654;
        auto MUL_659 = ADD_579 * 0.7071068;
        auto SUB_660 = MUL_657 - MUL_659;
        auto ADD_632 = MUL_657 + MUL_659;
        auto INPUT_5 = q[5];
        auto DIV_697 = INPUT_5 * 0.5;
        auto SIN_698 = sin(DIV_697);
        auto COS_704 = cos(DIV_697);
        auto MUL_716 = SUB_660 * SIN_698;
        auto MUL_721 = SUB_660 * COS_704;
        auto MUL_724 = SUB_655 * SIN_698;
        auto SUB_725 = MUL_721 - MUL_724;
        auto MUL_719 = SUB_655 * COS_704;
        auto ADD_720 = MUL_716 + MUL_719;
        auto MUL_2820 = SUB_725 * ADD_720;
        auto MUL_2819 = ADD_720 * ADD_720;
        auto MUL_711 = ADD_632 * SIN_698;
        auto MUL_706 = ADD_632 * COS_704;
        auto MUL_707 = ADD_645 * SIN_698;
        auto ADD_708 = MUL_706 + MUL_707;
        auto MUL_713 = ADD_645 * COS_704;
        auto SUB_714 = MUL_713 - MUL_711;
        auto MUL_2818 = SUB_714 * SUB_714;
        auto ADD_2827 = MUL_2818 + MUL_2819;
        auto MUL_2830 = ADD_2827 * 2.0;
        auto SUB_2833 = 1.0 - MUL_2830;
        auto MUL_2867 = SUB_2833 * 0.042;
        auto MUL_2824 = ADD_708 * SUB_714;
        auto SUB_2840 = MUL_2824 - MUL_2820;
        auto MUL_2842 = SUB_2840 * 2.0;
        auto MUL_2873 = MUL_2842 * 0.014;
        auto ADD_2884 = MUL_2867 + MUL_2873;
        auto ADD_2887 = ADD_564 + ADD_2884;
        auto ADD_2834 = MUL_2824 + MUL_2820;
        auto MUL_2836 = ADD_2834 * 2.0;
        auto MUL_2869 = MUL_2836 * 0.042;
        auto MUL_2822 = ADD_708 * ADD_708;
        auto ADD_2843 = MUL_2819 + MUL_2822;
        auto MUL_2846 = ADD_2843 * 2.0;
        auto SUB_2849 = 1.0 - MUL_2846;
        auto MUL_2875 = SUB_2849 * 0.014;
        auto ADD_2885 = MUL_2869 + MUL_2875;
        auto ADD_2888 = ADD_565 + ADD_2885;
        auto MUL_2821 = SUB_725 * SUB_714;
        auto MUL_2823 = SUB_725 * ADD_708;
        auto MUL_2826 = SUB_714 * ADD_720;
        auto ADD_2850 = MUL_2826 + MUL_2823;
        auto MUL_2852 = ADD_2850 * 2.0;
        auto MUL_2877 = MUL_2852 * 0.014;
        auto MUL_2825 = ADD_708 * ADD_720;
        auto SUB_2837 = MUL_2825 - MUL_2821;
        auto MUL_2839 = SUB_2837 * 2.0;
        auto MUL_2871 = MUL_2839 * 0.042;
        auto ADD_2886 = MUL_2871 + MUL_2877;
        auto ADD_2889 = ADD_566 + ADD_2886;
        auto MUL_2916 = MUL_2842 * 0.01;
        auto MUL_2909 = SUB_2833 * 0.08;
        auto SUB_2932 = MUL_2909 - MUL_2916;
        auto ADD_2935 = ADD_564 + SUB_2932;
        auto MUL_2920 = SUB_2849 * 0.01;
        auto MUL_2911 = MUL_2836 * 0.08;
        auto SUB_2933 = MUL_2911 - MUL_2920;
        auto ADD_2936 = ADD_565 + SUB_2933;
        auto MUL_2924 = MUL_2852 * 0.01;
        auto MUL_2913 = MUL_2839 * 0.08;
        auto SUB_2934 = MUL_2913 - MUL_2924;
        auto ADD_2937 = ADD_566 + SUB_2934;
        auto MUL_2945 = MUL_2842 * 0.035;
        auto ADD_2956 = MUL_2909 + MUL_2945;
        auto ADD_2959 = ADD_564 + ADD_2956;
        auto MUL_2947 = SUB_2849 * 0.035;
        auto ADD_2957 = MUL_2911 + MUL_2947;
        auto ADD_2960 = ADD_565 + ADD_2957;
        auto MUL_2949 = MUL_2852 * 0.035;
        auto ADD_2958 = MUL_2913 + MUL_2949;
        auto ADD_2961 = ADD_566 + ADD_2958;
        if (/*panda_link0 vs. panda_link6*/ sphere_sphere_self_collision(
            0.0, 0.0, 0.05, 0.08, ADD_2887, ADD_2888, ADD_2889, 0.095))
        {
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
        }  // (557, 637)
        if (/*panda_link1 vs. panda_link6*/ sphere_sphere_self_collision(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_2887, ADD_2888, ADD_2889, 0.095))
        {
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
        }  // (637, 637)
        if (/*panda_link6*/ sphere_environment_in_collision(environment, ADD_2887, ADD_2888, ADD_2889, 0.095))
        {
            if (sphere_environment_in_collision(environment, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
        }  // (637, 637)
        auto MUL_758 = SUB_725 * 0.7071068;
        auto MUL_773 = ADD_720 * 0.7071068;
        auto MUL_771 = SUB_714 * 0.7071068;
        auto SUB_784 = MUL_773 - MUL_771;
        auto ADD_774 = MUL_771 + MUL_773;
        auto MUL_760 = ADD_708 * 0.7071068;
        auto SUB_789 = MUL_758 - MUL_760;
        auto ADD_761 = MUL_758 + MUL_760;
        auto MUL_806 = ADD_720 * 0.088;
        auto MUL_811 = ADD_720 * MUL_806;
        auto MUL_801 = SUB_714 * 0.088;
        auto MUL_809 = SUB_714 * MUL_801;
        auto ADD_813 = MUL_809 + MUL_811;
        auto MUL_816 = ADD_813 * 2.0;
        auto SUB_819 = 0.088 - MUL_816;
        auto ADD_838 = ADD_564 + SUB_819;
        auto INPUT_6 = q[6];
        auto DIV_842 = INPUT_6 * 0.5;
        auto SIN_843 = sin(DIV_842);
        auto COS_849 = cos(DIV_842);
        auto MUL_866 = SUB_789 * COS_849;
        auto MUL_861 = SUB_789 * SIN_843;
        auto MUL_864 = SUB_784 * COS_849;
        auto ADD_865 = MUL_861 + MUL_864;
        auto MUL_2971 = ADD_865 * ADD_865;
        auto MUL_869 = SUB_784 * SIN_843;
        auto SUB_870 = MUL_866 - MUL_869;
        auto MUL_2972 = SUB_870 * ADD_865;
        auto MUL_851 = ADD_761 * COS_849;
        auto MUL_856 = ADD_761 * SIN_843;
        auto MUL_858 = ADD_774 * COS_849;
        auto SUB_859 = MUL_858 - MUL_856;
        auto MUL_2973 = SUB_870 * SUB_859;
        auto MUL_2970 = SUB_859 * SUB_859;
        auto ADD_2979 = MUL_2970 + MUL_2971;
        auto MUL_2982 = ADD_2979 * 2.0;
        auto SUB_2985 = 1.0 - MUL_2982;
        auto MUL_3019 = SUB_2985 * 0.015;
        auto MUL_852 = ADD_774 * SIN_843;
        auto ADD_853 = MUL_851 + MUL_852;
        auto MUL_2977 = ADD_853 * ADD_865;
        auto ADD_3005 = MUL_2977 + MUL_2973;
        auto MUL_3007 = ADD_3005 * 2.0;
        auto MUL_3031 = MUL_3007 * 0.075;
        auto MUL_2976 = ADD_853 * SUB_859;
        auto SUB_2992 = MUL_2976 - MUL_2972;
        auto MUL_2994 = SUB_2992 * 2.0;
        auto MUL_3025 = MUL_2994 * 0.015;
        auto ADD_3036 = MUL_3019 + MUL_3025;
        auto ADD_3039 = ADD_3036 + MUL_3031;
        auto ADD_3042 = ADD_838 + ADD_3039;
        auto ADD_2986 = MUL_2976 + MUL_2972;
        auto MUL_2988 = ADD_2986 * 2.0;
        auto MUL_3021 = MUL_2988 * 0.015;
        auto MUL_2975 = SUB_870 * ADD_853;
        auto MUL_2978 = SUB_859 * ADD_865;
        auto SUB_3008 = MUL_2978 - MUL_2975;
        auto MUL_3010 = SUB_3008 * 2.0;
        auto MUL_3033 = MUL_3010 * 0.075;
        auto MUL_2974 = ADD_853 * ADD_853;
        auto ADD_2995 = MUL_2971 + MUL_2974;
        auto MUL_2998 = ADD_2995 * 2.0;
        auto SUB_3001 = 1.0 - MUL_2998;
        auto MUL_3027 = SUB_3001 * 0.015;
        auto ADD_3037 = MUL_3021 + MUL_3027;
        auto ADD_3040 = ADD_3037 + MUL_3033;
        auto MUL_821 = SUB_725 * MUL_806;
        auto MUL_822 = ADD_708 * MUL_801;
        auto ADD_824 = MUL_821 + MUL_822;
        auto MUL_827 = ADD_824 * 2.0;
        auto ADD_839 = ADD_565 + MUL_827;
        auto ADD_3043 = ADD_839 + ADD_3040;
        auto SUB_2989 = MUL_2977 - MUL_2973;
        auto ADD_3002 = MUL_2978 + MUL_2975;
        auto ADD_3011 = MUL_2970 + MUL_2974;
        auto MUL_3014 = ADD_3011 * 2.0;
        auto SUB_3017 = 1.0 - MUL_3014;
        auto MUL_3035 = SUB_3017 * 0.075;
        auto MUL_3004 = ADD_3002 * 2.0;
        auto MUL_3029 = MUL_3004 * 0.015;
        auto MUL_2991 = SUB_2989 * 2.0;
        auto MUL_3023 = MUL_2991 * 0.015;
        auto ADD_3038 = MUL_3023 + MUL_3029;
        auto ADD_3041 = ADD_3038 + MUL_3035;
        auto MUL_829 = SUB_725 * MUL_801;
        auto MUL_831 = ADD_708 * MUL_806;
        auto SUB_833 = MUL_831 - MUL_829;
        auto MUL_836 = SUB_833 * 2.0;
        auto ADD_840 = ADD_566 + MUL_836;
        auto ADD_3044 = ADD_840 + ADD_3041;
        auto MUL_3058 = MUL_3007 * 0.07;
        auto ADD_3063 = ADD_838 + MUL_3058;
        auto MUL_3060 = MUL_3010 * 0.07;
        auto ADD_3064 = ADD_839 + MUL_3060;
        auto MUL_3062 = SUB_3017 * 0.07;
        auto ADD_3065 = ADD_840 + MUL_3062;
        auto MUL_3079 = MUL_3007 * 0.08;
        auto MUL_3073 = MUL_2994 * 0.04;
        auto MUL_3067 = SUB_2985 * 0.02;
        auto ADD_3084 = MUL_3067 + MUL_3073;
        auto ADD_3087 = ADD_3084 + MUL_3079;
        auto ADD_3090 = ADD_838 + ADD_3087;
        auto MUL_3081 = MUL_3010 * 0.08;
        auto MUL_3075 = SUB_3001 * 0.04;
        auto MUL_3069 = MUL_2988 * 0.02;
        auto ADD_3085 = MUL_3069 + MUL_3075;
        auto ADD_3088 = ADD_3085 + MUL_3081;
        auto ADD_3091 = ADD_839 + ADD_3088;
        auto MUL_3083 = SUB_3017 * 0.08;
        auto MUL_3077 = MUL_3004 * 0.04;
        auto MUL_3071 = MUL_2991 * 0.02;
        auto ADD_3086 = MUL_3071 + MUL_3077;
        auto ADD_3089 = ADD_3086 + MUL_3083;
        auto ADD_3092 = ADD_840 + ADD_3089;
        auto MUL_3100 = MUL_2994 * 0.02;
        auto MUL_3094 = SUB_2985 * 0.04;
        auto ADD_3111 = MUL_3094 + MUL_3100;
        auto ADD_3114 = ADD_3111 + MUL_3079;
        auto ADD_3117 = ADD_838 + ADD_3114;
        auto MUL_3102 = SUB_3001 * 0.02;
        auto MUL_3096 = MUL_2988 * 0.04;
        auto ADD_3112 = MUL_3096 + MUL_3102;
        auto ADD_3115 = ADD_3112 + MUL_3081;
        auto ADD_3118 = ADD_839 + ADD_3115;
        auto MUL_3104 = MUL_3004 * 0.02;
        auto MUL_3098 = MUL_2991 * 0.04;
        auto ADD_3113 = MUL_3098 + MUL_3104;
        auto ADD_3116 = ADD_3113 + MUL_3083;
        auto ADD_3119 = ADD_840 + ADD_3116;
        auto MUL_3133 = MUL_3007 * 0.085;
        auto MUL_3127 = MUL_2994 * 0.06;
        auto ADD_3138 = MUL_3094 + MUL_3127;
        auto ADD_3141 = ADD_3138 + MUL_3133;
        auto ADD_3144 = ADD_838 + ADD_3141;
        auto MUL_3135 = MUL_3010 * 0.085;
        auto MUL_3129 = SUB_3001 * 0.06;
        auto ADD_3139 = MUL_3096 + MUL_3129;
        auto ADD_3142 = ADD_3139 + MUL_3135;
        auto ADD_3145 = ADD_839 + ADD_3142;
        auto MUL_3137 = SUB_3017 * 0.085;
        auto MUL_3131 = MUL_3004 * 0.06;
        auto ADD_3140 = MUL_3098 + MUL_3131;
        auto ADD_3143 = ADD_3140 + MUL_3137;
        auto ADD_3146 = ADD_840 + ADD_3143;
        auto MUL_3148 = SUB_2985 * 0.06;
        auto ADD_3165 = MUL_3148 + MUL_3073;
        auto ADD_3168 = ADD_3165 + MUL_3133;
        auto ADD_3171 = ADD_838 + ADD_3168;
        auto MUL_3150 = MUL_2988 * 0.06;
        auto ADD_3166 = MUL_3150 + MUL_3075;
        auto ADD_3169 = ADD_3166 + MUL_3135;
        auto ADD_3172 = ADD_839 + ADD_3169;
        auto MUL_3152 = MUL_2991 * 0.06;
        auto ADD_3167 = MUL_3152 + MUL_3077;
        auto ADD_3170 = ADD_3167 + MUL_3137;
        auto ADD_3173 = ADD_840 + ADD_3170;
        if (/*panda_link0 vs. panda_link7*/ sphere_sphere_self_collision(
            0.0, 0.0, 0.05, 0.08, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (637, 793)
        if (/*panda_link1 vs. panda_link7*/ sphere_sphere_self_collision(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        if (/*panda_link2 vs. panda_link7*/ sphere_sphere_self_collision(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        if (/*panda_link5 vs. panda_link7*/ sphere_sphere_self_collision(
            ADD_2402, ADD_2403, ADD_2404, 0.176, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        if (/*panda_link7*/ sphere_environment_in_collision(environment, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_environment_in_collision(environment, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        auto MUL_1065 = SUB_870 * 0.9238795;
        auto MUL_1062 = ADD_865 * 0.9238795;
        auto MUL_1049 = SUB_859 * 0.9238795;
        auto MUL_1034 = ADD_853 * 0.9238795;
        auto MUL_1055 = SUB_870 * 0.3826834;
        auto SUB_1063 = MUL_1062 - MUL_1055;
        auto MUL_1072 = ADD_865 * 0.3826834;
        auto ADD_1074 = MUL_1065 + MUL_1072;
        auto MUL_1037 = SUB_859 * 0.3826834;
        auto SUB_1039 = MUL_1034 - MUL_1037;
        auto MUL_3241 = SUB_1039 * SUB_1063;
        auto MUL_1046 = ADD_853 * 0.3826834;
        auto ADD_1050 = MUL_1046 + MUL_1049;
        auto MUL_3237 = ADD_1074 * ADD_1050;
        auto ADD_3269 = MUL_3241 + MUL_3237;
        auto MUL_3271 = ADD_3269 * 2.0;
        auto MUL_931 = SUB_859 * 0.107;
        auto MUL_942 = SUB_870 * MUL_931;
        auto MUL_939 = ADD_853 * 0.107;
        auto MUL_944 = ADD_865 * MUL_939;
        auto ADD_945 = MUL_942 + MUL_944;
        auto MUL_947 = ADD_945 * 2.0;
        auto ADD_969 = ADD_838 + MUL_947;
        auto MUL_3295 = MUL_3271 * 0.022;
        auto ADD_3300 = ADD_969 + MUL_3295;
        auto MUL_3239 = ADD_1074 * SUB_1039;
        auto MUL_3242 = ADD_1050 * SUB_1063;
        auto SUB_3272 = MUL_3242 - MUL_3239;
        auto MUL_3274 = SUB_3272 * 2.0;
        auto MUL_3297 = MUL_3274 * 0.022;
        auto MUL_950 = SUB_870 * MUL_939;
        auto MUL_953 = ADD_865 * MUL_931;
        auto SUB_954 = MUL_953 - MUL_950;
        auto MUL_956 = SUB_954 * 2.0;
        auto ADD_970 = ADD_839 + MUL_956;
        auto ADD_3301 = ADD_970 + MUL_3297;
        auto MUL_3238 = SUB_1039 * SUB_1039;
        auto MUL_3234 = ADD_1050 * ADD_1050;
        auto ADD_3275 = MUL_3234 + MUL_3238;
        auto MUL_3278 = ADD_3275 * 2.0;
        auto SUB_3281 = 1.0 - MUL_3278;
        auto MUL_3299 = SUB_3281 * 0.022;
        auto MUL_961 = SUB_859 * MUL_931;
        auto MUL_959 = ADD_853 * MUL_939;
        auto ADD_962 = MUL_959 + MUL_961;
        auto MUL_965 = ADD_962 * 2.0;
        auto SUB_968 = 0.107 - MUL_965;
        auto ADD_971 = ADD_840 + SUB_968;
        auto ADD_3302 = ADD_971 + MUL_3299;
        auto MUL_3322 = MUL_3271 * 0.01;
        auto MUL_3236 = ADD_1074 * SUB_1063;
        auto MUL_3240 = SUB_1039 * ADD_1050;
        auto SUB_3256 = MUL_3240 - MUL_3236;
        auto MUL_3258 = SUB_3256 * 2.0;
        auto MUL_3311 = MUL_3258 * 0.075;
        auto SUB_3327 = MUL_3322 - MUL_3311;
        auto ADD_3330 = ADD_969 + SUB_3327;
        auto MUL_3324 = MUL_3274 * 0.01;
        auto MUL_3235 = SUB_1063 * SUB_1063;
        auto ADD_3259 = MUL_3235 + MUL_3238;
        auto MUL_3262 = ADD_3259 * 2.0;
        auto SUB_3265 = 1.0 - MUL_3262;
        auto MUL_3315 = SUB_3265 * 0.075;
        auto SUB_3328 = MUL_3324 - MUL_3315;
        auto ADD_3331 = ADD_970 + SUB_3328;
        auto ADD_3266 = MUL_3242 + MUL_3239;
        auto MUL_3326 = SUB_3281 * 0.01;
        auto MUL_3268 = ADD_3266 * 2.0;
        auto MUL_3319 = MUL_3268 * 0.075;
        auto SUB_3329 = MUL_3326 - MUL_3319;
        auto ADD_3332 = ADD_971 + SUB_3329;
        auto MUL_3341 = MUL_3258 * 0.045;
        auto SUB_3357 = MUL_3322 - MUL_3341;
        auto ADD_3360 = ADD_969 + SUB_3357;
        auto MUL_3345 = SUB_3265 * 0.045;
        auto SUB_3358 = MUL_3324 - MUL_3345;
        auto ADD_3361 = ADD_970 + SUB_3358;
        auto MUL_3349 = MUL_3268 * 0.045;
        auto SUB_3359 = MUL_3326 - MUL_3349;
        auto ADD_3362 = ADD_971 + SUB_3359;
        auto MUL_3371 = MUL_3258 * 0.015;
        auto SUB_3387 = MUL_3322 - MUL_3371;
        auto ADD_3390 = ADD_969 + SUB_3387;
        auto MUL_3375 = SUB_3265 * 0.015;
        auto SUB_3388 = MUL_3324 - MUL_3375;
        auto ADD_3391 = ADD_970 + SUB_3388;
        auto MUL_3379 = MUL_3268 * 0.015;
        auto SUB_3389 = MUL_3326 - MUL_3379;
        auto ADD_3392 = ADD_971 + SUB_3389;
        auto ADD_3411 = MUL_3371 + MUL_3322;
        auto ADD_3414 = ADD_969 + ADD_3411;
        auto ADD_3412 = MUL_3375 + MUL_3324;
        auto ADD_3415 = ADD_970 + ADD_3412;
        auto ADD_3413 = MUL_3379 + MUL_3326;
        auto ADD_3416 = ADD_971 + ADD_3413;
        auto ADD_3435 = MUL_3341 + MUL_3322;
        auto ADD_3438 = ADD_969 + ADD_3435;
        auto ADD_3436 = MUL_3345 + MUL_3324;
        auto ADD_3439 = ADD_970 + ADD_3436;
        auto ADD_3437 = MUL_3349 + MUL_3326;
        auto ADD_3440 = ADD_971 + ADD_3437;
        auto ADD_3459 = MUL_3311 + MUL_3322;
        auto ADD_3462 = ADD_969 + ADD_3459;
        auto ADD_3460 = MUL_3315 + MUL_3324;
        auto ADD_3463 = ADD_970 + ADD_3460;
        auto ADD_3461 = MUL_3319 + MUL_3326;
        auto ADD_3464 = ADD_971 + ADD_3461;
        auto MUL_3484 = MUL_3271 * 0.03;
        auto SUB_3489 = MUL_3484 - MUL_3311;
        auto ADD_3492 = ADD_969 + SUB_3489;
        auto MUL_3486 = MUL_3274 * 0.03;
        auto SUB_3490 = MUL_3486 - MUL_3315;
        auto ADD_3493 = ADD_970 + SUB_3490;
        auto MUL_3488 = SUB_3281 * 0.03;
        auto SUB_3491 = MUL_3488 - MUL_3319;
        auto ADD_3494 = ADD_971 + SUB_3491;
        auto SUB_3519 = MUL_3484 - MUL_3341;
        auto ADD_3522 = ADD_969 + SUB_3519;
        auto SUB_3520 = MUL_3486 - MUL_3345;
        auto ADD_3523 = ADD_970 + SUB_3520;
        auto SUB_3521 = MUL_3488 - MUL_3349;
        auto ADD_3524 = ADD_971 + SUB_3521;
        auto SUB_3549 = MUL_3484 - MUL_3371;
        auto ADD_3552 = ADD_969 + SUB_3549;
        auto SUB_3550 = MUL_3486 - MUL_3375;
        auto ADD_3553 = ADD_970 + SUB_3550;
        auto SUB_3551 = MUL_3488 - MUL_3379;
        auto ADD_3554 = ADD_971 + SUB_3551;
        auto ADD_3573 = MUL_3371 + MUL_3484;
        auto ADD_3576 = ADD_969 + ADD_3573;
        auto ADD_3574 = MUL_3375 + MUL_3486;
        auto ADD_3577 = ADD_970 + ADD_3574;
        auto ADD_3575 = MUL_3379 + MUL_3488;
        auto ADD_3578 = ADD_971 + ADD_3575;
        auto ADD_3597 = MUL_3341 + MUL_3484;
        auto ADD_3600 = ADD_969 + ADD_3597;
        auto ADD_3598 = MUL_3345 + MUL_3486;
        auto ADD_3601 = ADD_970 + ADD_3598;
        auto ADD_3599 = MUL_3349 + MUL_3488;
        auto ADD_3602 = ADD_971 + ADD_3599;
        auto ADD_3621 = MUL_3311 + MUL_3484;
        auto ADD_3624 = ADD_969 + ADD_3621;
        auto ADD_3622 = MUL_3315 + MUL_3486;
        auto ADD_3625 = ADD_970 + ADD_3622;
        auto ADD_3623 = MUL_3319 + MUL_3488;
        auto ADD_3626 = ADD_971 + ADD_3623;
        auto MUL_3646 = MUL_3271 * 0.05;
        auto SUB_3651 = MUL_3646 - MUL_3311;
        auto ADD_3654 = ADD_969 + SUB_3651;
        auto MUL_3648 = MUL_3274 * 0.05;
        auto SUB_3652 = MUL_3648 - MUL_3315;
        auto ADD_3655 = ADD_970 + SUB_3652;
        auto MUL_3650 = SUB_3281 * 0.05;
        auto SUB_3653 = MUL_3650 - MUL_3319;
        auto ADD_3656 = ADD_971 + SUB_3653;
        auto SUB_3681 = MUL_3646 - MUL_3341;
        auto ADD_3684 = ADD_969 + SUB_3681;
        auto SUB_3682 = MUL_3648 - MUL_3345;
        auto ADD_3685 = ADD_970 + SUB_3682;
        auto SUB_3683 = MUL_3650 - MUL_3349;
        auto ADD_3686 = ADD_971 + SUB_3683;
        auto SUB_3711 = MUL_3646 - MUL_3371;
        auto ADD_3714 = ADD_969 + SUB_3711;
        auto SUB_3712 = MUL_3648 - MUL_3375;
        auto ADD_3715 = ADD_970 + SUB_3712;
        auto SUB_3713 = MUL_3650 - MUL_3379;
        auto ADD_3716 = ADD_971 + SUB_3713;
        auto ADD_3735 = MUL_3371 + MUL_3646;
        auto ADD_3738 = ADD_969 + ADD_3735;
        auto ADD_3736 = MUL_3375 + MUL_3648;
        auto ADD_3739 = ADD_970 + ADD_3736;
        auto ADD_3737 = MUL_3379 + MUL_3650;
        auto ADD_3740 = ADD_971 + ADD_3737;
        auto ADD_3759 = MUL_3341 + MUL_3646;
        auto ADD_3762 = ADD_969 + ADD_3759;
        auto ADD_3760 = MUL_3345 + MUL_3648;
        auto ADD_3763 = ADD_970 + ADD_3760;
        auto ADD_3761 = MUL_3349 + MUL_3650;
        auto ADD_3764 = ADD_971 + ADD_3761;
        auto ADD_3783 = MUL_3311 + MUL_3646;
        auto ADD_3786 = ADD_969 + ADD_3783;
        auto ADD_3784 = MUL_3315 + MUL_3648;
        auto ADD_3787 = ADD_970 + ADD_3784;
        auto ADD_3785 = MUL_3319 + MUL_3650;
        auto ADD_3788 = ADD_971 + ADD_3785;
        if (/*panda_hand*/ sphere_environment_in_collision(environment, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_environment_in_collision(environment, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            // printf("about to check panda hand collision:\n");
            if (sphere_environment_in_collision(environment, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                // printf("panda hand collision!");
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (793, 978)
        if (/*panda_link0 vs. panda_hand*/ sphere_sphere_self_collision(
            0.0, 0.0, 0.05, 0.08, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (978, 978)
        if (/*panda_link1 vs. panda_hand*/ sphere_sphere_self_collision(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (978, 978)
        if (/*panda_link2 vs. panda_hand*/ sphere_sphere_self_collision(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (978, 978)
        if (/*panda_link5 vs. panda_hand*/ sphere_sphere_self_collision(
            ADD_2402, ADD_2403, ADD_2404, 0.176, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (978, 978)
        auto MUL_3864 = ADD_3269 * 2.0;
        auto MUL_3851 = SUB_3256 * 2.0;
        auto MUL_1196 = SUB_1063 * 0.065;
        auto MUL_1199 = SUB_1039 * 0.065;
        auto MUL_1207 = ADD_1050 * MUL_1199;
        auto MUL_1203 = SUB_1039 * 0.0584;
        auto MUL_1209 = SUB_1063 * MUL_1203;
        auto MUL_1194 = ADD_1050 * 0.0584;
        auto SUB_1197 = MUL_1194 - MUL_1196;
        auto MUL_1206 = ADD_1074 * SUB_1197;
        auto ADD_1208 = MUL_1206 + MUL_1207;
        auto ADD_1210 = ADD_1208 + MUL_1209;
        auto MUL_1212 = ADD_1210 * 2.0;
        auto ADD_1235 = ADD_969 + MUL_1212;
        auto MUL_3888 = MUL_3864 * 0.033;
        auto MUL_3882 = MUL_3851 * 0.012;
        auto ADD_3893 = MUL_3882 + MUL_3888;
        auto ADD_3896 = ADD_1235 + ADD_3893;
        auto MUL_3867 = SUB_3272 * 2.0;
        auto MUL_3890 = MUL_3867 * 0.033;
        auto MUL_3855 = ADD_3259 * 2.0;
        auto SUB_3858 = 1.0 - MUL_3855;
        auto MUL_3884 = SUB_3858 * 0.012;
        auto ADD_3894 = MUL_3884 + MUL_3890;
        auto MUL_1215 = ADD_1074 * MUL_1203;
        auto MUL_1220 = SUB_1063 * SUB_1197;
        auto MUL_1217 = SUB_1039 * MUL_1199;
        auto ADD_1218 = MUL_1215 + MUL_1217;
        auto SUB_1221 = MUL_1220 - ADD_1218;
        auto MUL_1223 = SUB_1221 * 2.0;
        auto ADD_1225 = MUL_1223 + 0.065;
        auto ADD_1236 = ADD_970 + ADD_1225;
        auto ADD_3897 = ADD_1236 + ADD_3894;
        auto MUL_3871 = ADD_3275 * 2.0;
        auto SUB_3874 = 1.0 - MUL_3871;
        auto MUL_3892 = SUB_3874 * 0.033;
        auto MUL_3861 = ADD_3266 * 2.0;
        auto MUL_3886 = MUL_3861 * 0.012;
        auto ADD_3895 = MUL_3886 + MUL_3892;
        auto MUL_1226 = ADD_1074 * MUL_1199;
        auto MUL_1227 = SUB_1039 * MUL_1203;
        auto SUB_1228 = MUL_1226 - MUL_1227;
        auto MUL_1229 = ADD_1050 * SUB_1197;
        auto SUB_1230 = SUB_1228 - MUL_1229;
        auto MUL_1232 = SUB_1230 * 2.0;
        auto ADD_1234 = MUL_1232 + 0.0584;
        auto ADD_1237 = ADD_971 + ADD_1234;
        auto ADD_3898 = ADD_1237 + ADD_3895;
        auto MUL_3912 = MUL_3864 * 0.022;
        auto MUL_3906 = MUL_3851 * 0.015;
        auto ADD_3917 = MUL_3906 + MUL_3912;
        auto ADD_3920 = ADD_1235 + ADD_3917;
        auto MUL_3914 = MUL_3867 * 0.022;
        auto MUL_3908 = SUB_3858 * 0.015;
        auto ADD_3918 = MUL_3908 + MUL_3914;
        auto ADD_3921 = ADD_1236 + ADD_3918;
        auto MUL_3916 = SUB_3874 * 0.022;
        auto MUL_3910 = MUL_3861 * 0.015;
        auto ADD_3919 = MUL_3910 + MUL_3916;
        auto ADD_3922 = ADD_1237 + ADD_3919;
        auto MUL_3936 = MUL_3864 * 0.044;
        auto MUL_3930 = MUL_3851 * 0.008;
        auto ADD_3941 = MUL_3930 + MUL_3936;
        auto ADD_3944 = ADD_1235 + ADD_3941;
        auto MUL_3938 = MUL_3867 * 0.044;
        auto MUL_3932 = SUB_3858 * 0.008;
        auto ADD_3942 = MUL_3932 + MUL_3938;
        auto ADD_3945 = ADD_1236 + ADD_3942;
        auto MUL_3940 = SUB_3874 * 0.044;
        auto MUL_3934 = MUL_3861 * 0.008;
        auto ADD_3943 = MUL_3934 + MUL_3940;
        auto ADD_3946 = ADD_1237 + ADD_3943;
        if (/*panda_leftfinger*/ sphere_environment_in_collision(
            environment, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_environment_in_collision(environment, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (978, 1050)
        if (/*panda_link0 vs. panda_leftfinger*/ sphere_sphere_self_collision(
            0.0, 0.0, 0.05, 0.08, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (1050, 1050)
        if (/*panda_link1 vs. panda_leftfinger*/ sphere_sphere_self_collision(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (1050, 1050)
        if (/*panda_link2 vs. panda_leftfinger*/ sphere_sphere_self_collision(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (1050, 1050)
        if (/*panda_link5 vs. panda_leftfinger*/ sphere_sphere_self_collision(
            ADD_2402, ADD_2403, ADD_2404, 0.176, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (1050, 1050)
        auto ADD_1331 = MUL_1194 + MUL_1196;
        auto MUL_3990 = ADD_3269 * 2.0;
        auto MUL_4020 = MUL_3990 * 0.033;
        auto MUL_3977 = SUB_3256 * 2.0;
        auto MUL_4009 = MUL_3977 * 0.012;
        auto SUB_4025 = MUL_4020 - MUL_4009;
        auto MUL_1342 = ADD_1074 * ADD_1331;
        auto SUB_1345 = MUL_1342 - MUL_1207;
        auto ADD_1347 = SUB_1345 + MUL_1209;
        auto MUL_1349 = ADD_1347 * 2.0;
        auto ADD_1377 = ADD_969 + MUL_1349;
        auto ADD_4028 = ADD_1377 + SUB_4025;
        auto SUB_1356 = MUL_1217 - MUL_1215;
        auto MUL_3993 = SUB_3272 * 2.0;
        auto MUL_4022 = MUL_3993 * 0.033;
        auto MUL_3981 = ADD_3259 * 2.0;
        auto SUB_3984 = 1.0 - MUL_3981;
        auto MUL_4013 = SUB_3984 * 0.012;
        auto SUB_4026 = MUL_4022 - MUL_4013;
        auto MUL_1357 = SUB_1063 * ADD_1331;
        auto ADD_1358 = SUB_1356 + MUL_1357;
        auto MUL_1360 = ADD_1358 * 2.0;
        auto SUB_1363 = MUL_1360 - 0.065;
        auto ADD_1378 = ADD_970 + SUB_1363;
        auto ADD_4029 = ADD_1378 + SUB_4026;
        auto ADD_1367 = MUL_1226 + MUL_1227;
        auto MUL_3997 = ADD_3275 * 2.0;
        auto SUB_4000 = 1.0 - MUL_3997;
        auto MUL_4024 = SUB_4000 * 0.033;
        auto MUL_3987 = ADD_3266 * 2.0;
        auto MUL_4017 = MUL_3987 * 0.012;
        auto SUB_4027 = MUL_4024 - MUL_4017;
        auto MUL_1369 = ADD_1050 * ADD_1331;
        auto ADD_1370 = ADD_1367 + MUL_1369;
        auto MUL_1373 = ADD_1370 * 2.0;
        auto SUB_1376 = 0.0584 - MUL_1373;
        auto ADD_1379 = ADD_971 + SUB_1376;
        auto ADD_4030 = ADD_1379 + SUB_4027;
        auto MUL_4050 = MUL_3990 * 0.022;
        auto MUL_4039 = MUL_3977 * 0.015;
        auto SUB_4055 = MUL_4050 - MUL_4039;
        auto ADD_4058 = ADD_1377 + SUB_4055;
        auto MUL_4052 = MUL_3993 * 0.022;
        auto MUL_4043 = SUB_3984 * 0.015;
        auto SUB_4056 = MUL_4052 - MUL_4043;
        auto ADD_4059 = ADD_1378 + SUB_4056;
        auto MUL_4054 = SUB_4000 * 0.022;
        auto MUL_4047 = MUL_3987 * 0.015;
        auto SUB_4057 = MUL_4054 - MUL_4047;
        auto ADD_4060 = ADD_1379 + SUB_4057;
        auto MUL_4080 = MUL_3990 * 0.044;
        auto MUL_4069 = MUL_3977 * 0.008;
        auto SUB_4085 = MUL_4080 - MUL_4069;
        auto ADD_4088 = ADD_1377 + SUB_4085;
        auto MUL_4082 = MUL_3993 * 0.044;
        auto MUL_4073 = SUB_3984 * 0.008;
        auto SUB_4086 = MUL_4082 - MUL_4073;
        auto ADD_4089 = ADD_1378 + SUB_4086;
        auto MUL_4084 = SUB_4000 * 0.044;
        auto MUL_4077 = MUL_3987 * 0.008;
        auto SUB_4087 = MUL_4084 - MUL_4077;
        auto ADD_4090 = ADD_1379 + SUB_4087;
        if (/*panda_link0 vs. panda_rightfinger*/ sphere_sphere_self_collision(
            0.0, 0.0, 0.05, 0.08, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.05, 0.08, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1050, 1112)
        if (/*panda_link1 vs. panda_rightfinger*/ sphere_sphere_self_collision(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.213, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.0, 0.0, 0.163, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1112, 1112)
        if (/*panda_link2 vs. panda_rightfinger*/ sphere_sphere_self_collision(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1112, 1112)
        if (/*panda_link5 vs. panda_rightfinger*/ sphere_sphere_self_collision(
            ADD_2402, ADD_2403, ADD_2404, 0.176, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1112, 1112)
        if (/*panda_rightfinger*/ sphere_environment_in_collision(
            environment, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_environment_in_collision(environment, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1112, 1112)

        return true;
    }
}