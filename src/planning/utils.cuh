#pragma once

#include "Robots.hh"
#include "src/collision/environment.hh"
#include "src/collision/shapes.hh"

#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <iostream>
#include <cassert>

/* All device utils and collision functions */

namespace ppln::device_utils {
    using namespace collision;

    /* math utils */
    __device__ __forceinline__ constexpr float dot_2(const float &ax, const float &ay, const float &bx, const float &by) 
    {
        return (ax * bx) + (ay * by);
    }

     __device__ __forceinline__ constexpr float dot_3(
    const float ax,
    const float ay,
    const float az,
    const float bx,
    const float by,
    const float bz)
    {
        return ax * bx + ay * by + az * bz;
    }

    __device__ __forceinline__ constexpr float sql2_3(
        const float &ax,
        const float &ay,
        const float &az,
        const float &bx,
        const float &by,
        const float &bz)
    {
        const float xs = (ax - bx);
        const float ys = (ay - by);
        const float zs = (az - bz);

        return dot_3(xs, ys, zs, xs, ys, zs);
    }

    __device__ __forceinline__ constexpr float clamp(const float &v, const float &lower, const float &upper) 
    {
        return fmaxf(fminf(v, upper), lower);
    }
    /* end math utils */


    /* Sphere collision utils*/
    __device__ __forceinline__ constexpr float sphere_sphere_sql2(
        const float ax,
        const float ay,
        const float az,
        const float ar,
        const float bx,
        const float by,
        const float bz,
        const float br)
    {
        float sum = sql2_3(ax, ay, az, bx, by, bz);
        float rs = ar + br;
        return sum - rs * rs;
    }

    __device__ __forceinline__ constexpr float sphere_sphere_sql2(
        const Sphere<float> &a,
        const float &x,
        const float &y,
        const float &z,
        const float &r) 
    {
        return sphere_sphere_sql2(a.x, a.y, a.z, a.r, x, y, z, r);
    }

    __device__ __forceinline__ constexpr float sphere_sphere_self_collision(float ax, float ay, float az, float ar, float bx, float by, float bz, float br)
    {
        return (sphere_sphere_sql2(ax, ay, az, ar, bx, by, bz, br) < 0);
    }

    // returns l2 distance between two configs
    __device__ __forceinline__ float l2_dist(float *config_a, float *config_b, const int dim) {
        float ans = 0;
        float diff;
        for (int i = 0; i < dim; i++) {
            diff = config_a[i] - config_b[i];
            ans += diff * diff;
        }
        return sqrt(ans);
    }

    __device__ __forceinline__ float sq_l2_dist(float *config_a, float *config_b, const int dim) {
        float ans = 0;
        float diff;
        for (int i = 0; i < dim; i++) {
            diff = config_a[i] - config_b[i];
            ans += diff * diff;
        }
        return ans;
    }
    /* End Sphere collision utils*/

    /* Capsule collision utils */
    __device__ __forceinline__ constexpr float sphere_capsule(
        const Capsule<float> &c,
        const float &x,
        const float &y,
        const float &z,
        const float &r) noexcept
    {
        float dot = dot_3(x - c.x1, y - c.y1, z - c.z1, c.xv, c.yv, c.zv);
        float cdf = clamp((dot * c.rdv), 0.F, 1.F);

        float sum = sql2_3(x, y, z, c.x1 + c.xv * cdf, c.y1 + c.yv * cdf, c.z1 + c.zv * cdf);
        float rs = r + c.r;
        return sum - rs * rs;
    }

    
    __device__ __forceinline__ constexpr float sphere_capsule(const Capsule<float> &c, const Sphere<float> &s) noexcept 
    {
        return sphere_capsule(c, s.x, s.y, s.z, s.r);
    }

    
    __device__ __forceinline__ constexpr float sphere_z_aligned_capsule(
        const Capsule<float> &c,
        const float &x,
        const float &y,
        const float &z,
        const float &r) noexcept
    {
        float dot = (z - c.z1) * c.zv;
        float cdf = clamp((dot * c.rdv), 0.F, 1.F);

        float sum = sql2_3(x, y, z, c.x1, c.y1, c.z1 + c.zv * cdf);
        float rs = r + c.r;
        return sum - rs * rs;
    }

    __device__ __forceinline__ constexpr float sphere_z_aligned_capsule(const Capsule<float> &c, const Sphere<float> &s) noexcept
        
    {
        return sphere_z_aligned_capsule(c, s.x, s.y, s.z, s.r);
    }
    /* End Capsule collision utils*/

    /* Cuboid collision utils*/
    __device__ __forceinline__ constexpr float sphere_cuboid(
        const Cuboid<float> &c,
        const float &x,
        const float &y,
        const float &z,
        const float &rsq) noexcept
    {
        float xs = x - c.x;
        float ys = y - c.y;
        float zs = z - c.z;

        float a1 = fmaxf(0., abs(dot_3(c.axis_1_x, c.axis_1_y, c.axis_1_z, xs, ys, zs)) - c.axis_1_r);
        float a2 = fmaxf(0., abs(dot_3(c.axis_2_x, c.axis_2_y, c.axis_2_z, xs, ys, zs)) - c.axis_2_r);
        float a3 = fmaxf(0., abs(dot_3(c.axis_3_x, c.axis_3_y, c.axis_3_z, xs, ys, zs)) - c.axis_3_r);

        float sum = dot_3(a1, a2, a3, a1, a2, a3);
        return sum - rsq;
    }

    
    __device__ __forceinline__ constexpr float sphere_cuboid(const Cuboid<float> &c, const Sphere<float> &s) noexcept 
    {
        return sphere_cuboid(c, s.x, s.y, s.z, s.r * s.r);
    }

    
    __device__ __forceinline__ constexpr float sphere_z_aligned_cuboid(
        const Cuboid<float> &c,
        const float &x,
        const float &y,
        const float &z,
        const float &rsq) noexcept
    {
        float xs = x - c.x;
        float ys = y - c.y;
        float zs = z - c.z;

        float a1 = fmaxf(0., (abs(dot_2(c.axis_1_x, c.axis_1_y, xs, ys)) - c.axis_1_r));
        float a2 = fmaxf(0., (abs(dot_2(c.axis_2_x, c.axis_2_y, xs, ys)) - c.axis_2_r));
        float a3 = fmaxf(0, (abs(zs) - c.axis_3_r));

        float sum = dot_3(a1, a2, a3, a1, a2, a3);
        return sum - rsq;
    }

    
    __device__ __forceinline__ constexpr float sphere_z_aligned_cuboid(const Cuboid<float> &c, const Sphere<float> &s) noexcept
        
    {
        return sphere_z_aligned_cuboid(c, s.x, s.y, s.z, s.r * s.r);
    }
    /* End Cuboid collision util*/

    __device__ __forceinline__ bool sphere_environment_in_collision(ppln::collision::Environment<float> *env, float sx_, float sy_, float sz_, float sr_)
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


/* Collision checking backend implementations for different robots */
namespace ppln::collision {
    using namespace device_utils;
    
    // fkcc -> checks if the config is "good"
    // returns false if the config does collide with an obstacle, returns true if the config does not collide

    template <typename Robot>
    __device__ __forceinline__ bool fkcc(volatile float *config, ppln::collision::Environment<float> *env, float var_cache[256][10], int tid);


    template <>
    __device__ __forceinline__ bool fkcc<ppln::robots::Sphere>(volatile float *config, ppln::collision::Environment<float> *env, float var_cache[256][10], int tid)
    {
        return not sphere_environment_in_collision(env, config[0], config[1], config[2], ppln::robots::Sphere::radius);
    }

    template <>
    __device__  __forceinline__ bool fkcc<ppln::robots::Panda>(volatile float *q, ppln::collision::Environment<float> *environment, float var_cache[256][10], int tid)
    {
        // Ignore static frame collisions - needed for some evaluation problems
        // if (/*panda_link0*/ sphere_environment_in_collision(environment, 0.0, 0.0, 0.05, 0.08))
        // {
        //     return false;
        // }  // (0, 0)
        float INPUT_0 = q[0];
        float DIV_8 = INPUT_0 * 0.5;
        float SIN_9 = sin(DIV_8);
        float COS_15 = cos(DIV_8);
        float MUL_1575 = COS_15 * SIN_9;
        float MUL_1594 = MUL_1575 * 2.0;
        float MUL_1625 = MUL_1594 * 0.039;
        float MUL_1574 = SIN_9 * SIN_9;
        float MUL_1584 = MUL_1574 * 2.0;
        float SUB_1587 = 1.0 - MUL_1584;
        float MUL_1614 = SUB_1587 * 0.001;
        float SUB_1641 = MUL_1625 - MUL_1614; // intermediate
        float MUL_1628 = SUB_1587 * 0.039;
        float MUL_1618 = MUL_1594 * 0.001;
        float ADD_1642 = MUL_1618 + MUL_1628;
        float NEGATE_1643 = -ADD_1642; // intermediate
        float MUL_1656 = MUL_1594 * 0.08; // intermediate
        float MUL_1659 = SUB_1587 * 0.08;
        float NEGATE_1660 = -MUL_1659; // intermediate
        float MUL_1680 = MUL_1594 * 0.03;  // intermediate
        float MUL_1683 = SUB_1587 * 0.03;
        float NEGATE_1684 = -MUL_1683; // intermediate
        var_cache[tid][0] = SUB_1641;
        var_cache[tid][1] = NEGATE_1643;
        var_cache[tid][2] = MUL_1656;
        var_cache[tid][3] = NEGATE_1660;
        var_cache[tid][4] = MUL_1680;
        var_cache[tid][5] = NEGATE_1684;
        if (/*panda_link1*/ sphere_environment_in_collision(environment, var_cache[tid][0], var_cache[tid][1], 0.248, 0.154))
        {
            if (sphere_environment_in_collision(environment, var_cache[tid][2], var_cache[tid][3], 0.333, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, var_cache[tid][4], var_cache[tid][5], 0.333, 0.06))
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
        float MUL_74 = COS_15 * 0.7071068;
        float MUL_72 = SIN_9 * 0.7071068;
        float INPUT_1 = q[1];
        float DIV_117 = INPUT_1 * 0.5;
        float SIN_118 = sin(DIV_117);
        float COS_124 = cos(DIV_117);
        float MUL_143 = MUL_72 * COS_124;
        float MUL_128 = MUL_72 * SIN_118;
        float MUL_126 = MUL_74 * COS_124;
        float SUB_149 = MUL_126 - MUL_128;
        float ADD_130 = MUL_126 + MUL_128;
        float MUL_140 = MUL_74 * SIN_118;
        float SUB_138 = MUL_140 - MUL_143;
        float ADD_144 = MUL_140 + MUL_143;
        float MUL_1756 = SUB_149 * ADD_144;
        float MUL_1757 = SUB_149 * SUB_138;
        float MUL_1763 = ADD_130 * ADD_144;
        float SUB_1796 = MUL_1757 - MUL_1763;
        float MUL_1798 = SUB_1796 * 2.0;
        float MUL_1827 = MUL_1798 * 0.04;
        float MUL_1761 = ADD_130 * SUB_138;
        float ADD_1781 = MUL_1761 + MUL_1756;
        float MUL_1784 = ADD_1781 * 2.0;
        float MUL_1817 = MUL_1784 * 0.085;
        float ADD_1832 = MUL_1817 + MUL_1827;
        float MUL_1759 = SUB_149 * ADD_130;
        float MUL_1755 = ADD_144 * ADD_144;
        float MUL_1765 = SUB_138 * ADD_144;
        float ADD_1799 = MUL_1765 + MUL_1759;
        float MUL_1801 = ADD_1799 * 2.0;
        float MUL_1829 = MUL_1801 * 0.04;
        float MUL_1758 = ADD_130 * ADD_130;
        float ADD_1786 = MUL_1755 + MUL_1758;
        float MUL_1789 = ADD_1786 * 2.0;
        float SUB_1792 = 1.0 - MUL_1789;
        float MUL_1820 = SUB_1792 * 0.085;
        float SUB_1833 = MUL_1829 - MUL_1820;
        float SUB_1793 = MUL_1765 - MUL_1759;
        float MUL_1795 = SUB_1793 * 2.0;
        float MUL_1824 = MUL_1795 * 0.085;
        float MUL_1754 = SUB_138 * SUB_138;
        float ADD_1802 = MUL_1754 + MUL_1758;
        float MUL_1805 = ADD_1802 * 2.0;
        float SUB_1808 = 1.0 - MUL_1805;
        float MUL_1831 = SUB_1808 * 0.04;
        float SUB_1834 = MUL_1831 - MUL_1824;
        float ADD_1835 = 0.333 + SUB_1834;
        float MUL_1849 = MUL_1798 * 0.03;
        float MUL_1851 = MUL_1801 * 0.03;
        float MUL_1853 = SUB_1808 * 0.03;
        float ADD_1854 = 0.333 + MUL_1853;
        float MUL_1868 = MUL_1798 * 0.08;
        float MUL_1870 = MUL_1801 * 0.08;
        float MUL_1872 = SUB_1808 * 0.08;
        float ADD_1873 = 0.333 + MUL_1872;
        float MUL_1882 = MUL_1784 * 0.12;
        float MUL_1885 = SUB_1792 * 0.12;
        float NEGATE_1886 = -MUL_1885;
        float MUL_1889 = MUL_1795 * 0.12;
        float SUB_1897 = 0.333 - MUL_1889;
        float MUL_1906 = MUL_1784 * 0.17;
        float MUL_1909 = SUB_1792 * 0.17;
        float NEGATE_1910 = -MUL_1909;
        float MUL_1913 = MUL_1795 * 0.17;
        float SUB_1921 = 0.333 - MUL_1913;
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
        float MUL_182 = SUB_149 * 0.7071068;
        float MUL_198 = ADD_144 * 0.7071068;
        float MUL_196 = SUB_138 * 0.7071068;
        float SUB_209 = MUL_198 - MUL_196;
        float ADD_199 = MUL_196 + MUL_198;
        float MUL_184 = ADD_130 * 0.7071068;
        float SUB_186 = MUL_182 - MUL_184;
        float ADD_215 = MUL_182 + MUL_184;
        float MUL_224 = ADD_144 * 0.316;
        float MUL_235 = SUB_149 * MUL_224;
        float MUL_228 = ADD_130 * 0.316;
        float MUL_236 = SUB_138 * MUL_228;
        float ADD_237 = MUL_235 + MUL_236;
        float MUL_240 = ADD_237 * 2.0;
        float INPUT_2 = q[2];
        float DIV_262 = INPUT_2 * 0.5;
        float SIN_263 = sin(DIV_262);
        float COS_269 = cos(DIV_262);
        float MUL_286 = ADD_215 * COS_269;
        float MUL_281 = ADD_215 * SIN_263;
        float MUL_284 = SUB_209 * COS_269;
        float ADD_285 = MUL_281 + MUL_284;
        float MUL_1933 = ADD_285 * ADD_285;
        float MUL_289 = SUB_209 * SIN_263;
        float SUB_290 = MUL_286 - MUL_289;
        float MUL_1934 = SUB_290 * ADD_285;
        float MUL_271 = SUB_186 * COS_269;
        float MUL_276 = SUB_186 * SIN_263;
        float MUL_278 = ADD_199 * COS_269;
        float SUB_279 = MUL_278 - MUL_276;
        float MUL_1935 = SUB_290 * SUB_279;
        float MUL_1932 = SUB_279 * SUB_279;
        float ADD_1941 = MUL_1932 + MUL_1933;
        float MUL_1944 = ADD_1941 * 2.0;
        float SUB_1947 = 1.0 - MUL_1944;
        float MUL_1981 = SUB_1947 * 0.039;
        float MUL_272 = ADD_199 * SIN_263;
        float ADD_273 = MUL_271 + MUL_272;
        float MUL_1939 = ADD_273 * ADD_285;
        float ADD_1967 = MUL_1939 + MUL_1935;
        float MUL_1969 = ADD_1967 * 2.0;
        float MUL_1994 = MUL_1969 * 0.052;
        float MUL_1938 = ADD_273 * SUB_279;
        float SUB_1954 = MUL_1938 - MUL_1934;
        float MUL_1956 = SUB_1954 * 2.0;
        float MUL_1987 = MUL_1956 * 0.028;
        float ADD_2004 = MUL_1981 + MUL_1987;
        float SUB_2007 = ADD_2004 - MUL_1994;
        float ADD_2010 = MUL_240 + SUB_2007;
        float ADD_1948 = MUL_1938 + MUL_1934;
        float MUL_1950 = ADD_1948 * 2.0;
        float MUL_1983 = MUL_1950 * 0.039;
        float MUL_1937 = SUB_290 * ADD_273;
        float MUL_1940 = SUB_279 * ADD_285;
        float SUB_1970 = MUL_1940 - MUL_1937;
        float MUL_1972 = SUB_1970 * 2.0;
        float MUL_1998 = MUL_1972 * 0.052;
        float MUL_1936 = ADD_273 * ADD_273;
        float ADD_1957 = MUL_1933 + MUL_1936;
        float MUL_1960 = ADD_1957 * 2.0;
        float SUB_1963 = 1.0 - MUL_1960;
        float MUL_1989 = SUB_1963 * 0.028;
        float ADD_2005 = MUL_1983 + MUL_1989;
        float SUB_2008 = ADD_2005 - MUL_1998;
        float MUL_246 = ADD_144 * MUL_224;
        float MUL_244 = ADD_130 * MUL_228;
        float ADD_247 = MUL_244 + MUL_246;
        float MUL_249 = ADD_247 * 2.0;
        float SUB_252 = MUL_249 - 0.316;
        float ADD_2011 = SUB_252 + SUB_2008;
        float SUB_1951 = MUL_1939 - MUL_1935;
        float ADD_1964 = MUL_1940 + MUL_1937;
        float ADD_1973 = MUL_1932 + MUL_1936;
        float MUL_1976 = ADD_1973 * 2.0;
        float SUB_1979 = 1.0 - MUL_1976;
        float MUL_2002 = SUB_1979 * 0.052;
        float MUL_1966 = ADD_1964 * 2.0;
        float MUL_1991 = MUL_1966 * 0.028;
        float MUL_1953 = SUB_1951 * 2.0;
        float MUL_1985 = MUL_1953 * 0.039;
        float ADD_2006 = MUL_1985 + MUL_1991;
        float SUB_2009 = ADD_2006 - MUL_2002;
        float MUL_253 = SUB_149 * MUL_228;
        float MUL_255 = SUB_138 * MUL_224;
        float SUB_256 = MUL_253 - MUL_255;
        float MUL_258 = SUB_256 * 2.0;
        float ADD_260 = 0.333 + MUL_258;
        float ADD_2012 = ADD_260 + SUB_2009;
        float MUL_2027 = MUL_1969 * 0.1;
        float SUB_2037 = MUL_240 - MUL_2027;
        float MUL_2031 = MUL_1972 * 0.1;
        float SUB_2038 = SUB_252 - MUL_2031;
        float MUL_2035 = SUB_1979 * 0.1;
        float SUB_2039 = ADD_260 - MUL_2035;
        float MUL_2054 = MUL_1969 * 0.06;
        float SUB_2064 = MUL_240 - MUL_2054;
        float MUL_2058 = MUL_1972 * 0.06;
        float SUB_2065 = SUB_252 - MUL_2058;
        float MUL_2062 = SUB_1979 * 0.06;
        float SUB_2066 = ADD_260 - MUL_2062;
        float MUL_2074 = MUL_1956 * 0.06;
        float MUL_2068 = SUB_1947 * 0.08;
        float ADD_2085 = MUL_2068 + MUL_2074;
        float ADD_2088 = MUL_240 + ADD_2085;
        float MUL_2076 = SUB_1963 * 0.06;
        float MUL_2070 = MUL_1950 * 0.08;
        float ADD_2086 = MUL_2070 + MUL_2076;
        float ADD_2089 = SUB_252 + ADD_2086;
        float MUL_2078 = MUL_1966 * 0.06;
        float MUL_2072 = MUL_1953 * 0.08;
        float ADD_2087 = MUL_2072 + MUL_2078;
        float ADD_2090 = ADD_260 + ADD_2087;
        float MUL_2098 = MUL_1956 * 0.02;
        float ADD_2109 = MUL_2068 + MUL_2098;
        float ADD_2112 = MUL_240 + ADD_2109;
        float MUL_2100 = SUB_1963 * 0.02;
        float ADD_2110 = MUL_2070 + MUL_2100;
        float ADD_2113 = SUB_252 + ADD_2110;
        float MUL_2102 = MUL_1966 * 0.02;
        float ADD_2111 = MUL_2072 + MUL_2102;
        float ADD_2114 = ADD_260 + ADD_2111;
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
        float MUL_323 = SUB_290 * 0.7071068;
        float MUL_338 = ADD_285 * 0.7071068;
        float MUL_336 = SUB_279 * 0.7071068;
        float SUB_349 = MUL_338 - MUL_336;
        float ADD_339 = MUL_336 + MUL_338;
        float MUL_325 = ADD_273 * 0.7071068;
        float SUB_354 = MUL_323 - MUL_325;
        float ADD_326 = MUL_323 + MUL_325;
        float MUL_371 = ADD_285 * 0.0825;
        float MUL_376 = ADD_285 * MUL_371;
        float MUL_366 = SUB_279 * 0.0825;
        float MUL_374 = SUB_279 * MUL_366;
        float ADD_378 = MUL_374 + MUL_376;
        float MUL_381 = ADD_378 * 2.0;
        float SUB_384 = 0.0825 - MUL_381;
        float ADD_403 = MUL_240 + SUB_384;
        float INPUT_3 = q[3];
        float DIV_407 = INPUT_3 * 0.5;
        float SIN_408 = sin(DIV_407);
        float COS_414 = cos(DIV_407);
        float MUL_431 = SUB_354 * COS_414;
        float MUL_426 = SUB_354 * SIN_408;
        float MUL_429 = SUB_349 * COS_414;
        float ADD_430 = MUL_426 + MUL_429;
        float MUL_2126 = ADD_430 * ADD_430;
        float MUL_434 = SUB_349 * SIN_408;
        float SUB_435 = MUL_431 - MUL_434;
        float MUL_2127 = SUB_435 * ADD_430;
        float MUL_416 = ADD_326 * COS_414;
        float MUL_421 = ADD_326 * SIN_408;
        float MUL_423 = ADD_339 * COS_414;
        float SUB_424 = MUL_423 - MUL_421;
        float MUL_2128 = SUB_435 * SUB_424;
        float MUL_2125 = SUB_424 * SUB_424;
        float ADD_2134 = MUL_2125 + MUL_2126;
        float MUL_2137 = ADD_2134 * 2.0;
        float SUB_2140 = 1.0 - MUL_2137;
        float MUL_2175 = SUB_2140 * 0.042;
        float MUL_417 = ADD_339 * SIN_408;
        float ADD_418 = MUL_416 + MUL_417;
        float MUL_2132 = ADD_418 * ADD_430;
        float ADD_2160 = MUL_2132 + MUL_2128;
        float MUL_2162 = ADD_2160 * 2.0;
        float MUL_2192 = MUL_2162 * 0.029;
        float MUL_2131 = ADD_418 * SUB_424;
        float SUB_2147 = MUL_2131 - MUL_2127;
        float MUL_2149 = SUB_2147 * 2.0;
        float MUL_2186 = MUL_2149 * 0.049;
        float SUB_2197 = MUL_2186 - MUL_2175;
        float ADD_2200 = SUB_2197 + MUL_2192;
        float ADD_2203 = ADD_403 + ADD_2200;
        float ADD_2141 = MUL_2131 + MUL_2127;
        float MUL_2143 = ADD_2141 * 2.0;
        float MUL_2179 = MUL_2143 * 0.042;
        float MUL_2130 = SUB_435 * ADD_418;
        float MUL_2133 = SUB_424 * ADD_430;
        float SUB_2163 = MUL_2133 - MUL_2130;
        float MUL_2165 = SUB_2163 * 2.0;
        float MUL_2194 = MUL_2165 * 0.029;
        float MUL_2129 = ADD_418 * ADD_418;
        float ADD_2150 = MUL_2126 + MUL_2129;
        float MUL_2153 = ADD_2150 * 2.0;
        float SUB_2156 = 1.0 - MUL_2153;
        float MUL_2188 = SUB_2156 * 0.049;
        float SUB_2198 = MUL_2188 - MUL_2179;
        float ADD_2201 = SUB_2198 + MUL_2194;
        float MUL_386 = SUB_290 * MUL_371;
        float MUL_387 = ADD_273 * MUL_366;
        float ADD_389 = MUL_386 + MUL_387;
        float MUL_392 = ADD_389 * 2.0;
        float ADD_404 = SUB_252 + MUL_392;
        float ADD_2204 = ADD_404 + ADD_2201;
        float SUB_2144 = MUL_2132 - MUL_2128;
        float ADD_2157 = MUL_2133 + MUL_2130;
        float ADD_2166 = MUL_2125 + MUL_2129;
        float MUL_2169 = ADD_2166 * 2.0;
        float SUB_2172 = 1.0 - MUL_2169;
        float MUL_2196 = SUB_2172 * 0.029;
        float MUL_2159 = ADD_2157 * 2.0;
        float MUL_2190 = MUL_2159 * 0.049;
        float MUL_2146 = SUB_2144 * 2.0;
        float MUL_2183 = MUL_2146 * 0.042;
        float SUB_2199 = MUL_2190 - MUL_2183;
        float ADD_2202 = SUB_2199 + MUL_2196;
        float MUL_394 = SUB_290 * MUL_366;
        float MUL_396 = ADD_273 * MUL_371;
        float SUB_398 = MUL_396 - MUL_394;
        float MUL_401 = SUB_398 * 2.0;
        float ADD_405 = ADD_260 + MUL_401;
        float ADD_2205 = ADD_405 + ADD_2202;
        float MUL_2208 = SUB_2140 * 0.08;
        float MUL_2219 = MUL_2149 * 0.095;
        float SUB_2230 = MUL_2219 - MUL_2208;
        float ADD_2233 = ADD_403 + SUB_2230;
        float MUL_2221 = SUB_2156 * 0.095;
        float MUL_2212 = MUL_2143 * 0.08;
        float SUB_2231 = MUL_2221 - MUL_2212;
        float ADD_2234 = ADD_404 + SUB_2231;
        float MUL_2223 = MUL_2159 * 0.095;
        float MUL_2216 = MUL_2146 * 0.08;
        float SUB_2232 = MUL_2223 - MUL_2216;
        float ADD_2235 = ADD_405 + SUB_2232;
        float MUL_2249 = MUL_2162 * 0.02;
        float ADD_2254 = ADD_403 + MUL_2249;
        float MUL_2251 = MUL_2165 * 0.02;
        float ADD_2255 = ADD_404 + MUL_2251;
        float MUL_2253 = SUB_2172 * 0.02;
        float ADD_2256 = ADD_405 + MUL_2253;
        float MUL_2270 = MUL_2162 * 0.06;
        float ADD_2275 = ADD_403 + MUL_2270;
        float MUL_2272 = MUL_2165 * 0.06;
        float ADD_2276 = ADD_404 + MUL_2272;
        float MUL_2274 = SUB_2172 * 0.06;
        float ADD_2277 = ADD_405 + MUL_2274;
        float MUL_2291 = MUL_2149 * 0.06;
        float SUB_2302 = MUL_2291 - MUL_2208;
        float ADD_2305 = ADD_403 + SUB_2302;
        float MUL_2293 = SUB_2156 * 0.06;
        float SUB_2303 = MUL_2293 - MUL_2212;
        float ADD_2306 = ADD_404 + SUB_2303;
        float MUL_2295 = MUL_2159 * 0.06;
        float SUB_2304 = MUL_2295 - MUL_2216;
        float ADD_2307 = ADD_405 + SUB_2304;
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
        float MUL_469 = SUB_435 * 0.7071068;
        float MUL_486 = ADD_430 * 0.7071068;
        float MUL_527 = ADD_430 * 0.0825;
        float MUL_533 = ADD_430 * MUL_527;
        float MUL_483 = SUB_424 * 0.7071068;
        float SUB_488 = MUL_483 - MUL_486;
        float ADD_499 = MUL_483 + MUL_486;
        float MUL_520 = SUB_424 * 0.0825;
        float MUL_472 = ADD_418 * 0.7071068;
        float SUB_473 = MUL_472 - MUL_469;
        float ADD_506 = MUL_469 + MUL_472;
        float MUL_514 = ADD_430 * 0.384;
        float MUL_529 = SUB_435 * MUL_514;
        float MUL_517 = ADD_418 * 0.384;
        float ADD_522 = MUL_517 + MUL_520;
        float MUL_531 = SUB_424 * ADD_522;
        float SUB_532 = MUL_531 - MUL_529;
        float ADD_534 = SUB_532 + MUL_533;
        float MUL_536 = ADD_534 * 2.0;
        float SUB_539 = MUL_536 - 0.0825;
        float ADD_564 = ADD_403 + SUB_539;
        float INPUT_4 = q[4];
        float DIV_568 = INPUT_4 * 0.5;
        float SIN_569 = sin(DIV_568);
        float COS_575 = cos(DIV_568);
        float MUL_592 = ADD_506 * COS_575;
        float MUL_587 = ADD_506 * SIN_569;
        float MUL_590 = ADD_499 * COS_575;
        float ADD_591 = MUL_587 + MUL_590;
        float MUL_2319 = ADD_591 * ADD_591;
        float MUL_595 = ADD_499 * SIN_569;
        float SUB_596 = MUL_592 - MUL_595;
        float MUL_2320 = SUB_596 * ADD_591;
        float MUL_577 = SUB_473 * COS_575;
        float MUL_582 = SUB_473 * SIN_569;
        float MUL_584 = SUB_488 * COS_575;
        float SUB_585 = MUL_584 - MUL_582;
        float MUL_2321 = SUB_596 * SUB_585;
        float MUL_2318 = SUB_585 * SUB_585;
        float ADD_2327 = MUL_2318 + MUL_2319;
        float MUL_2330 = ADD_2327 * 2.0;
        float SUB_2333 = 1.0 - MUL_2330;
        float MUL_2368 = SUB_2333 * 0.001;
        float MUL_578 = SUB_488 * SIN_569;
        float ADD_579 = MUL_577 + MUL_578;
        float MUL_2325 = ADD_579 * ADD_591;
        float ADD_2353 = MUL_2325 + MUL_2321;
        float MUL_2355 = ADD_2353 * 2.0;
        float MUL_2386 = MUL_2355 * 0.11;
        float MUL_2324 = ADD_579 * SUB_585;
        float SUB_2340 = MUL_2324 - MUL_2320;
        float MUL_2342 = SUB_2340 * 2.0;
        float MUL_2379 = MUL_2342 * 0.037;
        float SUB_2396 = MUL_2379 - MUL_2368;
        float SUB_2399 = SUB_2396 - MUL_2386;
        float ADD_2402 = ADD_564 + SUB_2399;
        float ADD_2334 = MUL_2324 + MUL_2320;
        float MUL_2336 = ADD_2334 * 2.0;
        float MUL_2372 = MUL_2336 * 0.001;
        float MUL_2323 = SUB_596 * ADD_579;
        float MUL_2326 = SUB_585 * ADD_591;
        float SUB_2356 = MUL_2326 - MUL_2323;
        float MUL_2358 = SUB_2356 * 2.0;
        float MUL_2390 = MUL_2358 * 0.11;
        float MUL_2322 = ADD_579 * ADD_579;
        float ADD_2343 = MUL_2319 + MUL_2322;
        float MUL_2346 = ADD_2343 * 2.0;
        float SUB_2349 = 1.0 - MUL_2346;
        float MUL_2381 = SUB_2349 * 0.037;
        float SUB_2397 = MUL_2381 - MUL_2372;
        float SUB_2400 = SUB_2397 - MUL_2390;
        float MUL_541 = SUB_435 * MUL_527;
        float MUL_546 = ADD_430 * MUL_514;
        float MUL_543 = ADD_418 * ADD_522;
        float ADD_544 = MUL_541 + MUL_543;
        float ADD_548 = ADD_544 + MUL_546;
        float MUL_551 = ADD_548 * 2.0;
        float SUB_554 = 0.384 - MUL_551;
        float ADD_565 = ADD_404 + SUB_554;
        float ADD_2403 = ADD_565 + SUB_2400;
        float SUB_2337 = MUL_2325 - MUL_2321;
        float ADD_2350 = MUL_2326 + MUL_2323;
        float ADD_2359 = MUL_2318 + MUL_2322;
        float MUL_2362 = ADD_2359 * 2.0;
        float SUB_2365 = 1.0 - MUL_2362;
        float MUL_2394 = SUB_2365 * 0.11;
        float MUL_2352 = ADD_2350 * 2.0;
        float MUL_2383 = MUL_2352 * 0.037;
        float MUL_2339 = SUB_2337 * 2.0;
        float MUL_2376 = MUL_2339 * 0.001;
        float SUB_2398 = MUL_2383 - MUL_2376;
        float SUB_2401 = SUB_2398 - MUL_2394;
        float MUL_555 = SUB_435 * ADD_522;
        float MUL_558 = SUB_424 * MUL_514;
        float MUL_556 = ADD_418 * MUL_527;
        float SUB_557 = MUL_555 - MUL_556;
        float ADD_560 = SUB_557 + MUL_558;
        float MUL_562 = ADD_560 * 2.0;
        float ADD_566 = ADD_405 + MUL_562;
        float ADD_2404 = ADD_566 + SUB_2401;
        float MUL_2412 = MUL_2342 * 0.055;
        float ADD_2423 = ADD_564 + MUL_2412;
        float MUL_2414 = SUB_2349 * 0.055;
        float ADD_2424 = ADD_565 + MUL_2414;
        float MUL_2416 = MUL_2352 * 0.055;
        float ADD_2425 = ADD_566 + MUL_2416;
        float MUL_2433 = MUL_2342 * 0.075;
        float ADD_2444 = ADD_564 + MUL_2433;
        float MUL_2435 = SUB_2349 * 0.075;
        float ADD_2445 = ADD_565 + MUL_2435;
        float MUL_2437 = MUL_2352 * 0.075;
        float ADD_2446 = ADD_566 + MUL_2437;
        float MUL_2461 = MUL_2355 * 0.22;
        float SUB_2471 = ADD_564 - MUL_2461;
        float MUL_2465 = MUL_2358 * 0.22;
        float SUB_2472 = ADD_565 - MUL_2465;
        float MUL_2469 = SUB_2365 * 0.22;
        float SUB_2473 = ADD_566 - MUL_2469;
        float MUL_2488 = MUL_2355 * 0.18;
        float MUL_2481 = MUL_2342 * 0.05;
        float SUB_2498 = MUL_2481 - MUL_2488;
        float ADD_2501 = ADD_564 + SUB_2498;
        float MUL_2492 = MUL_2358 * 0.18;
        float MUL_2483 = SUB_2349 * 0.05;
        float SUB_2499 = MUL_2483 - MUL_2492;
        float ADD_2502 = ADD_565 + SUB_2499;
        float MUL_2496 = SUB_2365 * 0.18;
        float MUL_2485 = MUL_2352 * 0.05;
        float SUB_2500 = MUL_2485 - MUL_2496;
        float ADD_2503 = ADD_566 + SUB_2500;
        float MUL_2511 = MUL_2342 * 0.08;
        float MUL_2518 = MUL_2355 * 0.14;
        float MUL_2505 = SUB_2333 * 0.01;
        float ADD_2528 = MUL_2505 + MUL_2511;
        float SUB_2531 = ADD_2528 - MUL_2518;
        float ADD_2534 = ADD_564 + SUB_2531;
        float MUL_2522 = MUL_2358 * 0.14;
        float MUL_2513 = SUB_2349 * 0.08;
        float MUL_2507 = MUL_2336 * 0.01;
        float ADD_2529 = MUL_2507 + MUL_2513;
        float SUB_2532 = ADD_2529 - MUL_2522;
        float ADD_2535 = ADD_565 + SUB_2532;
        float MUL_2526 = SUB_2365 * 0.14;
        float MUL_2515 = MUL_2352 * 0.08;
        float MUL_2509 = MUL_2339 * 0.01;
        float ADD_2530 = MUL_2509 + MUL_2515;
        float SUB_2533 = ADD_2530 - MUL_2526;
        float ADD_2536 = ADD_566 + SUB_2533;
        float MUL_2544 = MUL_2342 * 0.085;
        float ADD_2561 = MUL_2505 + MUL_2544;
        float SUB_2564 = ADD_2561 - MUL_2386;
        float ADD_2567 = ADD_564 + SUB_2564;
        float MUL_2546 = SUB_2349 * 0.085;
        float ADD_2562 = MUL_2507 + MUL_2546;
        float SUB_2565 = ADD_2562 - MUL_2390;
        float ADD_2568 = ADD_565 + SUB_2565;
        float MUL_2548 = MUL_2352 * 0.085;
        float ADD_2563 = MUL_2509 + MUL_2548;
        float SUB_2566 = ADD_2563 - MUL_2394;
        float ADD_2569 = ADD_566 + SUB_2566;
        float MUL_2584 = MUL_2355 * 0.08;
        float MUL_2577 = MUL_2342 * 0.09;
        float ADD_2594 = MUL_2505 + MUL_2577;
        float SUB_2597 = ADD_2594 - MUL_2584;
        float ADD_2600 = ADD_564 + SUB_2597;
        float MUL_2588 = MUL_2358 * 0.08;
        float MUL_2579 = SUB_2349 * 0.09;
        float ADD_2595 = MUL_2507 + MUL_2579;
        float SUB_2598 = ADD_2595 - MUL_2588;
        float ADD_2601 = ADD_565 + SUB_2598;
        float MUL_2592 = SUB_2365 * 0.08;
        float MUL_2581 = MUL_2352 * 0.09;
        float ADD_2596 = MUL_2509 + MUL_2581;
        float SUB_2599 = ADD_2596 - MUL_2592;
        float ADD_2602 = ADD_566 + SUB_2599;
        float MUL_2617 = MUL_2355 * 0.05;
        float MUL_2610 = MUL_2342 * 0.095;
        float ADD_2627 = MUL_2505 + MUL_2610;
        float SUB_2630 = ADD_2627 - MUL_2617;
        float ADD_2633 = ADD_564 + SUB_2630;
        float MUL_2621 = MUL_2358 * 0.05;
        float MUL_2612 = SUB_2349 * 0.095;
        float ADD_2628 = MUL_2507 + MUL_2612;
        float SUB_2631 = ADD_2628 - MUL_2621;
        float ADD_2634 = ADD_565 + SUB_2631;
        float MUL_2625 = SUB_2365 * 0.05;
        float MUL_2614 = MUL_2352 * 0.095;
        float ADD_2629 = MUL_2509 + MUL_2614;
        float SUB_2632 = ADD_2629 - MUL_2625;
        float ADD_2635 = ADD_566 + SUB_2632;
        float SUB_2666 = MUL_2511 - MUL_2505;
        float SUB_2669 = SUB_2666 - MUL_2518;
        float ADD_2672 = ADD_564 + SUB_2669;
        float SUB_2667 = MUL_2513 - MUL_2507;
        float SUB_2670 = SUB_2667 - MUL_2522;
        float ADD_2673 = ADD_565 + SUB_2670;
        float SUB_2668 = MUL_2515 - MUL_2509;
        float SUB_2671 = SUB_2668 - MUL_2526;
        float ADD_2674 = ADD_566 + SUB_2671;
        float SUB_2705 = MUL_2544 - MUL_2505;
        float SUB_2708 = SUB_2705 - MUL_2386;
        float ADD_2711 = ADD_564 + SUB_2708;
        float SUB_2706 = MUL_2546 - MUL_2507;
        float SUB_2709 = SUB_2706 - MUL_2390;
        float ADD_2712 = ADD_565 + SUB_2709;
        float SUB_2707 = MUL_2548 - MUL_2509;
        float SUB_2710 = SUB_2707 - MUL_2394;
        float ADD_2713 = ADD_566 + SUB_2710;
        float SUB_2744 = MUL_2577 - MUL_2505;
        float SUB_2747 = SUB_2744 - MUL_2584;
        float ADD_2750 = ADD_564 + SUB_2747;
        float SUB_2745 = MUL_2579 - MUL_2507;
        float SUB_2748 = SUB_2745 - MUL_2588;
        float ADD_2751 = ADD_565 + SUB_2748;
        float SUB_2746 = MUL_2581 - MUL_2509;
        float SUB_2749 = SUB_2746 - MUL_2592;
        float ADD_2752 = ADD_566 + SUB_2749;
        float SUB_2783 = MUL_2610 - MUL_2505;
        float SUB_2786 = SUB_2783 - MUL_2617;
        float ADD_2789 = ADD_564 + SUB_2786;
        float SUB_2784 = MUL_2612 - MUL_2507;
        float SUB_2787 = SUB_2784 - MUL_2621;
        float ADD_2790 = ADD_565 + SUB_2787;
        float SUB_2785 = MUL_2614 - MUL_2509;
        float SUB_2788 = SUB_2785 - MUL_2625;
        float ADD_2791 = ADD_566 + SUB_2788;
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
            var_cache[tid][0], var_cache[tid][1], 0.248, 0.154, ADD_2402, ADD_2403, ADD_2404, 0.176))
        {
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
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
        float MUL_657 = SUB_596 * 0.7071068;
        float MUL_654 = ADD_591 * 0.7071068;
        float MUL_651 = SUB_585 * 0.7071068;
        float SUB_655 = MUL_654 - MUL_651;
        float ADD_645 = MUL_651 + MUL_654;
        float MUL_659 = ADD_579 * 0.7071068;
        float SUB_660 = MUL_657 - MUL_659;
        float ADD_632 = MUL_657 + MUL_659;
        float INPUT_5 = q[5];
        float DIV_697 = INPUT_5 * 0.5;
        float SIN_698 = sin(DIV_697);
        float COS_704 = cos(DIV_697);
        float MUL_716 = SUB_660 * SIN_698;
        float MUL_721 = SUB_660 * COS_704;
        float MUL_724 = SUB_655 * SIN_698;
        float SUB_725 = MUL_721 - MUL_724;
        float MUL_719 = SUB_655 * COS_704;
        float ADD_720 = MUL_716 + MUL_719;
        float MUL_2820 = SUB_725 * ADD_720;
        float MUL_2819 = ADD_720 * ADD_720;
        float MUL_711 = ADD_632 * SIN_698;
        float MUL_706 = ADD_632 * COS_704;
        float MUL_707 = ADD_645 * SIN_698;
        float ADD_708 = MUL_706 + MUL_707;
        float MUL_713 = ADD_645 * COS_704;
        float SUB_714 = MUL_713 - MUL_711;
        float MUL_2818 = SUB_714 * SUB_714;
        float ADD_2827 = MUL_2818 + MUL_2819;
        float MUL_2830 = ADD_2827 * 2.0;
        float SUB_2833 = 1.0 - MUL_2830;
        float MUL_2867 = SUB_2833 * 0.042;
        float MUL_2824 = ADD_708 * SUB_714;
        float SUB_2840 = MUL_2824 - MUL_2820;
        float MUL_2842 = SUB_2840 * 2.0;
        float MUL_2873 = MUL_2842 * 0.014;
        float ADD_2884 = MUL_2867 + MUL_2873;
        float ADD_2887 = ADD_564 + ADD_2884;
        float ADD_2834 = MUL_2824 + MUL_2820;
        float MUL_2836 = ADD_2834 * 2.0;
        float MUL_2869 = MUL_2836 * 0.042;
        float MUL_2822 = ADD_708 * ADD_708;
        float ADD_2843 = MUL_2819 + MUL_2822;
        float MUL_2846 = ADD_2843 * 2.0;
        float SUB_2849 = 1.0 - MUL_2846;
        float MUL_2875 = SUB_2849 * 0.014;
        float ADD_2885 = MUL_2869 + MUL_2875;
        float ADD_2888 = ADD_565 + ADD_2885;
        float MUL_2821 = SUB_725 * SUB_714;
        float MUL_2823 = SUB_725 * ADD_708;
        float MUL_2826 = SUB_714 * ADD_720;
        float ADD_2850 = MUL_2826 + MUL_2823;
        float MUL_2852 = ADD_2850 * 2.0;
        float MUL_2877 = MUL_2852 * 0.014;
        float MUL_2825 = ADD_708 * ADD_720;
        float SUB_2837 = MUL_2825 - MUL_2821;
        float MUL_2839 = SUB_2837 * 2.0;
        float MUL_2871 = MUL_2839 * 0.042;
        float ADD_2886 = MUL_2871 + MUL_2877;
        float ADD_2889 = ADD_566 + ADD_2886;
        float MUL_2916 = MUL_2842 * 0.01;
        float MUL_2909 = SUB_2833 * 0.08;
        float SUB_2932 = MUL_2909 - MUL_2916;
        float ADD_2935 = ADD_564 + SUB_2932;
        float MUL_2920 = SUB_2849 * 0.01;
        float MUL_2911 = MUL_2836 * 0.08;
        float SUB_2933 = MUL_2911 - MUL_2920;
        float ADD_2936 = ADD_565 + SUB_2933;
        float MUL_2924 = MUL_2852 * 0.01;
        float MUL_2913 = MUL_2839 * 0.08;
        float SUB_2934 = MUL_2913 - MUL_2924;
        float ADD_2937 = ADD_566 + SUB_2934;
        float MUL_2945 = MUL_2842 * 0.035;
        float ADD_2956 = MUL_2909 + MUL_2945;
        float ADD_2959 = ADD_564 + ADD_2956;
        float MUL_2947 = SUB_2849 * 0.035;
        float ADD_2957 = MUL_2911 + MUL_2947;
        float ADD_2960 = ADD_565 + ADD_2957;
        float MUL_2949 = MUL_2852 * 0.035;
        float ADD_2958 = MUL_2913 + MUL_2949;
        float ADD_2961 = ADD_566 + ADD_2958;
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
            var_cache[tid][0], var_cache[tid][1], 0.248, 0.154, ADD_2887, ADD_2888, ADD_2889, 0.095))
        {
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
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
        float MUL_758 = SUB_725 * 0.7071068;
        float MUL_773 = ADD_720 * 0.7071068;
        float MUL_771 = SUB_714 * 0.7071068;
        float SUB_784 = MUL_773 - MUL_771;
        float ADD_774 = MUL_771 + MUL_773;
        float MUL_760 = ADD_708 * 0.7071068;
        float SUB_789 = MUL_758 - MUL_760;
        float ADD_761 = MUL_758 + MUL_760;
        float MUL_806 = ADD_720 * 0.088;
        float MUL_811 = ADD_720 * MUL_806;
        float MUL_801 = SUB_714 * 0.088;
        float MUL_809 = SUB_714 * MUL_801;
        float ADD_813 = MUL_809 + MUL_811;
        float MUL_816 = ADD_813 * 2.0;
        float SUB_819 = 0.088 - MUL_816;
        float ADD_838 = ADD_564 + SUB_819;
        float INPUT_6 = q[6];
        float DIV_842 = INPUT_6 * 0.5;
        float SIN_843 = sin(DIV_842);
        float COS_849 = cos(DIV_842);
        float MUL_866 = SUB_789 * COS_849;
        float MUL_861 = SUB_789 * SIN_843;
        float MUL_864 = SUB_784 * COS_849;
        float ADD_865 = MUL_861 + MUL_864;
        float MUL_2971 = ADD_865 * ADD_865;
        float MUL_869 = SUB_784 * SIN_843;
        float SUB_870 = MUL_866 - MUL_869;
        float MUL_2972 = SUB_870 * ADD_865;
        float MUL_851 = ADD_761 * COS_849;
        float MUL_856 = ADD_761 * SIN_843;
        float MUL_858 = ADD_774 * COS_849;
        float SUB_859 = MUL_858 - MUL_856;
        float MUL_2973 = SUB_870 * SUB_859;
        float MUL_2970 = SUB_859 * SUB_859;
        float ADD_2979 = MUL_2970 + MUL_2971;
        float MUL_2982 = ADD_2979 * 2.0;
        float SUB_2985 = 1.0 - MUL_2982;
        float MUL_3019 = SUB_2985 * 0.015;
        float MUL_852 = ADD_774 * SIN_843;
        float ADD_853 = MUL_851 + MUL_852;
        float MUL_2977 = ADD_853 * ADD_865;
        float ADD_3005 = MUL_2977 + MUL_2973;
        float MUL_3007 = ADD_3005 * 2.0;
        float MUL_3031 = MUL_3007 * 0.075;
        float MUL_2976 = ADD_853 * SUB_859;
        float SUB_2992 = MUL_2976 - MUL_2972;
        float MUL_2994 = SUB_2992 * 2.0;
        float MUL_3025 = MUL_2994 * 0.015;
        float ADD_3036 = MUL_3019 + MUL_3025;
        float ADD_3039 = ADD_3036 + MUL_3031;
        float ADD_3042 = ADD_838 + ADD_3039;
        float ADD_2986 = MUL_2976 + MUL_2972;
        float MUL_2988 = ADD_2986 * 2.0;
        float MUL_3021 = MUL_2988 * 0.015;
        float MUL_2975 = SUB_870 * ADD_853;
        float MUL_2978 = SUB_859 * ADD_865;
        float SUB_3008 = MUL_2978 - MUL_2975;
        float MUL_3010 = SUB_3008 * 2.0;
        float MUL_3033 = MUL_3010 * 0.075;
        float MUL_2974 = ADD_853 * ADD_853;
        float ADD_2995 = MUL_2971 + MUL_2974;
        float MUL_2998 = ADD_2995 * 2.0;
        float SUB_3001 = 1.0 - MUL_2998;
        float MUL_3027 = SUB_3001 * 0.015;
        float ADD_3037 = MUL_3021 + MUL_3027;
        float ADD_3040 = ADD_3037 + MUL_3033;
        float MUL_821 = SUB_725 * MUL_806;
        float MUL_822 = ADD_708 * MUL_801;
        float ADD_824 = MUL_821 + MUL_822;
        float MUL_827 = ADD_824 * 2.0;
        float ADD_839 = ADD_565 + MUL_827;
        float ADD_3043 = ADD_839 + ADD_3040;
        float SUB_2989 = MUL_2977 - MUL_2973;
        float ADD_3002 = MUL_2978 + MUL_2975;
        float ADD_3011 = MUL_2970 + MUL_2974;
        float MUL_3014 = ADD_3011 * 2.0;
        float SUB_3017 = 1.0 - MUL_3014;
        float MUL_3035 = SUB_3017 * 0.075;
        float MUL_3004 = ADD_3002 * 2.0;
        float MUL_3029 = MUL_3004 * 0.015;
        float MUL_2991 = SUB_2989 * 2.0;
        float MUL_3023 = MUL_2991 * 0.015;
        float ADD_3038 = MUL_3023 + MUL_3029;
        float ADD_3041 = ADD_3038 + MUL_3035;
        float MUL_829 = SUB_725 * MUL_801;
        float MUL_831 = ADD_708 * MUL_806;
        float SUB_833 = MUL_831 - MUL_829;
        float MUL_836 = SUB_833 * 2.0;
        float ADD_840 = ADD_566 + MUL_836;
        float ADD_3044 = ADD_840 + ADD_3041;
        float MUL_3058 = MUL_3007 * 0.07;
        float ADD_3063 = ADD_838 + MUL_3058;
        float MUL_3060 = MUL_3010 * 0.07;
        float ADD_3064 = ADD_839 + MUL_3060;
        float MUL_3062 = SUB_3017 * 0.07;
        float ADD_3065 = ADD_840 + MUL_3062;
        float MUL_3079 = MUL_3007 * 0.08;
        float MUL_3073 = MUL_2994 * 0.04;
        float MUL_3067 = SUB_2985 * 0.02;
        float ADD_3084 = MUL_3067 + MUL_3073;
        float ADD_3087 = ADD_3084 + MUL_3079;
        float ADD_3090 = ADD_838 + ADD_3087;
        float MUL_3081 = MUL_3010 * 0.08;
        float MUL_3075 = SUB_3001 * 0.04;
        float MUL_3069 = MUL_2988 * 0.02;
        float ADD_3085 = MUL_3069 + MUL_3075;
        float ADD_3088 = ADD_3085 + MUL_3081;
        float ADD_3091 = ADD_839 + ADD_3088;
        float MUL_3083 = SUB_3017 * 0.08;
        float MUL_3077 = MUL_3004 * 0.04;
        float MUL_3071 = MUL_2991 * 0.02;
        float ADD_3086 = MUL_3071 + MUL_3077;
        float ADD_3089 = ADD_3086 + MUL_3083;
        float ADD_3092 = ADD_840 + ADD_3089;
        float MUL_3100 = MUL_2994 * 0.02;
        float MUL_3094 = SUB_2985 * 0.04;
        float ADD_3111 = MUL_3094 + MUL_3100;
        float ADD_3114 = ADD_3111 + MUL_3079;
        float ADD_3117 = ADD_838 + ADD_3114;
        float MUL_3102 = SUB_3001 * 0.02;
        float MUL_3096 = MUL_2988 * 0.04;
        float ADD_3112 = MUL_3096 + MUL_3102;
        float ADD_3115 = ADD_3112 + MUL_3081;
        float ADD_3118 = ADD_839 + ADD_3115;
        float MUL_3104 = MUL_3004 * 0.02;
        float MUL_3098 = MUL_2991 * 0.04;
        float ADD_3113 = MUL_3098 + MUL_3104;
        float ADD_3116 = ADD_3113 + MUL_3083;
        float ADD_3119 = ADD_840 + ADD_3116;
        float MUL_3133 = MUL_3007 * 0.085;
        float MUL_3127 = MUL_2994 * 0.06;
        float ADD_3138 = MUL_3094 + MUL_3127;
        float ADD_3141 = ADD_3138 + MUL_3133;
        float ADD_3144 = ADD_838 + ADD_3141;
        float MUL_3135 = MUL_3010 * 0.085;
        float MUL_3129 = SUB_3001 * 0.06;
        float ADD_3139 = MUL_3096 + MUL_3129;
        float ADD_3142 = ADD_3139 + MUL_3135;
        float ADD_3145 = ADD_839 + ADD_3142;
        float MUL_3137 = SUB_3017 * 0.085;
        float MUL_3131 = MUL_3004 * 0.06;
        float ADD_3140 = MUL_3098 + MUL_3131;
        float ADD_3143 = ADD_3140 + MUL_3137;
        float ADD_3146 = ADD_840 + ADD_3143;
        float MUL_3148 = SUB_2985 * 0.06;
        float ADD_3165 = MUL_3148 + MUL_3073;
        float ADD_3168 = ADD_3165 + MUL_3133;
        float ADD_3171 = ADD_838 + ADD_3168;
        float MUL_3150 = MUL_2988 * 0.06;
        float ADD_3166 = MUL_3150 + MUL_3075;
        float ADD_3169 = ADD_3166 + MUL_3135;
        float ADD_3172 = ADD_839 + ADD_3169;
        float MUL_3152 = MUL_2991 * 0.06;
        float ADD_3167 = MUL_3152 + MUL_3077;
        float ADD_3170 = ADD_3167 + MUL_3137;
        float ADD_3173 = ADD_840 + ADD_3170;
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
            var_cache[tid][0], var_cache[tid][1], 0.248, 0.154, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
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
        float MUL_1065 = SUB_870 * 0.9238795;
        float MUL_1062 = ADD_865 * 0.9238795;
        float MUL_1049 = SUB_859 * 0.9238795;
        float MUL_1034 = ADD_853 * 0.9238795;
        float MUL_1055 = SUB_870 * 0.3826834;
        float SUB_1063 = MUL_1062 - MUL_1055;
        float MUL_1072 = ADD_865 * 0.3826834;
        float ADD_1074 = MUL_1065 + MUL_1072;
        float MUL_1037 = SUB_859 * 0.3826834;
        float SUB_1039 = MUL_1034 - MUL_1037;
        float MUL_3241 = SUB_1039 * SUB_1063;
        float MUL_1046 = ADD_853 * 0.3826834;
        float ADD_1050 = MUL_1046 + MUL_1049;
        float MUL_3237 = ADD_1074 * ADD_1050;
        float ADD_3269 = MUL_3241 + MUL_3237;
        float MUL_3271 = ADD_3269 * 2.0;
        float MUL_931 = SUB_859 * 0.107;
        float MUL_942 = SUB_870 * MUL_931;
        float MUL_939 = ADD_853 * 0.107;
        float MUL_944 = ADD_865 * MUL_939;
        float ADD_945 = MUL_942 + MUL_944;
        float MUL_947 = ADD_945 * 2.0;
        float ADD_969 = ADD_838 + MUL_947;
        float MUL_3295 = MUL_3271 * 0.022;
        float ADD_3300 = ADD_969 + MUL_3295;
        float MUL_3239 = ADD_1074 * SUB_1039;
        float MUL_3242 = ADD_1050 * SUB_1063;
        float SUB_3272 = MUL_3242 - MUL_3239;
        float MUL_3274 = SUB_3272 * 2.0;
        float MUL_3297 = MUL_3274 * 0.022;
        float MUL_950 = SUB_870 * MUL_939;
        float MUL_953 = ADD_865 * MUL_931;
        float SUB_954 = MUL_953 - MUL_950;
        float MUL_956 = SUB_954 * 2.0;
        float ADD_970 = ADD_839 + MUL_956;
        float ADD_3301 = ADD_970 + MUL_3297;
        float MUL_3238 = SUB_1039 * SUB_1039;
        float MUL_3234 = ADD_1050 * ADD_1050;
        float ADD_3275 = MUL_3234 + MUL_3238;
        float MUL_3278 = ADD_3275 * 2.0;
        float SUB_3281 = 1.0 - MUL_3278;
        float MUL_3299 = SUB_3281 * 0.022;
        float MUL_961 = SUB_859 * MUL_931;
        float MUL_959 = ADD_853 * MUL_939;
        float ADD_962 = MUL_959 + MUL_961;
        float MUL_965 = ADD_962 * 2.0;
        float SUB_968 = 0.107 - MUL_965;
        float ADD_971 = ADD_840 + SUB_968;
        float ADD_3302 = ADD_971 + MUL_3299;
        float MUL_3322 = MUL_3271 * 0.01;
        float MUL_3236 = ADD_1074 * SUB_1063;
        float MUL_3240 = SUB_1039 * ADD_1050;
        float SUB_3256 = MUL_3240 - MUL_3236;
        float MUL_3258 = SUB_3256 * 2.0;
        float MUL_3311 = MUL_3258 * 0.075;
        float SUB_3327 = MUL_3322 - MUL_3311;
        float ADD_3330 = ADD_969 + SUB_3327;
        float MUL_3324 = MUL_3274 * 0.01;
        float MUL_3235 = SUB_1063 * SUB_1063;
        float ADD_3259 = MUL_3235 + MUL_3238;
        float MUL_3262 = ADD_3259 * 2.0;
        float SUB_3265 = 1.0 - MUL_3262;
        float MUL_3315 = SUB_3265 * 0.075;
        float SUB_3328 = MUL_3324 - MUL_3315;
        float ADD_3331 = ADD_970 + SUB_3328;
        float ADD_3266 = MUL_3242 + MUL_3239;
        float MUL_3326 = SUB_3281 * 0.01;
        float MUL_3268 = ADD_3266 * 2.0;
        float MUL_3319 = MUL_3268 * 0.075;
        float SUB_3329 = MUL_3326 - MUL_3319;
        float ADD_3332 = ADD_971 + SUB_3329;
        float MUL_3341 = MUL_3258 * 0.045;
        float SUB_3357 = MUL_3322 - MUL_3341;
        float ADD_3360 = ADD_969 + SUB_3357;
        float MUL_3345 = SUB_3265 * 0.045;
        float SUB_3358 = MUL_3324 - MUL_3345;
        float ADD_3361 = ADD_970 + SUB_3358;
        float MUL_3349 = MUL_3268 * 0.045;
        float SUB_3359 = MUL_3326 - MUL_3349;
        float ADD_3362 = ADD_971 + SUB_3359;
        float MUL_3371 = MUL_3258 * 0.015;
        float SUB_3387 = MUL_3322 - MUL_3371;
        float ADD_3390 = ADD_969 + SUB_3387;
        float MUL_3375 = SUB_3265 * 0.015;
        float SUB_3388 = MUL_3324 - MUL_3375;
        float ADD_3391 = ADD_970 + SUB_3388;
        float MUL_3379 = MUL_3268 * 0.015;
        float SUB_3389 = MUL_3326 - MUL_3379;
        float ADD_3392 = ADD_971 + SUB_3389;
        float ADD_3411 = MUL_3371 + MUL_3322;
        float ADD_3414 = ADD_969 + ADD_3411;
        float ADD_3412 = MUL_3375 + MUL_3324;
        float ADD_3415 = ADD_970 + ADD_3412;
        float ADD_3413 = MUL_3379 + MUL_3326;
        float ADD_3416 = ADD_971 + ADD_3413;
        float ADD_3435 = MUL_3341 + MUL_3322;
        float ADD_3438 = ADD_969 + ADD_3435;
        float ADD_3436 = MUL_3345 + MUL_3324;
        float ADD_3439 = ADD_970 + ADD_3436;
        float ADD_3437 = MUL_3349 + MUL_3326;
        float ADD_3440 = ADD_971 + ADD_3437;
        float ADD_3459 = MUL_3311 + MUL_3322;
        float ADD_3462 = ADD_969 + ADD_3459;
        float ADD_3460 = MUL_3315 + MUL_3324;
        float ADD_3463 = ADD_970 + ADD_3460;
        float ADD_3461 = MUL_3319 + MUL_3326;
        float ADD_3464 = ADD_971 + ADD_3461;
        float MUL_3484 = MUL_3271 * 0.03;
        float SUB_3489 = MUL_3484 - MUL_3311;
        float ADD_3492 = ADD_969 + SUB_3489;
        float MUL_3486 = MUL_3274 * 0.03;
        float SUB_3490 = MUL_3486 - MUL_3315;
        float ADD_3493 = ADD_970 + SUB_3490;
        float MUL_3488 = SUB_3281 * 0.03;
        float SUB_3491 = MUL_3488 - MUL_3319;
        float ADD_3494 = ADD_971 + SUB_3491;
        float SUB_3519 = MUL_3484 - MUL_3341;
        float ADD_3522 = ADD_969 + SUB_3519;
        float SUB_3520 = MUL_3486 - MUL_3345;
        float ADD_3523 = ADD_970 + SUB_3520;
        float SUB_3521 = MUL_3488 - MUL_3349;
        float ADD_3524 = ADD_971 + SUB_3521;
        float SUB_3549 = MUL_3484 - MUL_3371;
        float ADD_3552 = ADD_969 + SUB_3549;
        float SUB_3550 = MUL_3486 - MUL_3375;
        float ADD_3553 = ADD_970 + SUB_3550;
        float SUB_3551 = MUL_3488 - MUL_3379;
        float ADD_3554 = ADD_971 + SUB_3551;
        float ADD_3573 = MUL_3371 + MUL_3484;
        float ADD_3576 = ADD_969 + ADD_3573;
        float ADD_3574 = MUL_3375 + MUL_3486;
        float ADD_3577 = ADD_970 + ADD_3574;
        float ADD_3575 = MUL_3379 + MUL_3488;
        float ADD_3578 = ADD_971 + ADD_3575;
        float ADD_3597 = MUL_3341 + MUL_3484;
        float ADD_3600 = ADD_969 + ADD_3597;
        float ADD_3598 = MUL_3345 + MUL_3486;
        float ADD_3601 = ADD_970 + ADD_3598;
        float ADD_3599 = MUL_3349 + MUL_3488;
        float ADD_3602 = ADD_971 + ADD_3599;
        float ADD_3621 = MUL_3311 + MUL_3484;
        float ADD_3624 = ADD_969 + ADD_3621;
        float ADD_3622 = MUL_3315 + MUL_3486;
        float ADD_3625 = ADD_970 + ADD_3622;
        float ADD_3623 = MUL_3319 + MUL_3488;
        float ADD_3626 = ADD_971 + ADD_3623;
        float MUL_3646 = MUL_3271 * 0.05;
        float SUB_3651 = MUL_3646 - MUL_3311;
        float ADD_3654 = ADD_969 + SUB_3651;
        float MUL_3648 = MUL_3274 * 0.05;
        float SUB_3652 = MUL_3648 - MUL_3315;
        float ADD_3655 = ADD_970 + SUB_3652;
        float MUL_3650 = SUB_3281 * 0.05;
        float SUB_3653 = MUL_3650 - MUL_3319;
        float ADD_3656 = ADD_971 + SUB_3653;
        float SUB_3681 = MUL_3646 - MUL_3341;
        float ADD_3684 = ADD_969 + SUB_3681;
        float SUB_3682 = MUL_3648 - MUL_3345;
        float ADD_3685 = ADD_970 + SUB_3682;
        float SUB_3683 = MUL_3650 - MUL_3349;
        float ADD_3686 = ADD_971 + SUB_3683;
        float SUB_3711 = MUL_3646 - MUL_3371;
        float ADD_3714 = ADD_969 + SUB_3711;
        float SUB_3712 = MUL_3648 - MUL_3375;
        float ADD_3715 = ADD_970 + SUB_3712;
        float SUB_3713 = MUL_3650 - MUL_3379;
        float ADD_3716 = ADD_971 + SUB_3713;
        float ADD_3735 = MUL_3371 + MUL_3646;
        float ADD_3738 = ADD_969 + ADD_3735;
        float ADD_3736 = MUL_3375 + MUL_3648;
        float ADD_3739 = ADD_970 + ADD_3736;
        float ADD_3737 = MUL_3379 + MUL_3650;
        float ADD_3740 = ADD_971 + ADD_3737;
        float ADD_3759 = MUL_3341 + MUL_3646;
        float ADD_3762 = ADD_969 + ADD_3759;
        float ADD_3760 = MUL_3345 + MUL_3648;
        float ADD_3763 = ADD_970 + ADD_3760;
        float ADD_3761 = MUL_3349 + MUL_3650;
        float ADD_3764 = ADD_971 + ADD_3761;
        float ADD_3783 = MUL_3311 + MUL_3646;
        float ADD_3786 = ADD_969 + ADD_3783;
        float ADD_3784 = MUL_3315 + MUL_3648;
        float ADD_3787 = ADD_970 + ADD_3784;
        float ADD_3785 = MUL_3319 + MUL_3650;
        float ADD_3788 = ADD_971 + ADD_3785;
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
            var_cache[tid][0], var_cache[tid][1], 0.248, 0.154, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
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
        float MUL_3864 = ADD_3269 * 2.0;
        float MUL_3851 = SUB_3256 * 2.0;
        float MUL_1196 = SUB_1063 * 0.065;
        float MUL_1199 = SUB_1039 * 0.065;
        float MUL_1207 = ADD_1050 * MUL_1199;
        float MUL_1203 = SUB_1039 * 0.0584;
        float MUL_1209 = SUB_1063 * MUL_1203;
        float MUL_1194 = ADD_1050 * 0.0584;
        float SUB_1197 = MUL_1194 - MUL_1196;
        float MUL_1206 = ADD_1074 * SUB_1197;
        float ADD_1208 = MUL_1206 + MUL_1207;
        float ADD_1210 = ADD_1208 + MUL_1209;
        float MUL_1212 = ADD_1210 * 2.0;
        float ADD_1235 = ADD_969 + MUL_1212;
        float MUL_3888 = MUL_3864 * 0.033;
        float MUL_3882 = MUL_3851 * 0.012;
        float ADD_3893 = MUL_3882 + MUL_3888;
        float ADD_3896 = ADD_1235 + ADD_3893;
        float MUL_3867 = SUB_3272 * 2.0;
        float MUL_3890 = MUL_3867 * 0.033;
        float MUL_3855 = ADD_3259 * 2.0;
        float SUB_3858 = 1.0 - MUL_3855;
        float MUL_3884 = SUB_3858 * 0.012;
        float ADD_3894 = MUL_3884 + MUL_3890;
        float MUL_1215 = ADD_1074 * MUL_1203;
        float MUL_1220 = SUB_1063 * SUB_1197;
        float MUL_1217 = SUB_1039 * MUL_1199;
        float ADD_1218 = MUL_1215 + MUL_1217;
        float SUB_1221 = MUL_1220 - ADD_1218;
        float MUL_1223 = SUB_1221 * 2.0;
        float ADD_1225 = MUL_1223 + 0.065;
        float ADD_1236 = ADD_970 + ADD_1225;
        float ADD_3897 = ADD_1236 + ADD_3894;
        float MUL_3871 = ADD_3275 * 2.0;
        float SUB_3874 = 1.0 - MUL_3871;
        float MUL_3892 = SUB_3874 * 0.033;
        float MUL_3861 = ADD_3266 * 2.0;
        float MUL_3886 = MUL_3861 * 0.012;
        float ADD_3895 = MUL_3886 + MUL_3892;
        float MUL_1226 = ADD_1074 * MUL_1199;
        float MUL_1227 = SUB_1039 * MUL_1203;
        float SUB_1228 = MUL_1226 - MUL_1227;
        float MUL_1229 = ADD_1050 * SUB_1197;
        float SUB_1230 = SUB_1228 - MUL_1229;
        float MUL_1232 = SUB_1230 * 2.0;
        float ADD_1234 = MUL_1232 + 0.0584;
        float ADD_1237 = ADD_971 + ADD_1234;
        float ADD_3898 = ADD_1237 + ADD_3895;
        float MUL_3912 = MUL_3864 * 0.022;
        float MUL_3906 = MUL_3851 * 0.015;
        float ADD_3917 = MUL_3906 + MUL_3912;
        float ADD_3920 = ADD_1235 + ADD_3917;
        float MUL_3914 = MUL_3867 * 0.022;
        float MUL_3908 = SUB_3858 * 0.015;
        float ADD_3918 = MUL_3908 + MUL_3914;
        float ADD_3921 = ADD_1236 + ADD_3918;
        float MUL_3916 = SUB_3874 * 0.022;
        float MUL_3910 = MUL_3861 * 0.015;
        float ADD_3919 = MUL_3910 + MUL_3916;
        float ADD_3922 = ADD_1237 + ADD_3919;
        float MUL_3936 = MUL_3864 * 0.044;
        float MUL_3930 = MUL_3851 * 0.008;
        float ADD_3941 = MUL_3930 + MUL_3936;
        float ADD_3944 = ADD_1235 + ADD_3941;
        float MUL_3938 = MUL_3867 * 0.044;
        float MUL_3932 = SUB_3858 * 0.008;
        float ADD_3942 = MUL_3932 + MUL_3938;
        float ADD_3945 = ADD_1236 + ADD_3942;
        float MUL_3940 = SUB_3874 * 0.044;
        float MUL_3934 = MUL_3861 * 0.008;
        float ADD_3943 = MUL_3934 + MUL_3940;
        float ADD_3946 = ADD_1237 + ADD_3943;
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
            var_cache[tid][0], var_cache[tid][1], 0.248, 0.154, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
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
        float ADD_1331 = MUL_1194 + MUL_1196;
        float MUL_3990 = ADD_3269 * 2.0;
        float MUL_4020 = MUL_3990 * 0.033;
        float MUL_3977 = SUB_3256 * 2.0;
        float MUL_4009 = MUL_3977 * 0.012;
        float SUB_4025 = MUL_4020 - MUL_4009;
        float MUL_1342 = ADD_1074 * ADD_1331;
        float SUB_1345 = MUL_1342 - MUL_1207;
        float ADD_1347 = SUB_1345 + MUL_1209;
        float MUL_1349 = ADD_1347 * 2.0;
        float ADD_1377 = ADD_969 + MUL_1349;
        float ADD_4028 = ADD_1377 + SUB_4025;
        float SUB_1356 = MUL_1217 - MUL_1215;
        float MUL_3993 = SUB_3272 * 2.0;
        float MUL_4022 = MUL_3993 * 0.033;
        float MUL_3981 = ADD_3259 * 2.0;
        float SUB_3984 = 1.0 - MUL_3981;
        float MUL_4013 = SUB_3984 * 0.012;
        float SUB_4026 = MUL_4022 - MUL_4013;
        float MUL_1357 = SUB_1063 * ADD_1331;
        float ADD_1358 = SUB_1356 + MUL_1357;
        float MUL_1360 = ADD_1358 * 2.0;
        float SUB_1363 = MUL_1360 - 0.065;
        float ADD_1378 = ADD_970 + SUB_1363;
        float ADD_4029 = ADD_1378 + SUB_4026;
        float ADD_1367 = MUL_1226 + MUL_1227;
        float MUL_3997 = ADD_3275 * 2.0;
        float SUB_4000 = 1.0 - MUL_3997;
        float MUL_4024 = SUB_4000 * 0.033;
        float MUL_3987 = ADD_3266 * 2.0;
        float MUL_4017 = MUL_3987 * 0.012;
        float SUB_4027 = MUL_4024 - MUL_4017;
        float MUL_1369 = ADD_1050 * ADD_1331;
        float ADD_1370 = ADD_1367 + MUL_1369;
        float MUL_1373 = ADD_1370 * 2.0;
        float SUB_1376 = 0.0584 - MUL_1373;
        float ADD_1379 = ADD_971 + SUB_1376;
        float ADD_4030 = ADD_1379 + SUB_4027;
        float MUL_4050 = MUL_3990 * 0.022;
        float MUL_4039 = MUL_3977 * 0.015;
        float SUB_4055 = MUL_4050 - MUL_4039;
        float ADD_4058 = ADD_1377 + SUB_4055;
        float MUL_4052 = MUL_3993 * 0.022;
        float MUL_4043 = SUB_3984 * 0.015;
        float SUB_4056 = MUL_4052 - MUL_4043;
        float ADD_4059 = ADD_1378 + SUB_4056;
        float MUL_4054 = SUB_4000 * 0.022;
        float MUL_4047 = MUL_3987 * 0.015;
        float SUB_4057 = MUL_4054 - MUL_4047;
        float ADD_4060 = ADD_1379 + SUB_4057;
        float MUL_4080 = MUL_3990 * 0.044;
        float MUL_4069 = MUL_3977 * 0.008;
        float SUB_4085 = MUL_4080 - MUL_4069;
        float ADD_4088 = ADD_1377 + SUB_4085;
        float MUL_4082 = MUL_3993 * 0.044;
        float MUL_4073 = SUB_3984 * 0.008;
        float SUB_4086 = MUL_4082 - MUL_4073;
        float ADD_4089 = ADD_1378 + SUB_4086;
        float MUL_4084 = SUB_4000 * 0.044;
        float MUL_4077 = MUL_3987 * 0.008;
        float SUB_4087 = MUL_4084 - MUL_4077;
        float ADD_4090 = ADD_1379 + SUB_4087;
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
            var_cache[tid][0], var_cache[tid][1], 0.248, 0.154, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][2], var_cache[tid][3], 0.333, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    var_cache[tid][4], var_cache[tid][5], 0.333, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
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

    template <>
    __device__  __forceinline__ bool fkcc<ppln::robots::Fetch>(volatile float *q, ppln::collision::Environment<float> *environment, float var_cache[256][10], int tid)
    {
        if (/*base_link*/ sphere_environment_in_collision(environment, -0.02, 0.0, 0.188, 0.34))
        {
            if (sphere_environment_in_collision(environment, -0.12, 0.0, 0.182, 0.24))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.225, 0.0, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.08, -0.06, 0.16, 0.22))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.215, -0.07, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.185, -0.135, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.13, -0.185, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.065, -0.2, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.01, -0.2, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.08, 0.06, 0.16, 0.22))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.215, 0.07, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.185, 0.135, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.13, 0.185, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.065, 0.2, 0.31, 0.066))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.01, 0.2, 0.31, 0.066))
            {
                return false;
            }
        }  // (0, 0)
        if (/*torso_fixed_link*/ sphere_environment_in_collision(
            environment, -0.186875, 0.0, 0.587425, 0.277))
        {
            if (sphere_environment_in_collision(environment, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
        }  // (0, 0)
        auto INPUT_0 = q[0];
        auto ADD_54 = INPUT_0 + 0.37743;
        auto ADD_1421 = ADD_54 + 0.3;
        auto ADD_1429 = ADD_54 + 0.15;
        auto ADD_1450 = ADD_54 + 0.45;
        if (/*torso_lift_link*/ sphere_environment_in_collision(environment, -0.186875, 0.0, ADD_1421, 0.308))
        {
            if (sphere_environment_in_collision(environment, -0.186875, -0.05, ADD_1429, 0.15))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, 0.05, ADD_1429, 0.15))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, 0.05, ADD_1421, 0.15))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, 0.05, ADD_1450, 0.15))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, -0.05, ADD_1450, 0.15))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.186875, -0.05, ADD_1421, 0.15))
            {
                return false;
            }
        }  // (0, 5)
        auto ADD_1490 = ADD_54 + 0.24;
        if (sphere_environment_in_collision(environment, 0.013125, 0.0, ADD_1490, 0.07))
        {
            return false;
        }  // (5, 6)
        auto ADD_66 = ADD_54 + 0.6030014;
        auto ADD_1497 = ADD_66 + 0.059;
        auto ADD_1501 = ADD_66 + 0.06;
        auto ADD_1506 = ADD_66 + 0.058;
        auto ADD_1534 = ADD_66 + 0.03;
        auto ADD_1558 = ADD_66 + 0.085;
        auto ADD_1582 = ADD_66 + 0.075;
        auto ADD_1588 = ADD_66 + 0.0575;
        auto ADD_1594 = ADD_66 + 0.04;
        if (/*head_pan_link*/ sphere_environment_in_collision(environment, 0.01325, 0.0, ADD_1497, 0.197))
        {
            if (sphere_environment_in_collision(environment, -0.03375, 0.0, ADD_1501, 0.15))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.11125, 0.0, ADD_1506, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.11125, -0.0425, ADD_1506, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.11125, 0.0425, ADD_1506, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.11125, 0.085, ADD_1506, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.11125, -0.085, ADD_1506, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.02875, -0.115, ADD_1534, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.05425, -0.115, ADD_1534, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.07975, -0.115, ADD_1534, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.10525, -0.115, ADD_1534, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.02875, -0.115, ADD_1558, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.05425, -0.115, ADD_1558, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.07975, -0.115, ADD_1558, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.10525, -0.115, ADD_1558, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.12625, -0.115, ADD_1582, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.13425, -0.115, ADD_1588, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.12625, -0.115, ADD_1594, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.02875, 0.115, ADD_1534, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.05425, 0.115, ADD_1534, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.07975, 0.115, ADD_1534, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.10525, 0.115, ADD_1534, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.02875, 0.115, ADD_1558, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.05425, 0.115, ADD_1558, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.07975, 0.115, ADD_1558, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.10525, 0.115, ADD_1558, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.12625, 0.115, ADD_1582, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.13425, 0.115, ADD_1588, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.12625, 0.115, ADD_1594, 0.03))
            {
                return false;
            }
        }  // (6, 15)
        auto INPUT_1 = q[1];
        auto DIV_84 = INPUT_1 * 0.5;
        auto SIN_85 = sin(DIV_84);
        auto COS_91 = cos(DIV_84);
        auto MUL_1710 = COS_91 * SIN_85;
        auto MUL_1729 = MUL_1710 * 2.0;
        auto MUL_1755 = MUL_1729 * 0.015;
        auto MUL_1709 = SIN_85 * SIN_85;
        auto MUL_1719 = MUL_1709 * 2.0;
        auto SUB_1722 = 1.0 - MUL_1719;
        auto MUL_1748 = SUB_1722 * 0.06;
        auto ADD_1768 = MUL_1748 + MUL_1755;
        auto ADD_1770 = 0.03265 + ADD_1768;
        auto MUL_1758 = SUB_1722 * 0.015;
        auto MUL_1750 = MUL_1729 * 0.06;
        auto SUB_1769 = MUL_1750 - MUL_1758;
        auto ADD_82 = ADD_54 + 0.34858;
        auto ADD_1771 = ADD_82 + 0.03;
        auto MUL_1790 = SUB_1722 * 0.025;
        auto ADD_1810 = MUL_1790 + MUL_1755;
        auto ADD_1812 = 0.03265 + ADD_1810;
        auto MUL_1792 = MUL_1729 * 0.025;
        auto SUB_1811 = MUL_1792 - MUL_1758;
        auto ADD_1813 = ADD_82 + 0.035;
        auto MUL_1822 = MUL_1729 * 0.03;
        auto MUL_1815 = SUB_1722 * 0.05;
        auto ADD_1835 = MUL_1815 + MUL_1822;
        auto ADD_1837 = 0.03265 + ADD_1835;
        auto MUL_1817 = MUL_1729 * 0.05;
        auto MUL_1825 = SUB_1722 * 0.03;
        auto SUB_1836 = MUL_1817 - MUL_1825;
        auto ADD_1838 = ADD_82 + 0.06;
        auto MUL_1840 = SUB_1722 * 0.12;
        auto ADD_1860 = MUL_1840 + MUL_1822;
        auto ADD_1862 = 0.03265 + ADD_1860;
        auto MUL_1842 = MUL_1729 * 0.12;
        auto SUB_1861 = MUL_1842 - MUL_1825;
        if (/*shoulder_pan_link*/ sphere_environment_in_collision(
            environment, ADD_1770, SUB_1769, ADD_1771, 0.124))
        {
            if (sphere_environment_in_collision(environment, 0.03265, 0.0, ADD_82, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_1812, SUB_1811, ADD_1813, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_1837, SUB_1836, ADD_1838, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_1862, SUB_1861, ADD_1838, 0.055))
            {
                return false;
            }
        }  // (15, 52)
        auto MUL_162 = SIN_85 * 0.117;
        auto MUL_166 = SIN_85 * MUL_162;
        auto MUL_169 = MUL_166 * 2.0;
        auto SUB_172 = 0.117 - MUL_169;
        auto ADD_186 = 0.03265 + SUB_172;
        auto INPUT_2 = q[2];
        auto DIV_189 = INPUT_2 * 0.5;
        auto SIN_190 = sin(DIV_189);
        auto COS_196 = cos(DIV_189);
        auto MUL_210 = COS_91 * COS_196;
        auto MUL_202 = COS_91 * SIN_190;
        auto MUL_1874 = MUL_202 * MUL_202;
        auto MUL_209 = SIN_85 * COS_196;
        auto MUL_1875 = MUL_209 * MUL_209;
        auto ADD_1886 = MUL_1874 + MUL_1875;
        auto MUL_1889 = ADD_1886 * 2.0;
        auto SUB_1892 = 1.0 - MUL_1889;
        auto MUL_1930 = SUB_1892 * 0.063;
        auto MUL_1876 = MUL_210 * MUL_209;
        auto MUL_200 = SIN_85 * SIN_190;
        auto MUL_1881 = MUL_200 * MUL_202;
        auto ADD_1901 = MUL_1881 + MUL_1876;
        auto MUL_1904 = ADD_1901 * 2.0;
        auto MUL_1937 = MUL_1904 * 0.019;
        auto SUB_1949 = MUL_1930 - MUL_1937;
        auto ADD_1952 = ADD_186 + SUB_1949;
        auto SUB_1893 = MUL_1876 - MUL_1881;
        auto MUL_1895 = SUB_1893 * 2.0;
        auto MUL_1932 = MUL_1895 * 0.063;
        auto MUL_1878 = MUL_200 * MUL_200;
        auto ADD_1906 = MUL_1875 + MUL_1878;
        auto MUL_1909 = ADD_1906 * 2.0;
        auto SUB_1912 = 1.0 - MUL_1909;
        auto MUL_1940 = SUB_1912 * 0.019;
        auto ADD_1950 = MUL_1932 + MUL_1940;
        auto MUL_174 = COS_91 * MUL_162;
        auto MUL_178 = MUL_174 * 2.0;
        auto ADD_1953 = MUL_178 + ADD_1950;
        auto MUL_1879 = MUL_210 * MUL_200;
        auto MUL_1877 = MUL_210 * MUL_202;
        auto MUL_1883 = MUL_200 * MUL_209;
        auto ADD_1896 = MUL_1883 + MUL_1877;
        auto MUL_1899 = ADD_1896 * 2.0;
        auto MUL_1934 = MUL_1899 * 0.063;
        auto MUL_1885 = MUL_202 * MUL_209;
        auto SUB_1913 = MUL_1885 - MUL_1879;
        auto MUL_1915 = SUB_1913 * 2.0;
        auto MUL_1942 = MUL_1915 * 0.019;
        auto SUB_1951 = MUL_1942 - MUL_1934;
        auto ADD_1954 = ADD_1838 + SUB_1951;
        auto SUB_1916 = MUL_1877 - MUL_1883;
        auto MUL_1918 = SUB_1916 * 2.0;
        auto MUL_1970 = MUL_1918 * 0.025;
        auto MUL_1963 = MUL_1904 * 0.04;
        auto MUL_1956 = SUB_1892 * 0.025;
        auto SUB_1975 = MUL_1956 - MUL_1963;
        auto ADD_1978 = SUB_1975 + MUL_1970;
        auto ADD_1981 = ADD_186 + ADD_1978;
        auto ADD_1919 = MUL_1885 + MUL_1879;
        auto MUL_1921 = ADD_1919 * 2.0;
        auto MUL_1972 = MUL_1921 * 0.025;
        auto MUL_1966 = SUB_1912 * 0.04;
        auto MUL_1958 = MUL_1895 * 0.025;
        auto ADD_1976 = MUL_1958 + MUL_1966;
        auto ADD_1979 = ADD_1976 + MUL_1972;
        auto ADD_1982 = MUL_178 + ADD_1979;
        auto ADD_1922 = MUL_1874 + MUL_1878;
        auto MUL_1925 = ADD_1922 * 2.0;
        auto SUB_1928 = 1.0 - MUL_1925;
        auto MUL_1974 = SUB_1928 * 0.025;
        auto MUL_1968 = MUL_1915 * 0.04;
        auto MUL_1960 = MUL_1899 * 0.025;
        auto SUB_1977 = MUL_1968 - MUL_1960;
        auto ADD_1980 = SUB_1977 + MUL_1974;
        auto ADD_1983 = ADD_1838 + ADD_1980;
        auto ADD_2014 = MUL_1956 + MUL_1963;
        auto ADD_2018 = ADD_2014 + MUL_1970;
        auto SUB_2022 = ADD_186 - ADD_2018;
        auto SUB_2016 = MUL_1966 - MUL_1958;
        auto SUB_2020 = SUB_2016 - MUL_1972;
        auto ADD_2023 = MUL_178 + SUB_2020;
        auto ADD_2017 = MUL_1960 + MUL_1968;
        auto SUB_2021 = ADD_2017 - MUL_1974;
        auto ADD_2024 = ADD_1838 + SUB_2021;
        auto SUB_2054 = SUB_1975 - MUL_1970;
        auto ADD_2057 = ADD_186 + SUB_2054;
        auto SUB_2055 = ADD_1976 - MUL_1972;
        auto ADD_2058 = MUL_178 + SUB_2055;
        auto SUB_2056 = SUB_1977 - MUL_1974;
        auto ADD_2059 = ADD_1838 + SUB_2056;
        auto SUB_2088 = MUL_1970 - ADD_2014;
        auto ADD_2091 = ADD_186 + SUB_2088;
        auto ADD_2089 = SUB_2016 + MUL_1972;
        auto ADD_2092 = MUL_178 + ADD_2089;
        auto ADD_2090 = ADD_2017 + MUL_1974;
        auto ADD_2093 = ADD_1838 + ADD_2090;
        auto MUL_2095 = SUB_1892 * 0.08;
        auto ADD_2113 = ADD_186 + MUL_2095;
        auto MUL_2097 = MUL_1895 * 0.08;
        auto ADD_2114 = MUL_178 + MUL_2097;
        auto MUL_2099 = MUL_1899 * 0.08;
        auto SUB_2115 = ADD_1838 - MUL_2099;
        auto MUL_2117 = SUB_1892 * 0.11;
        auto ADD_2135 = ADD_186 + MUL_2117;
        auto MUL_2119 = MUL_1895 * 0.11;
        auto ADD_2136 = MUL_178 + MUL_2119;
        auto MUL_2121 = MUL_1899 * 0.11;
        auto SUB_2137 = ADD_1838 - MUL_2121;
        auto MUL_2139 = SUB_1892 * 0.14;
        auto ADD_2157 = ADD_186 + MUL_2139;
        auto MUL_2141 = MUL_1895 * 0.14;
        auto ADD_2158 = MUL_178 + MUL_2141;
        auto MUL_2143 = MUL_1899 * 0.14;
        auto SUB_2159 = ADD_1838 - MUL_2143;
        if (/*shoulder_lift_link*/ sphere_environment_in_collision(
            environment, ADD_1952, ADD_1953, ADD_1954, 0.134))
        {
            if (sphere_environment_in_collision(environment, ADD_1981, ADD_1982, ADD_1983, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2022, ADD_2023, ADD_2024, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2057, ADD_2058, ADD_2059, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2091, ADD_2092, ADD_2093, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2113, ADD_2114, SUB_2115, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2135, ADD_2136, SUB_2137, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2157, ADD_2158, SUB_2159, 0.055))
            {
                return false;
            }
        }  // (52, 166)
        if (/*torso_lift_link_collision_2 vs. shoulder_lift_link*/ sphere_sphere_self_collision(0.013125, 0.0, ADD_1490, 0.07, ADD_1952, ADD_1953, ADD_1954, 0.134))
        {
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_1981, ADD_1982, ADD_1983, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, SUB_2022, ADD_2023, ADD_2024, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2057, ADD_2058, ADD_2059, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2091, ADD_2092, ADD_2093, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2113, ADD_2114, SUB_2115, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2135, ADD_2136, SUB_2137, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2157, ADD_2158, SUB_2159, 0.055))
            {
                return false;
            }
        }  // (166, 166)
        auto MUL_285 = MUL_209 * 0.219;
        auto MUL_290 = MUL_209 * MUL_285;
        auto MUL_280 = MUL_202 * 0.219;
        auto MUL_288 = MUL_202 * MUL_280;
        auto ADD_292 = MUL_288 + MUL_290;
        auto MUL_295 = ADD_292 * 2.0;
        auto SUB_298 = 0.219 - MUL_295;
        auto ADD_317 = ADD_186 + SUB_298;
        auto INPUT_3 = q[3];
        auto DIV_321 = INPUT_3 * 0.5;
        auto SIN_322 = sin(DIV_321);
        auto COS_328 = cos(DIV_321);
        auto MUL_344 = MUL_209 * COS_328;
        auto MUL_338 = MUL_209 * SIN_322;
        auto MUL_346 = MUL_210 * COS_328;
        auto MUL_329 = MUL_210 * SIN_322;
        auto MUL_330 = MUL_200 * COS_328;
        auto SUB_332 = MUL_329 - MUL_330;
        auto MUL_347 = MUL_200 * SIN_322;
        auto ADD_349 = MUL_346 + MUL_347;
        auto MUL_337 = MUL_202 * COS_328;
        auto ADD_339 = MUL_337 + MUL_338;
        auto MUL_2176 = ADD_339 * ADD_339;
        auto MUL_2182 = SUB_332 * ADD_339;
        auto MUL_342 = MUL_202 * SIN_322;
        auto SUB_345 = MUL_344 - MUL_342;
        auto MUL_2178 = ADD_349 * SUB_345;
        auto SUB_2198 = MUL_2182 - MUL_2178;
        auto MUL_2200 = SUB_2198 * 2.0;
        auto MUL_2232 = MUL_2200 * 0.02;
        auto MUL_2177 = SUB_345 * SUB_345;
        auto ADD_2185 = MUL_2176 + MUL_2177;
        auto MUL_2188 = ADD_2185 * 2.0;
        auto SUB_2191 = 1.0 - MUL_2188;
        auto MUL_2225 = SUB_2191 * 0.056;
        auto SUB_2248 = MUL_2225 - MUL_2232;
        auto ADD_2251 = ADD_317 + SUB_2248;
        auto ADD_2192 = MUL_2182 + MUL_2178;
        auto MUL_2194 = ADD_2192 * 2.0;
        auto MUL_2227 = MUL_2194 * 0.056;
        auto MUL_2180 = SUB_332 * SUB_332;
        auto ADD_2201 = MUL_2177 + MUL_2180;
        auto MUL_2204 = ADD_2201 * 2.0;
        auto SUB_2207 = 1.0 - MUL_2204;
        auto MUL_2236 = SUB_2207 * 0.02;
        auto SUB_2249 = MUL_2227 - MUL_2236;
        auto MUL_300 = MUL_210 * MUL_285;
        auto MUL_301 = MUL_200 * MUL_280;
        auto SUB_302 = MUL_300 - MUL_301;
        auto MUL_305 = SUB_302 * 2.0;
        auto ADD_318 = MUL_178 + MUL_305;
        auto ADD_2252 = ADD_318 + SUB_2249;
        auto MUL_2179 = ADD_349 * ADD_339;
        auto MUL_2181 = ADD_349 * SUB_332;
        auto MUL_2184 = ADD_339 * SUB_345;
        auto ADD_2208 = MUL_2184 + MUL_2181;
        auto MUL_2210 = ADD_2208 * 2.0;
        auto MUL_2240 = MUL_2210 * 0.02;
        auto MUL_2183 = SUB_332 * SUB_345;
        auto SUB_2195 = MUL_2183 - MUL_2179;
        auto MUL_2197 = SUB_2195 * 2.0;
        auto MUL_2229 = MUL_2197 * 0.056;
        auto SUB_2250 = MUL_2229 - MUL_2240;
        auto MUL_307 = MUL_210 * MUL_280;
        auto MUL_309 = MUL_200 * MUL_285;
        auto ADD_310 = MUL_307 + MUL_309;
        auto MUL_314 = ADD_310 * 2.0;
        auto SUB_319 = ADD_1838 - MUL_314;
        auto ADD_2253 = SUB_319 + SUB_2250;
        auto MUL_2256 = SUB_2191 * 0.02;
        auto SUB_2278 = ADD_317 - MUL_2256;
        auto MUL_2260 = MUL_2194 * 0.02;
        auto SUB_2279 = ADD_318 - MUL_2260;
        auto MUL_2264 = MUL_2197 * 0.02;
        auto SUB_2280 = SUB_319 - MUL_2264;
        auto MUL_2282 = SUB_2191 * 0.03;
        auto ADD_2299 = ADD_317 + MUL_2282;
        auto MUL_2284 = MUL_2194 * 0.03;
        auto ADD_2300 = ADD_318 + MUL_2284;
        auto MUL_2286 = MUL_2197 * 0.03;
        auto ADD_2301 = SUB_319 + MUL_2286;
        auto MUL_2303 = SUB_2191 * 0.08;
        auto ADD_2320 = ADD_317 + MUL_2303;
        auto MUL_2305 = MUL_2194 * 0.08;
        auto ADD_2321 = ADD_318 + MUL_2305;
        auto MUL_2307 = MUL_2197 * 0.08;
        auto ADD_2322 = SUB_319 + MUL_2307;
        auto ADD_2211 = MUL_2183 + MUL_2179;
        auto MUL_2213 = ADD_2211 * 2.0;
        auto MUL_2342 = MUL_2213 * 0.02;
        auto MUL_2324 = SUB_2191 * 0.11;
        auto MUL_2331 = MUL_2200 * 0.045;
        auto SUB_2347 = MUL_2324 - MUL_2331;
        auto ADD_2350 = SUB_2347 + MUL_2342;
        auto ADD_2353 = ADD_317 + ADD_2350;
        auto SUB_2214 = MUL_2184 - MUL_2181;
        auto MUL_2216 = SUB_2214 * 2.0;
        auto MUL_2344 = MUL_2216 * 0.02;
        auto MUL_2335 = SUB_2207 * 0.045;
        auto MUL_2326 = MUL_2194 * 0.11;
        auto SUB_2348 = MUL_2326 - MUL_2335;
        auto ADD_2351 = SUB_2348 + MUL_2344;
        auto ADD_2354 = ADD_318 + ADD_2351;
        auto ADD_2217 = MUL_2176 + MUL_2180;
        auto MUL_2220 = ADD_2217 * 2.0;
        auto SUB_2223 = 1.0 - MUL_2220;
        auto MUL_2346 = SUB_2223 * 0.02;
        auto MUL_2339 = MUL_2210 * 0.045;
        auto MUL_2328 = MUL_2197 * 0.11;
        auto SUB_2349 = MUL_2328 - MUL_2339;
        auto ADD_2352 = SUB_2349 + MUL_2346;
        auto ADD_2355 = SUB_319 + ADD_2352;
        auto SUB_2389 = SUB_2347 - MUL_2342;
        auto ADD_2392 = ADD_317 + SUB_2389;
        auto SUB_2390 = SUB_2348 - MUL_2344;
        auto ADD_2393 = ADD_318 + SUB_2390;
        auto SUB_2391 = SUB_2349 - MUL_2346;
        auto ADD_2394 = SUB_319 + SUB_2391;
        auto MUL_2396 = SUB_2191 * 0.155;
        auto SUB_2419 = MUL_2396 - MUL_2331;
        auto ADD_2422 = SUB_2419 + MUL_2342;
        auto ADD_2425 = ADD_317 + ADD_2422;
        auto MUL_2398 = MUL_2194 * 0.155;
        auto SUB_2420 = MUL_2398 - MUL_2335;
        auto ADD_2423 = SUB_2420 + MUL_2344;
        auto ADD_2426 = ADD_318 + ADD_2423;
        auto MUL_2400 = MUL_2197 * 0.155;
        auto SUB_2421 = MUL_2400 - MUL_2339;
        auto ADD_2424 = SUB_2421 + MUL_2346;
        auto ADD_2427 = SUB_319 + ADD_2424;
        auto SUB_2461 = SUB_2419 - MUL_2342;
        auto ADD_2464 = ADD_317 + SUB_2461;
        auto SUB_2462 = SUB_2420 - MUL_2344;
        auto ADD_2465 = ADD_318 + SUB_2462;
        auto SUB_2463 = SUB_2421 - MUL_2346;
        auto ADD_2466 = SUB_319 + SUB_2463;
        auto MUL_2468 = SUB_2191 * 0.13;
        auto ADD_2485 = ADD_317 + MUL_2468;
        auto MUL_2470 = MUL_2194 * 0.13;
        auto ADD_2486 = ADD_318 + MUL_2470;
        auto MUL_2472 = MUL_2197 * 0.13;
        auto ADD_2487 = SUB_319 + MUL_2472;
        if (/*head_pan_link vs. upperarm_roll_link*/ sphere_sphere_self_collision(
            0.01325, 0.0, ADD_1497, 0.197, ADD_2251, ADD_2252, ADD_2253, 0.134))
        {
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
        }  // (166, 308)
        if (/*shoulder_pan_link vs. upperarm_roll_link*/ sphere_sphere_self_collision(
            ADD_1770, SUB_1769, ADD_1771, 0.124, ADD_2251, ADD_2252, ADD_2253, 0.134))
        {
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
        }  // (308, 308)
        if (/*torso_lift_link vs. upperarm_roll_link*/ sphere_sphere_self_collision(
            -0.186875, 0.0, ADD_1421, 0.308, ADD_2251, ADD_2252, ADD_2253, 0.134))
        {
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
        }  // (308, 308)
        if (/*torso_lift_link_collision_2 vs. upperarm_roll_link*/ sphere_sphere_self_collision(0.013125, 0.0, ADD_1490, 0.07, ADD_2251, ADD_2252, ADD_2253, 0.134))
        {
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
        }  // (308, 308)
        if (/*upperarm_roll_link vs. torso_fixed_link*/ sphere_sphere_self_collision(
            ADD_2251, ADD_2252, ADD_2253, 0.134, -0.186875, 0.0, 0.587425, 0.277))
        {
            if (sphere_sphere_self_collision(
                    SUB_2278, SUB_2279, SUB_2280, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2278, SUB_2279, SUB_2280, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2278, SUB_2279, SUB_2280, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2278, SUB_2279, SUB_2280, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2278, SUB_2279, SUB_2280, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2278, SUB_2279, SUB_2280, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2299, ADD_2300, ADD_2301, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2299, ADD_2300, ADD_2301, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2299, ADD_2300, ADD_2301, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2299, ADD_2300, ADD_2301, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2299, ADD_2300, ADD_2301, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2299, ADD_2300, ADD_2301, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2320, ADD_2321, ADD_2322, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2320, ADD_2321, ADD_2322, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2320, ADD_2321, ADD_2322, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2320, ADD_2321, ADD_2322, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2320, ADD_2321, ADD_2322, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2320, ADD_2321, ADD_2322, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2353, ADD_2354, ADD_2355, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2353, ADD_2354, ADD_2355, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2353, ADD_2354, ADD_2355, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2353, ADD_2354, ADD_2355, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2353, ADD_2354, ADD_2355, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2353, ADD_2354, ADD_2355, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2392, ADD_2393, ADD_2394, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2392, ADD_2393, ADD_2394, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2392, ADD_2393, ADD_2394, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2392, ADD_2393, ADD_2394, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2392, ADD_2393, ADD_2394, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2392, ADD_2393, ADD_2394, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2425, ADD_2426, ADD_2427, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2425, ADD_2426, ADD_2427, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2425, ADD_2426, ADD_2427, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2425, ADD_2426, ADD_2427, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2425, ADD_2426, ADD_2427, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2425, ADD_2426, ADD_2427, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2464, ADD_2465, ADD_2466, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2464, ADD_2465, ADD_2466, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2464, ADD_2465, ADD_2466, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2464, ADD_2465, ADD_2466, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2464, ADD_2465, ADD_2466, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2464, ADD_2465, ADD_2466, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2485, ADD_2486, ADD_2487, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2485, ADD_2486, ADD_2487, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2485, ADD_2486, ADD_2487, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2485, ADD_2486, ADD_2487, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2485, ADD_2486, ADD_2487, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2485, ADD_2486, ADD_2487, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
        }  // (308, 308)
        if (/*upperarm_roll_link*/ sphere_environment_in_collision(
            environment, ADD_2251, ADD_2252, ADD_2253, 0.134))
        {
            if (sphere_environment_in_collision(environment, SUB_2278, SUB_2279, SUB_2280, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2299, ADD_2300, ADD_2301, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2320, ADD_2321, ADD_2322, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2353, ADD_2354, ADD_2355, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2392, ADD_2393, ADD_2394, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2425, ADD_2426, ADD_2427, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2464, ADD_2465, ADD_2466, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2485, ADD_2486, ADD_2487, 0.055))
            {
                return false;
            }
        }  // (308, 308)
        auto MUL_423 = SUB_345 * 0.133;
        auto MUL_428 = SUB_345 * MUL_423;
        auto MUL_418 = ADD_339 * 0.133;
        auto MUL_426 = ADD_339 * MUL_418;
        auto ADD_430 = MUL_426 + MUL_428;
        auto MUL_433 = ADD_430 * 2.0;
        auto SUB_436 = 0.133 - MUL_433;
        auto ADD_455 = ADD_317 + SUB_436;
        auto INPUT_4 = q[4];
        auto DIV_459 = INPUT_4 * 0.5;
        auto SIN_460 = sin(DIV_459);
        auto COS_466 = cos(DIV_459);
        auto MUL_482 = ADD_349 * COS_466;
        auto MUL_472 = ADD_349 * SIN_460;
        auto MUL_480 = SUB_345 * COS_466;
        auto MUL_470 = SUB_345 * SIN_460;
        auto MUL_474 = ADD_339 * COS_466;
        auto ADD_475 = MUL_472 + MUL_474;
        auto MUL_2506 = ADD_475 * ADD_475;
        auto MUL_484 = ADD_339 * SIN_460;
        auto SUB_485 = MUL_482 - MUL_484;
        auto MUL_468 = SUB_332 * COS_466;
        auto SUB_471 = MUL_468 - MUL_470;
        auto MUL_2512 = SUB_471 * ADD_475;
        auto MUL_478 = SUB_332 * SIN_460;
        auto ADD_481 = MUL_478 + MUL_480;
        auto MUL_2508 = SUB_485 * ADD_481;
        auto SUB_2528 = MUL_2512 - MUL_2508;
        auto MUL_2530 = SUB_2528 * 2.0;
        auto MUL_2561 = MUL_2530 * 0.021;
        auto MUL_2507 = ADD_481 * ADD_481;
        auto ADD_2515 = MUL_2506 + MUL_2507;
        auto MUL_2518 = ADD_2515 * 2.0;
        auto SUB_2521 = 1.0 - MUL_2518;
        auto MUL_2555 = SUB_2521 * 0.071;
        auto ADD_2572 = MUL_2555 + MUL_2561;
        auto ADD_2575 = ADD_455 + ADD_2572;
        auto ADD_2522 = MUL_2512 + MUL_2508;
        auto MUL_2524 = ADD_2522 * 2.0;
        auto MUL_2557 = MUL_2524 * 0.071;
        auto MUL_2510 = SUB_471 * SUB_471;
        auto ADD_2531 = MUL_2507 + MUL_2510;
        auto MUL_2534 = ADD_2531 * 2.0;
        auto SUB_2537 = 1.0 - MUL_2534;
        auto MUL_2563 = SUB_2537 * 0.021;
        auto ADD_2573 = MUL_2557 + MUL_2563;
        auto MUL_438 = ADD_349 * MUL_423;
        auto MUL_439 = SUB_332 * MUL_418;
        auto ADD_441 = MUL_438 + MUL_439;
        auto MUL_444 = ADD_441 * 2.0;
        auto ADD_456 = ADD_318 + MUL_444;
        auto ADD_2576 = ADD_456 + ADD_2573;
        auto MUL_2509 = SUB_485 * ADD_475;
        auto MUL_2511 = SUB_485 * SUB_471;
        auto MUL_2514 = ADD_475 * ADD_481;
        auto ADD_2538 = MUL_2514 + MUL_2511;
        auto MUL_2540 = ADD_2538 * 2.0;
        auto MUL_2565 = MUL_2540 * 0.021;
        auto MUL_2513 = SUB_471 * ADD_481;
        auto SUB_2525 = MUL_2513 - MUL_2509;
        auto MUL_2527 = SUB_2525 * 2.0;
        auto MUL_2559 = MUL_2527 * 0.071;
        auto ADD_2574 = MUL_2559 + MUL_2565;
        auto MUL_446 = ADD_349 * MUL_418;
        auto MUL_448 = SUB_332 * MUL_423;
        auto SUB_450 = MUL_448 - MUL_446;
        auto MUL_453 = SUB_450 * 2.0;
        auto ADD_457 = SUB_319 + MUL_453;
        auto ADD_2577 = ADD_457 + ADD_2574;
        auto ADD_2541 = MUL_2513 + MUL_2509;
        auto MUL_2543 = ADD_2541 * 2.0;
        auto MUL_2591 = MUL_2543 * 0.02;
        auto MUL_2585 = MUL_2530 * 0.045;
        auto MUL_2579 = SUB_2521 * 0.02;
        auto ADD_2596 = MUL_2579 + MUL_2585;
        auto ADD_2599 = ADD_2596 + MUL_2591;
        auto ADD_2602 = ADD_455 + ADD_2599;
        auto SUB_2544 = MUL_2514 - MUL_2511;
        auto MUL_2546 = SUB_2544 * 2.0;
        auto MUL_2593 = MUL_2546 * 0.02;
        auto MUL_2587 = SUB_2537 * 0.045;
        auto MUL_2581 = MUL_2524 * 0.02;
        auto ADD_2597 = MUL_2581 + MUL_2587;
        auto ADD_2600 = ADD_2597 + MUL_2593;
        auto ADD_2603 = ADD_456 + ADD_2600;
        auto ADD_2547 = MUL_2506 + MUL_2510;
        auto MUL_2550 = ADD_2547 * 2.0;
        auto SUB_2553 = 1.0 - MUL_2550;
        auto MUL_2595 = SUB_2553 * 0.02;
        auto MUL_2589 = MUL_2540 * 0.045;
        auto MUL_2583 = MUL_2527 * 0.02;
        auto ADD_2598 = MUL_2583 + MUL_2589;
        auto ADD_2601 = ADD_2598 + MUL_2595;
        auto ADD_2604 = ADD_457 + ADD_2601;
        auto SUB_2632 = ADD_2596 - MUL_2591;
        auto ADD_2635 = ADD_455 + SUB_2632;
        auto SUB_2633 = ADD_2597 - MUL_2593;
        auto ADD_2636 = ADD_456 + SUB_2633;
        auto SUB_2634 = ADD_2598 - MUL_2595;
        auto ADD_2637 = ADD_457 + SUB_2634;
        auto SUB_2662 = MUL_2585 - MUL_2579;
        auto ADD_2665 = SUB_2662 + MUL_2591;
        auto ADD_2668 = ADD_455 + ADD_2665;
        auto SUB_2663 = MUL_2587 - MUL_2581;
        auto ADD_2666 = SUB_2663 + MUL_2593;
        auto ADD_2669 = ADD_456 + ADD_2666;
        auto SUB_2664 = MUL_2589 - MUL_2583;
        auto ADD_2667 = SUB_2664 + MUL_2595;
        auto ADD_2670 = ADD_457 + ADD_2667;
        auto SUB_2704 = SUB_2662 - MUL_2591;
        auto ADD_2707 = ADD_455 + SUB_2704;
        auto SUB_2705 = SUB_2663 - MUL_2593;
        auto ADD_2708 = ADD_456 + SUB_2705;
        auto SUB_2706 = SUB_2664 - MUL_2595;
        auto ADD_2709 = ADD_457 + SUB_2706;
        auto MUL_2711 = SUB_2521 * 0.08;
        auto ADD_2728 = ADD_455 + MUL_2711;
        auto MUL_2713 = MUL_2524 * 0.08;
        auto ADD_2729 = ADD_456 + MUL_2713;
        auto MUL_2715 = MUL_2527 * 0.08;
        auto ADD_2730 = ADD_457 + MUL_2715;
        auto MUL_2732 = SUB_2521 * 0.14;
        auto ADD_2749 = ADD_455 + MUL_2732;
        auto MUL_2734 = MUL_2524 * 0.14;
        auto ADD_2750 = ADD_456 + MUL_2734;
        auto MUL_2736 = MUL_2527 * 0.14;
        auto ADD_2751 = ADD_457 + MUL_2736;
        if (/*base_link vs. elbow_flex_link*/ sphere_sphere_self_collision(
            -0.02, 0.0, 0.188, 0.34, ADD_2575, ADD_2576, ADD_2577, 0.127))
        {
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
        }  // (308, 435)
        if (/*elbow_flex_link vs. torso_fixed_link*/ sphere_sphere_self_collision(
            ADD_2575, ADD_2576, ADD_2577, 0.127, -0.186875, 0.0, 0.587425, 0.277))
        {
            if (sphere_sphere_self_collision(
                    ADD_2602, ADD_2603, ADD_2604, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2602, ADD_2603, ADD_2604, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2602, ADD_2603, ADD_2604, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2602, ADD_2603, ADD_2604, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2602, ADD_2603, ADD_2604, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2602, ADD_2603, ADD_2604, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2635, ADD_2636, ADD_2637, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2635, ADD_2636, ADD_2637, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2635, ADD_2636, ADD_2637, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2635, ADD_2636, ADD_2637, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2635, ADD_2636, ADD_2637, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2635, ADD_2636, ADD_2637, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2668, ADD_2669, ADD_2670, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2668, ADD_2669, ADD_2670, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2668, ADD_2669, ADD_2670, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2668, ADD_2669, ADD_2670, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2668, ADD_2669, ADD_2670, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2668, ADD_2669, ADD_2670, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2707, ADD_2708, ADD_2709, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2707, ADD_2708, ADD_2709, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2707, ADD_2708, ADD_2709, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2707, ADD_2708, ADD_2709, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2707, ADD_2708, ADD_2709, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2707, ADD_2708, ADD_2709, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2728, ADD_2729, ADD_2730, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2728, ADD_2729, ADD_2730, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2728, ADD_2729, ADD_2730, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2728, ADD_2729, ADD_2730, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2728, ADD_2729, ADD_2730, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2728, ADD_2729, ADD_2730, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2749, ADD_2750, ADD_2751, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2749, ADD_2750, ADD_2751, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2749, ADD_2750, ADD_2751, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2749, ADD_2750, ADD_2751, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2749, ADD_2750, ADD_2751, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2749, ADD_2750, ADD_2751, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
        }  // (435, 435)
        if (/*elbow_flex_link*/ sphere_environment_in_collision(
            environment, ADD_2575, ADD_2576, ADD_2577, 0.127))
        {
            if (sphere_environment_in_collision(environment, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
        }  // (435, 435)
        if (/*head_pan_link vs. elbow_flex_link*/ sphere_sphere_self_collision(
            0.01325, 0.0, ADD_1497, 0.197, ADD_2575, ADD_2576, ADD_2577, 0.127))
        {
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
        }  // (435, 435)
        if (/*torso_lift_link vs. elbow_flex_link*/ sphere_sphere_self_collision(
            -0.186875, 0.0, ADD_1421, 0.308, ADD_2575, ADD_2576, ADD_2577, 0.127))
        {
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
        }  // (435, 435)
        if (/*torso_lift_link_collision_2 vs. elbow_flex_link*/ sphere_sphere_self_collision(
            0.013125, 0.0, ADD_1490, 0.07, ADD_2575, ADD_2576, ADD_2577, 0.127))
        {
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2602, ADD_2603, ADD_2604, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2635, ADD_2636, ADD_2637, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2668, ADD_2669, ADD_2670, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2707, ADD_2708, ADD_2709, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2728, ADD_2729, ADD_2730, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2749, ADD_2750, ADD_2751, 0.055))
            {
                return false;
            }
        }  // (435, 435)
        auto MUL_558 = ADD_481 * 0.197;
        auto MUL_563 = ADD_481 * MUL_558;
        auto MUL_553 = ADD_475 * 0.197;
        auto MUL_561 = ADD_475 * MUL_553;
        auto ADD_565 = MUL_561 + MUL_563;
        auto MUL_568 = ADD_565 * 2.0;
        auto SUB_571 = 0.197 - MUL_568;
        auto ADD_590 = ADD_455 + SUB_571;
        auto INPUT_5 = q[5];
        auto DIV_594 = INPUT_5 * 0.5;
        auto SIN_595 = sin(DIV_594);
        auto COS_601 = cos(DIV_594);
        auto MUL_618 = SUB_485 * COS_601;
        auto MUL_602 = SUB_485 * SIN_595;
        auto MUL_616 = ADD_481 * COS_601;
        auto MUL_610 = ADD_481 * SIN_595;
        auto MUL_609 = ADD_475 * COS_601;
        auto ADD_611 = MUL_609 + MUL_610;
        auto MUL_2766 = ADD_611 * ADD_611;
        auto MUL_614 = ADD_475 * SIN_595;
        auto SUB_617 = MUL_616 - MUL_614;
        auto MUL_2767 = SUB_617 * SUB_617;
        auto ADD_2775 = MUL_2766 + MUL_2767;
        auto MUL_2778 = ADD_2775 * 2.0;
        auto SUB_2781 = 1.0 - MUL_2778;
        auto MUL_2815 = SUB_2781 * 0.064;
        auto MUL_603 = SUB_471 * COS_601;
        auto ADD_604 = MUL_602 + MUL_603;
        auto MUL_2772 = ADD_604 * ADD_611;
        auto MUL_619 = SUB_471 * SIN_595;
        auto SUB_620 = MUL_618 - MUL_619;
        auto MUL_2768 = SUB_620 * SUB_617;
        auto SUB_2788 = MUL_2772 - MUL_2768;
        auto MUL_2790 = SUB_2788 * 2.0;
        auto MUL_2822 = MUL_2790 * 0.026;
        auto SUB_2838 = MUL_2815 - MUL_2822;
        auto ADD_2841 = ADD_590 + SUB_2838;
        auto ADD_2782 = MUL_2772 + MUL_2768;
        auto MUL_2784 = ADD_2782 * 2.0;
        auto MUL_2817 = MUL_2784 * 0.064;
        auto MUL_2770 = ADD_604 * ADD_604;
        auto ADD_2791 = MUL_2767 + MUL_2770;
        auto MUL_2794 = ADD_2791 * 2.0;
        auto SUB_2797 = 1.0 - MUL_2794;
        auto MUL_2826 = SUB_2797 * 0.026;
        auto SUB_2839 = MUL_2817 - MUL_2826;
        auto MUL_573 = SUB_485 * MUL_558;
        auto MUL_574 = SUB_471 * MUL_553;
        auto ADD_576 = MUL_573 + MUL_574;
        auto MUL_579 = ADD_576 * 2.0;
        auto ADD_591 = ADD_456 + MUL_579;
        auto ADD_2842 = ADD_591 + SUB_2839;
        auto MUL_2769 = SUB_620 * ADD_611;
        auto MUL_2771 = SUB_620 * ADD_604;
        auto MUL_2774 = ADD_611 * SUB_617;
        auto ADD_2798 = MUL_2774 + MUL_2771;
        auto MUL_2800 = ADD_2798 * 2.0;
        auto MUL_2830 = MUL_2800 * 0.026;
        auto MUL_2773 = ADD_604 * SUB_617;
        auto SUB_2785 = MUL_2773 - MUL_2769;
        auto MUL_2787 = SUB_2785 * 2.0;
        auto MUL_2819 = MUL_2787 * 0.064;
        auto SUB_2840 = MUL_2819 - MUL_2830;
        auto MUL_581 = SUB_485 * MUL_553;
        auto MUL_583 = SUB_471 * MUL_558;
        auto SUB_585 = MUL_583 - MUL_581;
        auto MUL_588 = SUB_585 * 2.0;
        auto ADD_592 = ADD_457 + MUL_588;
        auto ADD_2843 = ADD_592 + SUB_2840;
        auto ADD_2801 = MUL_2773 + MUL_2769;
        auto MUL_2803 = ADD_2801 * 2.0;
        auto MUL_2881 = MUL_2803 * 0.02;
        auto MUL_2870 = MUL_2790 * 0.06;
        auto MUL_2863 = SUB_2781 * 0.05;
        auto SUB_2886 = MUL_2863 - MUL_2870;
        auto ADD_2889 = SUB_2886 + MUL_2881;
        auto ADD_2892 = ADD_590 + ADD_2889;
        auto SUB_2804 = MUL_2774 - MUL_2771;
        auto MUL_2806 = SUB_2804 * 2.0;
        auto MUL_2883 = MUL_2806 * 0.02;
        auto MUL_2874 = SUB_2797 * 0.06;
        auto MUL_2865 = MUL_2784 * 0.05;
        auto SUB_2887 = MUL_2865 - MUL_2874;
        auto ADD_2890 = SUB_2887 + MUL_2883;
        auto ADD_2893 = ADD_591 + ADD_2890;
        auto ADD_2807 = MUL_2766 + MUL_2770;
        auto MUL_2810 = ADD_2807 * 2.0;
        auto SUB_2813 = 1.0 - MUL_2810;
        auto MUL_2885 = SUB_2813 * 0.02;
        auto MUL_2878 = MUL_2800 * 0.06;
        auto MUL_2867 = MUL_2787 * 0.05;
        auto SUB_2888 = MUL_2867 - MUL_2878;
        auto ADD_2891 = SUB_2888 + MUL_2885;
        auto ADD_2894 = ADD_592 + ADD_2891;
        auto SUB_2928 = SUB_2886 - MUL_2881;
        auto ADD_2931 = ADD_590 + SUB_2928;
        auto SUB_2929 = SUB_2887 - MUL_2883;
        auto ADD_2932 = ADD_591 + SUB_2929;
        auto SUB_2930 = SUB_2888 - MUL_2885;
        auto ADD_2933 = ADD_592 + SUB_2930;
        auto MUL_2935 = SUB_2781 * 0.1;
        auto SUB_2958 = MUL_2935 - MUL_2870;
        auto ADD_2961 = SUB_2958 + MUL_2881;
        auto ADD_2964 = ADD_590 + ADD_2961;
        auto MUL_2937 = MUL_2784 * 0.1;
        auto SUB_2959 = MUL_2937 - MUL_2874;
        auto ADD_2962 = SUB_2959 + MUL_2883;
        auto ADD_2965 = ADD_591 + ADD_2962;
        auto MUL_2939 = MUL_2787 * 0.1;
        auto SUB_2960 = MUL_2939 - MUL_2878;
        auto ADD_2963 = SUB_2960 + MUL_2885;
        auto ADD_2966 = ADD_592 + ADD_2963;
        auto SUB_3000 = SUB_2958 - MUL_2881;
        auto ADD_3003 = ADD_590 + SUB_3000;
        auto SUB_3001 = SUB_2959 - MUL_2883;
        auto ADD_3004 = ADD_591 + SUB_3001;
        auto SUB_3002 = SUB_2960 - MUL_2885;
        auto ADD_3005 = ADD_592 + SUB_3002;
        auto MUL_3007 = SUB_2781 * 0.15;
        auto SUB_3030 = MUL_3007 - MUL_2870;
        auto ADD_3033 = SUB_3030 + MUL_2881;
        auto ADD_3036 = ADD_590 + ADD_3033;
        auto MUL_3009 = MUL_2784 * 0.15;
        auto SUB_3031 = MUL_3009 - MUL_2874;
        auto ADD_3034 = SUB_3031 + MUL_2883;
        auto ADD_3037 = ADD_591 + ADD_3034;
        auto MUL_3011 = MUL_2787 * 0.15;
        auto SUB_3032 = MUL_3011 - MUL_2878;
        auto ADD_3035 = SUB_3032 + MUL_2885;
        auto ADD_3038 = ADD_592 + ADD_3035;
        auto SUB_3072 = SUB_3030 - MUL_2881;
        auto ADD_3075 = ADD_590 + SUB_3072;
        auto SUB_3073 = SUB_3031 - MUL_2883;
        auto ADD_3076 = ADD_591 + SUB_3073;
        auto SUB_3074 = SUB_3032 - MUL_2885;
        auto ADD_3077 = ADD_592 + SUB_3074;
        if (/*base_link vs. forearm_roll_link*/ sphere_sphere_self_collision(
            -0.02, 0.0, 0.188, 0.34, ADD_2841, ADD_2842, ADD_2843, 0.124))
        {
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
        }  // (435, 571)
        if (/*forearm_roll_link vs. torso_fixed_link*/ sphere_sphere_self_collision(
            ADD_2841, ADD_2842, ADD_2843, 0.124, -0.186875, 0.0, 0.587425, 0.277))
        {
            if (sphere_sphere_self_collision(
                    ADD_590, ADD_591, ADD_592, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_590, ADD_591, ADD_592, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_590, ADD_591, ADD_592, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_590, ADD_591, ADD_592, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_590, ADD_591, ADD_592, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_590, ADD_591, ADD_592, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2892, ADD_2893, ADD_2894, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2892, ADD_2893, ADD_2894, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2892, ADD_2893, ADD_2894, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2892, ADD_2893, ADD_2894, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2892, ADD_2893, ADD_2894, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2892, ADD_2893, ADD_2894, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2931, ADD_2932, ADD_2933, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2931, ADD_2932, ADD_2933, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2931, ADD_2932, ADD_2933, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2931, ADD_2932, ADD_2933, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2931, ADD_2932, ADD_2933, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2931, ADD_2932, ADD_2933, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2964, ADD_2965, ADD_2966, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2964, ADD_2965, ADD_2966, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2964, ADD_2965, ADD_2966, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2964, ADD_2965, ADD_2966, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2964, ADD_2965, ADD_2966, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2964, ADD_2965, ADD_2966, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3003, ADD_3004, ADD_3005, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3003, ADD_3004, ADD_3005, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3003, ADD_3004, ADD_3005, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3003, ADD_3004, ADD_3005, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3003, ADD_3004, ADD_3005, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3003, ADD_3004, ADD_3005, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3036, ADD_3037, ADD_3038, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3036, ADD_3037, ADD_3038, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3036, ADD_3037, ADD_3038, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3036, ADD_3037, ADD_3038, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3036, ADD_3037, ADD_3038, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3036, ADD_3037, ADD_3038, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3075, ADD_3076, ADD_3077, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3075, ADD_3076, ADD_3077, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3075, ADD_3076, ADD_3077, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3075, ADD_3076, ADD_3077, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3075, ADD_3076, ADD_3077, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3075, ADD_3076, ADD_3077, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
        }  // (571, 571)
        if (/*forearm_roll_link*/ sphere_environment_in_collision(
            environment, ADD_2841, ADD_2842, ADD_2843, 0.124))
        {
            if (sphere_environment_in_collision(environment, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
        }  // (571, 571)
        if (/*head_pan_link vs. forearm_roll_link*/ sphere_sphere_self_collision(
            0.01325, 0.0, ADD_1497, 0.197, ADD_2841, ADD_2842, ADD_2843, 0.124))
        {
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
        }  // (571, 571)
        if (/*torso_lift_link vs. forearm_roll_link*/ sphere_sphere_self_collision(
            -0.186875, 0.0, ADD_1421, 0.308, ADD_2841, ADD_2842, ADD_2843, 0.124))
        {
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
        }  // (571, 571)
        if (/*torso_lift_link_collision_2 vs. forearm_roll_link*/ sphere_sphere_self_collision(0.013125, 0.0, ADD_1490, 0.07, ADD_2841, ADD_2842, ADD_2843, 0.124))
        {
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_590, ADD_591, ADD_592, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2892, ADD_2893, ADD_2894, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2931, ADD_2932, ADD_2933, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_2964, ADD_2965, ADD_2966, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3003, ADD_3004, ADD_3005, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3036, ADD_3037, ADD_3038, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3075, ADD_3076, ADD_3077, 0.03))
            {
                return false;
            }
        }  // (571, 571)
        auto MUL_694 = SUB_617 * 0.1245;
        auto MUL_699 = SUB_617 * MUL_694;
        auto MUL_689 = ADD_611 * 0.1245;
        auto MUL_697 = ADD_611 * MUL_689;
        auto ADD_701 = MUL_697 + MUL_699;
        auto MUL_704 = ADD_701 * 2.0;
        auto SUB_707 = 0.1245 - MUL_704;
        auto ADD_726 = ADD_590 + SUB_707;
        auto INPUT_6 = q[6];
        auto DIV_730 = INPUT_6 * 0.5;
        auto SIN_731 = sin(DIV_730);
        auto COS_737 = cos(DIV_730);
        auto MUL_753 = SUB_620 * COS_737;
        auto MUL_743 = SUB_620 * SIN_731;
        auto MUL_751 = SUB_617 * COS_737;
        auto MUL_741 = SUB_617 * SIN_731;
        auto MUL_745 = ADD_611 * COS_737;
        auto ADD_746 = MUL_743 + MUL_745;
        auto MUL_3094 = ADD_746 * ADD_746;
        auto MUL_755 = ADD_611 * SIN_731;
        auto SUB_756 = MUL_753 - MUL_755;
        auto MUL_739 = ADD_604 * COS_737;
        auto SUB_742 = MUL_739 - MUL_741;
        auto MUL_3100 = SUB_742 * ADD_746;
        auto MUL_749 = ADD_604 * SIN_731;
        auto ADD_752 = MUL_749 + MUL_751;
        auto MUL_3096 = SUB_756 * ADD_752;
        auto SUB_3116 = MUL_3100 - MUL_3096;
        auto MUL_3118 = SUB_3116 * 2.0;
        auto MUL_3149 = MUL_3118 * 0.017;
        auto MUL_3095 = ADD_752 * ADD_752;
        auto ADD_3103 = MUL_3094 + MUL_3095;
        auto MUL_3106 = ADD_3103 * 2.0;
        auto SUB_3109 = 1.0 - MUL_3106;
        auto MUL_3143 = SUB_3109 * 0.029;
        auto ADD_3160 = MUL_3143 + MUL_3149;
        auto ADD_3163 = ADD_726 + ADD_3160;
        auto ADD_3110 = MUL_3100 + MUL_3096;
        auto MUL_3112 = ADD_3110 * 2.0;
        auto MUL_3145 = MUL_3112 * 0.029;
        auto MUL_3098 = SUB_742 * SUB_742;
        auto ADD_3119 = MUL_3095 + MUL_3098;
        auto MUL_3122 = ADD_3119 * 2.0;
        auto SUB_3125 = 1.0 - MUL_3122;
        auto MUL_3151 = SUB_3125 * 0.017;
        auto ADD_3161 = MUL_3145 + MUL_3151;
        auto MUL_709 = SUB_620 * MUL_694;
        auto MUL_710 = ADD_604 * MUL_689;
        auto ADD_712 = MUL_709 + MUL_710;
        auto MUL_715 = ADD_712 * 2.0;
        auto ADD_727 = ADD_591 + MUL_715;
        auto ADD_3164 = ADD_727 + ADD_3161;
        auto MUL_3097 = SUB_756 * ADD_746;
        auto MUL_3099 = SUB_756 * SUB_742;
        auto MUL_3102 = ADD_746 * ADD_752;
        auto ADD_3126 = MUL_3102 + MUL_3099;
        auto MUL_3128 = ADD_3126 * 2.0;
        auto MUL_3153 = MUL_3128 * 0.017;
        auto MUL_3101 = SUB_742 * ADD_752;
        auto SUB_3113 = MUL_3101 - MUL_3097;
        auto MUL_3115 = SUB_3113 * 2.0;
        auto MUL_3147 = MUL_3115 * 0.029;
        auto ADD_3162 = MUL_3147 + MUL_3153;
        auto MUL_717 = SUB_620 * MUL_689;
        auto MUL_719 = ADD_604 * MUL_694;
        auto SUB_721 = MUL_719 - MUL_717;
        auto MUL_724 = SUB_721 * 2.0;
        auto ADD_728 = ADD_592 + MUL_724;
        auto ADD_3165 = ADD_728 + ADD_3162;
        auto MUL_3185 = SUB_3109 * 0.06;
        auto ADD_3202 = ADD_726 + MUL_3185;
        auto MUL_3187 = MUL_3112 * 0.06;
        auto ADD_3203 = ADD_727 + MUL_3187;
        auto MUL_3189 = MUL_3115 * 0.06;
        auto ADD_3204 = ADD_728 + MUL_3189;
        auto ADD_3129 = MUL_3101 + MUL_3097;
        auto MUL_3131 = ADD_3129 * 2.0;
        auto MUL_3218 = MUL_3131 * 0.02;
        auto MUL_3212 = MUL_3118 * 0.045;
        auto MUL_3206 = SUB_3109 * 0.02;
        auto ADD_3223 = MUL_3206 + MUL_3212;
        auto ADD_3226 = ADD_3223 + MUL_3218;
        auto ADD_3229 = ADD_726 + ADD_3226;
        auto SUB_3132 = MUL_3102 - MUL_3099;
        auto MUL_3134 = SUB_3132 * 2.0;
        auto MUL_3220 = MUL_3134 * 0.02;
        auto MUL_3214 = SUB_3125 * 0.045;
        auto MUL_3208 = MUL_3112 * 0.02;
        auto ADD_3224 = MUL_3208 + MUL_3214;
        auto ADD_3227 = ADD_3224 + MUL_3220;
        auto ADD_3230 = ADD_727 + ADD_3227;
        auto ADD_3135 = MUL_3094 + MUL_3098;
        auto MUL_3138 = ADD_3135 * 2.0;
        auto SUB_3141 = 1.0 - MUL_3138;
        auto MUL_3222 = SUB_3141 * 0.02;
        auto MUL_3216 = MUL_3128 * 0.045;
        auto MUL_3210 = MUL_3115 * 0.02;
        auto ADD_3225 = MUL_3210 + MUL_3216;
        auto ADD_3228 = ADD_3225 + MUL_3222;
        auto ADD_3231 = ADD_728 + ADD_3228;
        auto SUB_3259 = ADD_3223 - MUL_3218;
        auto ADD_3262 = ADD_726 + SUB_3259;
        auto SUB_3260 = ADD_3224 - MUL_3220;
        auto ADD_3263 = ADD_727 + SUB_3260;
        auto SUB_3261 = ADD_3225 - MUL_3222;
        auto ADD_3264 = ADD_728 + SUB_3261;
        auto SUB_3289 = MUL_3212 - MUL_3206;
        auto ADD_3292 = SUB_3289 + MUL_3218;
        auto ADD_3295 = ADD_726 + ADD_3292;
        auto SUB_3290 = MUL_3214 - MUL_3208;
        auto ADD_3293 = SUB_3290 + MUL_3220;
        auto ADD_3296 = ADD_727 + ADD_3293;
        auto SUB_3291 = MUL_3216 - MUL_3210;
        auto ADD_3294 = SUB_3291 + MUL_3222;
        auto ADD_3297 = ADD_728 + ADD_3294;
        auto SUB_3331 = SUB_3289 - MUL_3218;
        auto ADD_3334 = ADD_726 + SUB_3331;
        auto SUB_3332 = SUB_3290 - MUL_3220;
        auto ADD_3335 = ADD_727 + SUB_3332;
        auto SUB_3333 = SUB_3291 - MUL_3222;
        auto ADD_3336 = ADD_728 + SUB_3333;
        if (/*base_link vs. wrist_flex_link*/ sphere_sphere_self_collision(
            -0.02, 0.0, 0.188, 0.34, ADD_3163, ADD_3164, ADD_3165, 0.09))
        {
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
        }  // (571, 692)
        if (/*head_pan_link vs. wrist_flex_link*/ sphere_sphere_self_collision(
            0.01325, 0.0, ADD_1497, 0.197, ADD_3163, ADD_3164, ADD_3165, 0.09))
        {
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
        }  // (692, 692)
        if (/*torso_lift_link vs. wrist_flex_link*/ sphere_sphere_self_collision(
            -0.186875, 0.0, ADD_1421, 0.308, ADD_3163, ADD_3164, ADD_3165, 0.09))
        {
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
        }  // (692, 692)
        if (/*torso_lift_link_collision_2 vs. wrist_flex_link*/ sphere_sphere_self_collision(
            0.013125, 0.0, ADD_1490, 0.07, ADD_3163, ADD_3164, ADD_3165, 0.09))
        {
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
        }  // (692, 692)
        if (/*wrist_flex_link vs. torso_fixed_link*/ sphere_sphere_self_collision(
            ADD_3163, ADD_3164, ADD_3165, 0.09, -0.186875, 0.0, 0.587425, 0.277))
        {
            if (sphere_sphere_self_collision(
                    ADD_726, ADD_727, ADD_728, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_726, ADD_727, ADD_728, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_726, ADD_727, ADD_728, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_726, ADD_727, ADD_728, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_726, ADD_727, ADD_728, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_726, ADD_727, ADD_728, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3202, ADD_3203, ADD_3204, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3202, ADD_3203, ADD_3204, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3202, ADD_3203, ADD_3204, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3202, ADD_3203, ADD_3204, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3202, ADD_3203, ADD_3204, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3202, ADD_3203, ADD_3204, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3229, ADD_3230, ADD_3231, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3229, ADD_3230, ADD_3231, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3229, ADD_3230, ADD_3231, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3229, ADD_3230, ADD_3231, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3229, ADD_3230, ADD_3231, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3229, ADD_3230, ADD_3231, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3262, ADD_3263, ADD_3264, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3262, ADD_3263, ADD_3264, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3262, ADD_3263, ADD_3264, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3262, ADD_3263, ADD_3264, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3262, ADD_3263, ADD_3264, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3262, ADD_3263, ADD_3264, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3295, ADD_3296, ADD_3297, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3295, ADD_3296, ADD_3297, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3295, ADD_3296, ADD_3297, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3295, ADD_3296, ADD_3297, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3295, ADD_3296, ADD_3297, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3295, ADD_3296, ADD_3297, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3334, ADD_3335, ADD_3336, 0.03, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3334, ADD_3335, ADD_3336, 0.03, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3334, ADD_3335, ADD_3336, 0.03, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3334, ADD_3335, ADD_3336, 0.03, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3334, ADD_3335, ADD_3336, 0.03, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3334, ADD_3335, ADD_3336, 0.03, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
        }  // (692, 692)
        if (/*wrist_flex_link*/ sphere_environment_in_collision(
            environment, ADD_3163, ADD_3164, ADD_3165, 0.09))
        {
            if (sphere_environment_in_collision(environment, ADD_726, ADD_727, ADD_728, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3202, ADD_3203, ADD_3204, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3229, ADD_3230, ADD_3231, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3262, ADD_3263, ADD_3264, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3295, ADD_3296, ADD_3297, 0.03))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3334, ADD_3335, ADD_3336, 0.03))
            {
                return false;
            }
        }  // (692, 692)
        auto MUL_829 = ADD_752 * 0.1385;
        auto MUL_834 = ADD_752 * MUL_829;
        auto MUL_824 = ADD_746 * 0.1385;
        auto MUL_832 = ADD_746 * MUL_824;
        auto ADD_836 = MUL_832 + MUL_834;
        auto MUL_839 = ADD_836 * 2.0;
        auto SUB_842 = 0.1385 - MUL_839;
        auto ADD_861 = ADD_726 + SUB_842;
        auto INPUT_7 = q[7];
        auto DIV_865 = INPUT_7 * 0.5;
        auto SIN_866 = sin(DIV_865);
        auto COS_872 = cos(DIV_865);
        auto MUL_887 = ADD_752 * COS_872;
        auto MUL_881 = ADD_752 * SIN_866;
        auto MUL_880 = ADD_746 * COS_872;
        auto ADD_882 = MUL_880 + MUL_881;
        auto MUL_3351 = ADD_882 * ADD_882;
        auto MUL_885 = ADD_746 * SIN_866;
        auto SUB_888 = MUL_887 - MUL_885;
        auto MUL_3352 = SUB_888 * SUB_888;
        auto ADD_3360 = MUL_3351 + MUL_3352;
        auto MUL_3363 = ADD_3360 * 2.0;
        auto SUB_3366 = 1.0 - MUL_3363;
        auto MUL_3401 = SUB_3366 * 0.015;
        auto SUB_3423 = ADD_861 - MUL_3401;
        auto MUL_889 = SUB_756 * COS_872;
        auto MUL_873 = SUB_756 * SIN_866;
        auto MUL_844 = SUB_756 * MUL_829;
        auto MUL_874 = SUB_742 * COS_872;
        auto ADD_875 = MUL_873 + MUL_874;
        auto MUL_3357 = ADD_875 * ADD_882;
        auto MUL_890 = SUB_742 * SIN_866;
        auto SUB_891 = MUL_889 - MUL_890;
        auto MUL_3353 = SUB_891 * SUB_888;
        auto ADD_3367 = MUL_3357 + MUL_3353;
        auto MUL_3369 = ADD_3367 * 2.0;
        auto MUL_3405 = MUL_3369 * 0.015;
        auto MUL_845 = SUB_742 * MUL_824;
        auto ADD_847 = MUL_844 + MUL_845;
        auto MUL_850 = ADD_847 * 2.0;
        auto ADD_862 = ADD_727 + MUL_850;
        auto SUB_3424 = ADD_862 - MUL_3405;
        auto MUL_3354 = SUB_891 * ADD_882;
        auto MUL_3358 = ADD_875 * SUB_888;
        auto SUB_3370 = MUL_3358 - MUL_3354;
        auto MUL_3372 = SUB_3370 * 2.0;
        auto MUL_3409 = MUL_3372 * 0.015;
        auto MUL_852 = SUB_756 * MUL_824;
        auto MUL_854 = SUB_742 * MUL_829;
        auto SUB_856 = MUL_854 - MUL_852;
        auto MUL_859 = SUB_856 * 2.0;
        auto ADD_863 = ADD_728 + MUL_859;
        auto SUB_3425 = ADD_863 - MUL_3409;
        auto MUL_3428 = SUB_3366 * 0.03;
        auto SUB_3450 = ADD_861 - MUL_3428;
        auto MUL_3432 = MUL_3369 * 0.03;
        auto SUB_3451 = ADD_862 - MUL_3432;
        auto MUL_3436 = MUL_3372 * 0.03;
        auto SUB_3452 = ADD_863 - MUL_3436;
        if (/*base_link vs. wrist_roll_link*/ sphere_sphere_self_collision(
            -0.02, 0.0, 0.188, 0.34, SUB_3423, SUB_3424, SUB_3425, 0.07))
        {
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
        }  // (692, 751)
        if (/*head_pan_link vs. wrist_roll_link*/ sphere_sphere_self_collision(
            0.01325, 0.0, ADD_1497, 0.197, SUB_3423, SUB_3424, SUB_3425, 0.07))
        {
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
        }  // (751, 751)
        if (/*shoulder_pan_link vs. wrist_roll_link*/ sphere_sphere_self_collision(
            ADD_1770, SUB_1769, ADD_1771, 0.124, SUB_3423, SUB_3424, SUB_3425, 0.07))
        {
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
        }  // (751, 751)
        if (/*torso_lift_link vs. wrist_roll_link*/ sphere_sphere_self_collision(
            -0.186875, 0.0, ADD_1421, 0.308, SUB_3423, SUB_3424, SUB_3425, 0.07))
        {
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
        }  // (751, 751)
        if (/*torso_lift_link_collision_2 vs. wrist_roll_link*/ sphere_sphere_self_collision(
            0.013125, 0.0, ADD_1490, 0.07, SUB_3423, SUB_3424, SUB_3425, 0.07))
        {
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
        }  // (751, 751)
        if (/*wrist_roll_link vs. torso_fixed_link*/ sphere_sphere_self_collision(
            SUB_3423, SUB_3424, SUB_3425, 0.07, -0.186875, 0.0, 0.587425, 0.277))
        {
            if (sphere_sphere_self_collision(
                    SUB_3450, SUB_3451, SUB_3452, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3450, SUB_3451, SUB_3452, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3450, SUB_3451, SUB_3452, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3450, SUB_3451, SUB_3452, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3450, SUB_3451, SUB_3452, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3450, SUB_3451, SUB_3452, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_861, ADD_862, ADD_863, 0.055, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_861, ADD_862, ADD_863, 0.055, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_861, ADD_862, ADD_863, 0.055, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_861, ADD_862, ADD_863, 0.055, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_861, ADD_862, ADD_863, 0.055, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_861, ADD_862, ADD_863, 0.055, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
        }  // (751, 751)
        if (/*wrist_roll_link*/ sphere_environment_in_collision(
            environment, SUB_3423, SUB_3424, SUB_3425, 0.07))
        {
            if (sphere_environment_in_collision(environment, SUB_3450, SUB_3451, SUB_3452, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_861, ADD_862, ADD_863, 0.055))
            {
                return false;
            }
        }  // (751, 751)
        auto MUL_3489 = ADD_3360 * 2.0;
        auto SUB_3492 = 1.0 - MUL_3489;
        auto MUL_3527 = SUB_3492 * 0.085;
        auto MUL_965 = SUB_888 * 0.16645;
        auto MUL_970 = SUB_888 * MUL_965;
        auto MUL_960 = ADD_882 * 0.16645;
        auto MUL_968 = ADD_882 * MUL_960;
        auto ADD_972 = MUL_968 + MUL_970;
        auto MUL_975 = ADD_972 * 2.0;
        auto SUB_978 = 0.16645 - MUL_975;
        auto ADD_997 = ADD_861 + SUB_978;
        auto SUB_3549 = ADD_997 - MUL_3527;
        auto MUL_3495 = ADD_3367 * 2.0;
        auto MUL_3531 = MUL_3495 * 0.085;
        auto MUL_980 = SUB_891 * MUL_965;
        auto MUL_981 = ADD_875 * MUL_960;
        auto ADD_983 = MUL_980 + MUL_981;
        auto MUL_986 = ADD_983 * 2.0;
        auto ADD_998 = ADD_862 + MUL_986;
        auto SUB_3550 = ADD_998 - MUL_3531;
        auto MUL_3498 = SUB_3370 * 2.0;
        auto MUL_3535 = MUL_3498 * 0.085;
        auto MUL_988 = SUB_891 * MUL_960;
        auto MUL_990 = ADD_875 * MUL_965;
        auto SUB_992 = MUL_990 - MUL_988;
        auto MUL_995 = SUB_992 * 2.0;
        auto ADD_999 = ADD_863 + MUL_995;
        auto SUB_3551 = ADD_999 - MUL_3535;
        auto SUB_3499 = MUL_3357 - MUL_3353;
        auto MUL_3501 = SUB_3499 * 2.0;
        auto MUL_3565 = MUL_3501 * 0.02;
        auto MUL_3554 = SUB_3492 * 0.07;
        auto SUB_3576 = MUL_3565 - MUL_3554;
        auto ADD_3579 = ADD_997 + SUB_3576;
        auto MUL_3558 = MUL_3495 * 0.07;
        auto MUL_3481 = ADD_875 * ADD_875;
        auto ADD_3502 = MUL_3352 + MUL_3481;
        auto MUL_3505 = ADD_3502 * 2.0;
        auto SUB_3508 = 1.0 - MUL_3505;
        auto MUL_3567 = SUB_3508 * 0.02;
        auto SUB_3577 = MUL_3567 - MUL_3558;
        auto ADD_3580 = ADD_998 + SUB_3577;
        auto MUL_3562 = MUL_3498 * 0.07;
        auto MUL_3482 = SUB_891 * ADD_875;
        auto MUL_3485 = ADD_882 * SUB_888;
        auto ADD_3509 = MUL_3485 + MUL_3482;
        auto MUL_3511 = ADD_3509 * 2.0;
        auto MUL_3569 = MUL_3511 * 0.02;
        auto SUB_3578 = MUL_3569 - MUL_3562;
        auto ADD_3581 = ADD_999 + SUB_3578;
        auto ADD_3612 = MUL_3554 + MUL_3565;
        auto SUB_3618 = ADD_997 - ADD_3612;
        auto ADD_3614 = MUL_3558 + MUL_3567;
        auto SUB_3619 = ADD_998 - ADD_3614;
        auto ADD_3616 = MUL_3562 + MUL_3569;
        auto SUB_3620 = ADD_999 - ADD_3616;
        auto MUL_3623 = SUB_3492 * 0.1;
        auto SUB_3645 = MUL_3565 - MUL_3623;
        auto ADD_3648 = ADD_997 + SUB_3645;
        auto MUL_3627 = MUL_3495 * 0.1;
        auto SUB_3646 = MUL_3567 - MUL_3627;
        auto ADD_3649 = ADD_998 + SUB_3646;
        auto MUL_3631 = MUL_3498 * 0.1;
        auto SUB_3647 = MUL_3569 - MUL_3631;
        auto ADD_3650 = ADD_999 + SUB_3647;
        auto ADD_3681 = MUL_3623 + MUL_3565;
        auto SUB_3687 = ADD_997 - ADD_3681;
        auto ADD_3683 = MUL_3627 + MUL_3567;
        auto SUB_3688 = ADD_998 - ADD_3683;
        auto ADD_3685 = MUL_3631 + MUL_3569;
        auto SUB_3689 = ADD_999 - ADD_3685;
        if (/*base_link vs. gripper_link*/ sphere_sphere_self_collision(
            -0.02, 0.0, 0.188, 0.34, SUB_3549, SUB_3550, SUB_3551, 0.075))
        {
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
        }  // (751, 822)
        if (/*gripper_link vs. torso_fixed_link*/ sphere_sphere_self_collision(
            SUB_3549, SUB_3550, SUB_3551, 0.075, -0.186875, 0.0, 0.587425, 0.277))
        {
            if (sphere_sphere_self_collision(
                    ADD_3579, ADD_3580, ADD_3581, 0.05, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3579, ADD_3580, ADD_3581, 0.05, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3579, ADD_3580, ADD_3581, 0.05, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3579, ADD_3580, ADD_3581, 0.05, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3579, ADD_3580, ADD_3581, 0.05, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3579, ADD_3580, ADD_3581, 0.05, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3618, SUB_3619, SUB_3620, 0.05, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3618, SUB_3619, SUB_3620, 0.05, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3618, SUB_3619, SUB_3620, 0.05, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3618, SUB_3619, SUB_3620, 0.05, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3618, SUB_3619, SUB_3620, 0.05, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3618, SUB_3619, SUB_3620, 0.05, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3648, ADD_3649, ADD_3650, 0.05, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3648, ADD_3649, ADD_3650, 0.05, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3648, ADD_3649, ADD_3650, 0.05, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3648, ADD_3649, ADD_3650, 0.05, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3648, ADD_3649, ADD_3650, 0.05, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3648, ADD_3649, ADD_3650, 0.05, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3687, SUB_3688, SUB_3689, 0.05, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3687, SUB_3688, SUB_3689, 0.05, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3687, SUB_3688, SUB_3689, 0.05, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3687, SUB_3688, SUB_3689, 0.05, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3687, SUB_3688, SUB_3689, 0.05, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3687, SUB_3688, SUB_3689, 0.05, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
        }  // (822, 822)
        if (/*gripper_link*/ sphere_environment_in_collision(
            environment, SUB_3549, SUB_3550, SUB_3551, 0.075))
        {
            if (sphere_environment_in_collision(environment, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
        }  // (822, 822)
        if (/*head_pan_link vs. gripper_link*/ sphere_sphere_self_collision(
            0.01325, 0.0, ADD_1497, 0.197, SUB_3549, SUB_3550, SUB_3551, 0.075))
        {
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
        }  // (822, 822)
        if (/*shoulder_lift_link vs. gripper_link*/ sphere_sphere_self_collision(
            ADD_1952, ADD_1953, ADD_1954, 0.134, SUB_3549, SUB_3550, SUB_3551, 0.075))
        {
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
        }  // (822, 822)
        if (/*shoulder_pan_link vs. gripper_link*/ sphere_sphere_self_collision(
            ADD_1770, SUB_1769, ADD_1771, 0.124, SUB_3549, SUB_3550, SUB_3551, 0.075))
        {
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
        }  // (822, 822)
        if (/*torso_lift_link vs. gripper_link*/ sphere_sphere_self_collision(
            -0.186875, 0.0, ADD_1421, 0.308, SUB_3549, SUB_3550, SUB_3551, 0.075))
        {
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
        }  // (822, 822)
        if (/*torso_lift_link_collision_2 vs. gripper_link*/ sphere_sphere_self_collision(
            0.013125, 0.0, ADD_1490, 0.07, SUB_3549, SUB_3550, SUB_3551, 0.075))
        {
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3579, ADD_3580, ADD_3581, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, SUB_3618, SUB_3619, SUB_3620, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3648, ADD_3649, ADD_3650, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, SUB_3687, SUB_3688, SUB_3689, 0.05))
            {
                return false;
            }
        }  // (822, 822)
        auto MUL_3724 = SUB_3499 * 2.0;
        auto MUL_1090 = SUB_888 * 0.065425;
        auto MUL_1100 = SUB_891 * MUL_1090;
        auto MUL_1093 = ADD_875 * 0.065425;
        auto MUL_1102 = ADD_882 * MUL_1093;
        auto SUB_1103 = MUL_1102 - MUL_1100;
        auto MUL_1106 = SUB_1103 * 2.0;
        auto ADD_1129 = ADD_997 + MUL_1106;
        auto MUL_3756 = MUL_3724 * 0.009;
        auto SUB_3772 = ADD_1129 - MUL_3756;
        auto MUL_3728 = ADD_3502 * 2.0;
        auto SUB_3731 = 1.0 - MUL_3728;
        auto MUL_3760 = SUB_3731 * 0.009;
        auto MUL_1112 = SUB_888 * MUL_1090;
        auto MUL_1110 = ADD_875 * MUL_1093;
        auto ADD_1114 = MUL_1110 + MUL_1112;
        auto MUL_1117 = ADD_1114 * 2.0;
        auto SUB_1120 = 0.065425 - MUL_1117;
        auto ADD_1130 = ADD_998 + SUB_1120;
        auto SUB_3773 = ADD_1130 - MUL_3760;
        auto MUL_3734 = ADD_3509 * 2.0;
        auto MUL_3764 = MUL_3734 * 0.009;
        auto MUL_1121 = SUB_891 * MUL_1093;
        auto MUL_1123 = ADD_882 * MUL_1090;
        auto ADD_1125 = MUL_1121 + MUL_1123;
        auto MUL_1127 = ADD_1125 * 2.0;
        auto ADD_1131 = ADD_999 + MUL_1127;
        auto SUB_3774 = ADD_1131 - MUL_3764;
        auto ADD_3735 = MUL_3358 + MUL_3354;
        auto MUL_3737 = ADD_3735 * 2.0;
        auto MUL_3712 = ADD_3360 * 2.0;
        auto SUB_3715 = 1.0 - MUL_3712;
        auto MUL_3776 = SUB_3715 * 0.017;
        auto MUL_3783 = MUL_3724 * 0.0085;
        auto SUB_3805 = MUL_3776 - MUL_3783;
        auto MUL_3795 = MUL_3737 * 0.005;
        auto SUB_3808 = SUB_3805 - MUL_3795;
        auto ADD_3811 = ADD_1129 + SUB_3808;
        auto SUB_3738 = MUL_3485 - MUL_3482;
        auto MUL_3740 = SUB_3738 * 2.0;
        auto MUL_3799 = MUL_3740 * 0.005;
        auto MUL_3787 = SUB_3731 * 0.0085;
        auto MUL_3718 = ADD_3367 * 2.0;
        auto MUL_3778 = MUL_3718 * 0.017;
        auto SUB_3806 = MUL_3778 - MUL_3787;
        auto SUB_3809 = SUB_3806 - MUL_3799;
        auto ADD_3812 = ADD_1130 + SUB_3809;
        auto ADD_3741 = MUL_3351 + MUL_3481;
        auto MUL_3744 = ADD_3741 * 2.0;
        auto SUB_3747 = 1.0 - MUL_3744;
        auto MUL_3803 = SUB_3747 * 0.005;
        auto MUL_3791 = MUL_3734 * 0.0085;
        auto MUL_3721 = SUB_3370 * 2.0;
        auto MUL_3780 = MUL_3721 * 0.017;
        auto SUB_3807 = MUL_3780 - MUL_3791;
        auto SUB_3810 = SUB_3807 - MUL_3803;
        auto ADD_3813 = ADD_1131 + SUB_3810;
        auto ADD_3841 = SUB_3805 + MUL_3795;
        auto ADD_3844 = ADD_1129 + ADD_3841;
        auto ADD_3842 = SUB_3806 + MUL_3799;
        auto ADD_3845 = ADD_1130 + ADD_3842;
        auto ADD_3843 = SUB_3807 + MUL_3803;
        auto ADD_3846 = ADD_1131 + ADD_3843;
        auto ADD_3877 = MUL_3783 + MUL_3795;
        auto SUB_3883 = ADD_1129 - ADD_3877;
        auto ADD_3879 = MUL_3787 + MUL_3799;
        auto SUB_3884 = ADD_1130 - ADD_3879;
        auto ADD_3881 = MUL_3791 + MUL_3803;
        auto SUB_3885 = ADD_1131 - ADD_3881;
        auto SUB_3910 = MUL_3795 - MUL_3783;
        auto ADD_3913 = ADD_1129 + SUB_3910;
        auto SUB_3911 = MUL_3799 - MUL_3787;
        auto ADD_3914 = ADD_1130 + SUB_3911;
        auto SUB_3912 = MUL_3803 - MUL_3791;
        auto ADD_3915 = ADD_1131 + SUB_3912;
        auto ADD_3952 = MUL_3776 + MUL_3783;
        auto ADD_3958 = ADD_3952 + MUL_3795;
        auto SUB_3964 = ADD_1129 - ADD_3958;
        auto ADD_3954 = MUL_3778 + MUL_3787;
        auto ADD_3960 = ADD_3954 + MUL_3799;
        auto SUB_3965 = ADD_1130 - ADD_3960;
        auto ADD_3956 = MUL_3780 + MUL_3791;
        auto ADD_3962 = ADD_3956 + MUL_3803;
        auto SUB_3966 = ADD_1131 - ADD_3962;
        auto SUB_4003 = MUL_3795 - ADD_3952;
        auto ADD_4006 = ADD_1129 + SUB_4003;
        auto SUB_4004 = MUL_3799 - ADD_3954;
        auto ADD_4007 = ADD_1130 + SUB_4004;
        auto SUB_4005 = MUL_3803 - ADD_3956;
        auto ADD_4008 = ADD_1131 + SUB_4005;
        if (/*base_link vs. r_gripper_finger_link*/ sphere_sphere_self_collision(
            -0.02, 0.0, 0.188, 0.34, SUB_3772, SUB_3773, SUB_3774, 0.03))
        {
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
        }  // (822, 912)
        if (/*head_pan_link vs. r_gripper_finger_link*/ sphere_sphere_self_collision(
            0.01325, 0.0, ADD_1497, 0.197, SUB_3772, SUB_3773, SUB_3774, 0.03))
        {
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
        }  // (912, 912)
        if (/*r_gripper_finger_link vs. torso_fixed_link*/ sphere_sphere_self_collision(
            SUB_3772, SUB_3773, SUB_3774, 0.03, -0.186875, 0.0, 0.587425, 0.277))
        {
            if (sphere_sphere_self_collision(
                    ADD_3811, ADD_3812, ADD_3813, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3811, ADD_3812, ADD_3813, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3811, ADD_3812, ADD_3813, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3811, ADD_3812, ADD_3813, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3811, ADD_3812, ADD_3813, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3811, ADD_3812, ADD_3813, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3844, ADD_3845, ADD_3846, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3844, ADD_3845, ADD_3846, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3844, ADD_3845, ADD_3846, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3844, ADD_3845, ADD_3846, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3844, ADD_3845, ADD_3846, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3844, ADD_3845, ADD_3846, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3883, SUB_3884, SUB_3885, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3883, SUB_3884, SUB_3885, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3883, SUB_3884, SUB_3885, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3883, SUB_3884, SUB_3885, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3883, SUB_3884, SUB_3885, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3883, SUB_3884, SUB_3885, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3913, ADD_3914, ADD_3915, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3913, ADD_3914, ADD_3915, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3913, ADD_3914, ADD_3915, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3913, ADD_3914, ADD_3915, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3913, ADD_3914, ADD_3915, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_3913, ADD_3914, ADD_3915, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3964, SUB_3965, SUB_3966, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3964, SUB_3965, SUB_3966, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3964, SUB_3965, SUB_3966, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3964, SUB_3965, SUB_3966, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3964, SUB_3965, SUB_3966, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_3964, SUB_3965, SUB_3966, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4006, ADD_4007, ADD_4008, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4006, ADD_4007, ADD_4008, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4006, ADD_4007, ADD_4008, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4006, ADD_4007, ADD_4008, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4006, ADD_4007, ADD_4008, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4006, ADD_4007, ADD_4008, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
        }  // (912, 912)
        if (/*r_gripper_finger_link*/ sphere_environment_in_collision(
            environment, SUB_3772, SUB_3773, SUB_3774, 0.03))
        {
            if (sphere_environment_in_collision(environment, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
        }  // (912, 912)
        if (/*shoulder_lift_link vs. r_gripper_finger_link*/ sphere_sphere_self_collision(
            ADD_1952, ADD_1953, ADD_1954, 0.134, SUB_3772, SUB_3773, SUB_3774, 0.03))
        {
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
        }  // (912, 912)
        if (/*shoulder_pan_link vs. r_gripper_finger_link*/ sphere_sphere_self_collision(
            ADD_1770, SUB_1769, ADD_1771, 0.124, SUB_3772, SUB_3773, SUB_3774, 0.03))
        {
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
        }  // (912, 912)
        if (/*torso_lift_link vs. r_gripper_finger_link*/ sphere_sphere_self_collision(
            -0.186875, 0.0, ADD_1421, 0.308, SUB_3772, SUB_3773, SUB_3774, 0.03))
        {
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
        }  // (912, 912)
        if (/*torso_lift_link_collision_2 vs. r_gripper_finger_link*/ sphere_sphere_self_collision(0.013125, 0.0, ADD_1490, 0.07, SUB_3772, SUB_3773, SUB_3774, 0.03))
        {
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3811, ADD_3812, ADD_3813, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3844, ADD_3845, ADD_3846, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, SUB_3883, SUB_3884, SUB_3885, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_3913, ADD_3914, ADD_3915, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, SUB_3964, SUB_3965, SUB_3966, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_4006, ADD_4007, ADD_4008, 0.012))
            {
                return false;
            }
        }  // (912, 912)
        auto SUB_1238 = MUL_1100 - MUL_1102;
        auto MUL_4047 = SUB_3499 * 2.0;
        auto MUL_4078 = MUL_4047 * 0.009;
        auto MUL_1241 = SUB_1238 * 2.0;
        auto ADD_1264 = ADD_997 + MUL_1241;
        auto ADD_4089 = ADD_1264 + MUL_4078;
        auto ADD_1248 = MUL_1110 + MUL_1112;
        auto MUL_4051 = ADD_3502 * 2.0;
        auto SUB_4054 = 1.0 - MUL_4051;
        auto MUL_4080 = SUB_4054 * 0.009;
        auto MUL_1250 = ADD_1248 * 2.0;
        auto SUB_1253 = MUL_1250 - 0.065425;
        auto ADD_1265 = ADD_998 + SUB_1253;
        auto ADD_4090 = ADD_1265 + MUL_4080;
        auto ADD_1258 = MUL_1121 + MUL_1123;
        auto MUL_4057 = ADD_3509 * 2.0;
        auto MUL_4082 = MUL_4057 * 0.009;
        auto MUL_1261 = ADD_1258 * 2.0;
        auto SUB_1266 = ADD_999 - MUL_1261;
        auto ADD_4091 = SUB_1266 + MUL_4082;
        auto MUL_4060 = ADD_3735 * 2.0;
        auto MUL_4106 = MUL_4060 * 0.005;
        auto MUL_4099 = MUL_4047 * 0.0085;
        auto MUL_4035 = ADD_3360 * 2.0;
        auto SUB_4038 = 1.0 - MUL_4035;
        auto MUL_4093 = SUB_4038 * 0.017;
        auto ADD_4116 = MUL_4093 + MUL_4099;
        auto SUB_4119 = ADD_4116 - MUL_4106;
        auto ADD_4122 = ADD_1264 + SUB_4119;
        auto MUL_4063 = SUB_3738 * 2.0;
        auto MUL_4110 = MUL_4063 * 0.005;
        auto MUL_4101 = SUB_4054 * 0.0085;
        auto MUL_4041 = ADD_3367 * 2.0;
        auto MUL_4095 = MUL_4041 * 0.017;
        auto ADD_4117 = MUL_4095 + MUL_4101;
        auto SUB_4120 = ADD_4117 - MUL_4110;
        auto ADD_4123 = ADD_1265 + SUB_4120;
        auto MUL_4067 = ADD_3741 * 2.0;
        auto SUB_4070 = 1.0 - MUL_4067;
        auto MUL_4114 = SUB_4070 * 0.005;
        auto MUL_4103 = MUL_4057 * 0.0085;
        auto MUL_4044 = SUB_3370 * 2.0;
        auto MUL_4097 = MUL_4044 * 0.017;
        auto ADD_4118 = MUL_4097 + MUL_4103;
        auto SUB_4121 = ADD_4118 - MUL_4114;
        auto ADD_4124 = SUB_1266 + SUB_4121;
        auto ADD_4146 = ADD_4116 + MUL_4106;
        auto ADD_4149 = ADD_1264 + ADD_4146;
        auto ADD_4147 = ADD_4117 + MUL_4110;
        auto ADD_4150 = ADD_1265 + ADD_4147;
        auto ADD_4148 = ADD_4118 + MUL_4114;
        auto ADD_4151 = SUB_1266 + ADD_4148;
        auto SUB_4176 = MUL_4099 - MUL_4106;
        auto ADD_4179 = ADD_1264 + SUB_4176;
        auto SUB_4177 = MUL_4101 - MUL_4110;
        auto ADD_4180 = ADD_1265 + SUB_4177;
        auto SUB_4178 = MUL_4103 - MUL_4114;
        auto ADD_4181 = SUB_1266 + SUB_4178;
        auto ADD_4200 = MUL_4099 + MUL_4106;
        auto ADD_4203 = ADD_1264 + ADD_4200;
        auto ADD_4201 = MUL_4101 + MUL_4110;
        auto ADD_4204 = ADD_1265 + ADD_4201;
        auto ADD_4202 = MUL_4103 + MUL_4114;
        auto ADD_4205 = SUB_1266 + ADD_4202;
        auto SUB_4236 = MUL_4099 - MUL_4093;
        auto SUB_4239 = SUB_4236 - MUL_4106;
        auto ADD_4242 = ADD_1264 + SUB_4239;
        auto SUB_4237 = MUL_4101 - MUL_4095;
        auto SUB_4240 = SUB_4237 - MUL_4110;
        auto ADD_4243 = ADD_1265 + SUB_4240;
        auto SUB_4238 = MUL_4103 - MUL_4097;
        auto SUB_4241 = SUB_4238 - MUL_4114;
        auto ADD_4244 = SUB_1266 + SUB_4241;
        auto ADD_4272 = SUB_4236 + MUL_4106;
        auto ADD_4275 = ADD_1264 + ADD_4272;
        auto ADD_4273 = SUB_4237 + MUL_4110;
        auto ADD_4276 = ADD_1265 + ADD_4273;
        auto ADD_4274 = SUB_4238 + MUL_4114;
        auto ADD_4277 = SUB_1266 + ADD_4274;
        if (/*base_link vs. l_gripper_finger_link*/ sphere_sphere_self_collision(
            -0.02, 0.0, 0.188, 0.34, ADD_4089, ADD_4090, ADD_4091, 0.03))
        {
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.12, 0.0, 0.182, 0.24, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.225, 0.0, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, -0.06, 0.16, 0.22, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, -0.07, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, -0.135, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, -0.185, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, -0.2, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, -0.2, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.08, 0.06, 0.16, 0.22, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.215, 0.07, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.185, 0.135, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13, 0.185, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.065, 0.2, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.01, 0.2, 0.31, 0.066, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
        }  // (912, 991)
        if (/*head_pan_link vs. l_gripper_finger_link*/ sphere_sphere_self_collision(
            0.01325, 0.0, ADD_1497, 0.197, ADD_4089, ADD_4090, ADD_4091, 0.03))
        {
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.03375, 0.0, ADD_1501, 0.15, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0, ADD_1506, 0.05, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.0425, ADD_1506, 0.05, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.0425, ADD_1506, 0.05, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, 0.085, ADD_1506, 0.05, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.11125, -0.085, ADD_1506, 0.05, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1534, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1534, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1534, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1534, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, -0.115, ADD_1558, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, -0.115, ADD_1558, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, -0.115, ADD_1558, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, -0.115, ADD_1558, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1582, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, -0.115, ADD_1588, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, -0.115, ADD_1594, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1534, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1534, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1534, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1534, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.02875, 0.115, ADD_1558, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.05425, 0.115, ADD_1558, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.07975, 0.115, ADD_1558, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.10525, 0.115, ADD_1558, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1582, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.13425, 0.115, ADD_1588, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.12625, 0.115, ADD_1594, 0.03, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
        }  // (991, 991)
        if (/*l_gripper_finger_link vs. torso_fixed_link*/ sphere_sphere_self_collision(
            ADD_4089, ADD_4090, ADD_4091, 0.03, -0.186875, 0.0, 0.587425, 0.277))
        {
            if (sphere_sphere_self_collision(
                    ADD_4122, ADD_4123, ADD_4124, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4122, ADD_4123, ADD_4124, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4122, ADD_4123, ADD_4124, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4122, ADD_4123, ADD_4124, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4122, ADD_4123, ADD_4124, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4122, ADD_4123, ADD_4124, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4149, ADD_4150, ADD_4151, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4149, ADD_4150, ADD_4151, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4149, ADD_4150, ADD_4151, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4149, ADD_4150, ADD_4151, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4149, ADD_4150, ADD_4151, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4149, ADD_4150, ADD_4151, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4179, ADD_4180, ADD_4181, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4179, ADD_4180, ADD_4181, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4179, ADD_4180, ADD_4181, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4179, ADD_4180, ADD_4181, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4179, ADD_4180, ADD_4181, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4179, ADD_4180, ADD_4181, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4203, ADD_4204, ADD_4205, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4203, ADD_4204, ADD_4205, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4203, ADD_4204, ADD_4205, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4203, ADD_4204, ADD_4205, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4203, ADD_4204, ADD_4205, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4203, ADD_4204, ADD_4205, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4242, ADD_4243, ADD_4244, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4242, ADD_4243, ADD_4244, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4242, ADD_4243, ADD_4244, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4242, ADD_4243, ADD_4244, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4242, ADD_4243, ADD_4244, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4242, ADD_4243, ADD_4244, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4275, ADD_4276, ADD_4277, 0.012, -0.186875, -0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4275, ADD_4276, ADD_4277, 0.012, -0.186875, 0.07, 0.727425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4275, ADD_4276, ADD_4277, 0.012, -0.186875, -0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4275, ADD_4276, ADD_4277, 0.012, -0.186875, 0.07, 0.577425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4275, ADD_4276, ADD_4277, 0.012, -0.186875, 0.07, 0.447425, 0.12))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_4275, ADD_4276, ADD_4277, 0.012, -0.186875, -0.07, 0.447425, 0.12))
            {
                return false;
            }
        }  // (991, 991)
        if (/*l_gripper_finger_link*/ sphere_environment_in_collision(
            environment, ADD_4089, ADD_4090, ADD_4091, 0.03))
        {
            if (sphere_environment_in_collision(environment, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
        }  // (991, 991)
        if (/*shoulder_lift_link vs. l_gripper_finger_link*/ sphere_sphere_self_collision(
            ADD_1952, ADD_1953, ADD_1954, 0.134, ADD_4089, ADD_4090, ADD_4091, 0.03))
        {
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1981, ADD_1982, ADD_1983, 0.04, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    SUB_2022, ADD_2023, ADD_2024, 0.04, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2057, ADD_2058, ADD_2059, 0.04, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2091, ADD_2092, ADD_2093, 0.04, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2113, ADD_2114, SUB_2115, 0.055, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2135, ADD_2136, SUB_2137, 0.055, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_2157, ADD_2158, SUB_2159, 0.055, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
        }  // (991, 991)
        if (/*shoulder_pan_link vs. l_gripper_finger_link*/ sphere_sphere_self_collision(
            ADD_1770, SUB_1769, ADD_1771, 0.124, ADD_4089, ADD_4090, ADD_4091, 0.03))
        {
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.03265, 0.0, ADD_82, 0.055, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1812, SUB_1811, ADD_1813, 0.055, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1837, SUB_1836, ADD_1838, 0.055, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    ADD_1862, SUB_1861, ADD_1838, 0.055, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
        }  // (991, 991)
        if (/*torso_lift_link vs. l_gripper_finger_link*/ sphere_sphere_self_collision(
            -0.186875, 0.0, ADD_1421, 0.308, ADD_4089, ADD_4090, ADD_4091, 0.03))
        {
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1429, 0.15, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1429, 0.15, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1421, 0.15, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, 0.05, ADD_1450, 0.15, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1450, 0.15, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    -0.186875, -0.05, ADD_1421, 0.15, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
        }  // (991, 991)
        if (/*torso_lift_link_collision_2 vs. l_gripper_finger_link*/ sphere_sphere_self_collision(0.013125, 0.0, ADD_1490, 0.07, ADD_4089, ADD_4090, ADD_4091, 0.03))
        {
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_4122, ADD_4123, ADD_4124, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_4149, ADD_4150, ADD_4151, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_4179, ADD_4180, ADD_4181, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_4203, ADD_4204, ADD_4205, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_4242, ADD_4243, ADD_4244, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision(
                    0.013125, 0.0, ADD_1490, 0.07, ADD_4275, ADD_4276, ADD_4277, 0.012))
            {
                return false;
            }
        }  // (991, 991)

        return true;
    }
}
