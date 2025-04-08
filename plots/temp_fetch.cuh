template <>
__device__ __forceinline__ bool fkcc<ppln::robots::Fetch>(volatile float *q, ppln::collision::Environment<float> *environment, int tid)
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
    } // (0, 0)
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
    } // (0, 0)
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
    } // (0, 5)
    auto ADD_1490 = ADD_54 + 0.24;
    if (sphere_environment_in_collision(environment, 0.013125, 0.0, ADD_1490, 0.07))
    {
        return false;
    } // (5, 6)
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
    } // (6, 15)
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
    } // (15, 52)
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
    } // (52, 166)
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
    } // (166, 166)
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
    } // (166, 308)
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
    } // (308, 308)
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
    } // (308, 308)
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
    } // (308, 308)
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
    } // (308, 308)
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
    } // (308, 308)
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
    } // (308, 435)
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
    } // (435, 435)
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
    } // (435, 435)
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
    } // (435, 435)
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
    } // (435, 435)
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
    } // (435, 435)
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
    } // (435, 571)
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
    } // (571, 571)
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
    } // (571, 571)
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
    } // (571, 571)
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
    } // (571, 571)
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
    } // (571, 571)
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
    } // (571, 692)
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
    } // (692, 692)
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
    } // (692, 692)
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
    } // (692, 692)
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
    } // (692, 692)
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
    } // (692, 692)
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
    } // (692, 751)
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
    } // (751, 751)
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
    } // (751, 751)
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
    } // (751, 751)
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
    } // (751, 751)
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
    } // (751, 751)
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
    } // (751, 751)
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
    } // (751, 822)
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
    } // (822, 822)
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
    } // (822, 822)
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
    } // (822, 822)
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
    } // (822, 822)
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
    } // (822, 822)
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
    } // (822, 822)
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
    } // (822, 822)
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
    } // (822, 912)
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
    } // (912, 912)
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
    } // (912, 912)
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
    } // (912, 912)
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
    } // (912, 912)
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
    } // (912, 912)
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
    } // (912, 912)
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
    } // (912, 912)
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
    } // (912, 991)
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
    } // (991, 991)
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
    } // (991, 991)
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
    } // (991, 991)
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
    } // (991, 991)
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
    } // (991, 991)
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
    } // (991, 991)
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
    } // (991, 991)

    return true;
}