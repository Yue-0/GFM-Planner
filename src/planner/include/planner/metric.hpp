/* @author: YueLin */

#pragma once

#include <cmath>
#include <cstring>

#include <Eigen/Eigen>

#include "utils/se2.hpp"
#include "utils/popcount.h"
#include "utils/interpolation.h"

namespace gfm_planner
{
    class GeometricFeatureMetric
    {
        private:
            int h, w;
            double epsilon;
            uint64 *metric;
            uint64 pow2[65];
            double sigma[65];
        
        public:
            float resolution = 0;
        
        public:
            ~GeometricFeatureMetric() {delete[] metric;}

            GeometricFeatureMetric(int height, int width, double e)
            {
                epsilon = e;
                h = height; w = width;
                metric = new uint64[w * h];
                for(int p = pow2[0] = 1; p <= 64; p++)
                    pow2[p] = pow2[p - 1] << 1;
                for(int p = 0; p <= 64; p++)
                    sigma[p] = sigmoid(p);
            }
        
        private:
            int index(int x, int y) {return w * y + x;}
            
            double sigmoid(double x)
            {
                return 1. / (1 + std::exp(epsilon * (1 - 0.03125 * x)));
            }

            int q(int x, int y, int n, int range)
            {
                int i = (n - range) & 0x3F;
                int j = (n + range) & 0x3F;
                return popcount(metric[index(x, y)] & (
                    i <= j?
                    pow2[j + 1] - pow2[i]:
                    ~(pow2[i] - pow2[j + 1])
                ));
            }

        public:
            double max() const {return sigma[64];}

            double evaluate(int x, int y)
            {
                return sigma[popcount(metric[index(x, y)])];
            }

            double evaluate(int x, int y, int n, int range)
            {
                return sigma[q(x, y, n, range)];
            }
            
            double evaluate(const Eigen::Vector3d& se2, 
                            Eigen::Vector3d& grad, 
                            double range)
            {
                int x[2], y[2], z[2], t[2];
                double u[2], v[2], s[2], r[2];
                interpolation4(
                    32. * range, t, t + 1, r, r + 1,
                    32. * clip(se2[2]) / PI, z, z + 1, s, s + 1,
                    clip(se2[0] * resolution, w), x, x + 1, u, u + 1,
                    clip(se2[1] * resolution, h), y, y + 1, v, v + 1
                );

                z[1] &= 0x3F;
                grad.setZero();
                double value, gfm = 0;
                for(int i, j, k, l, n = 0; n <= 0xF; n++)
                {
                    i = n >> 3; j = (n >> 2) & 1; k = (n >> 1) & 1; l = n & 1;
                    value = r[l] * q(x[!i], y[!j], z[!k], t[!l]);
                    grad[0] += v[j] * s[k] * value * (i? -1: 1);
                    grad[1] += u[i] * s[k] * value * (j? -1: 1);
                    grad[2] += u[i] * v[j] * value * (k? -1: 1);
                    gfm += u[i] * v[j] * s[k] * value;
                }
                
                value = epsilon * (gfm * 0.03125 - 1.);
                grad *= 0.03125 * epsilon / (1. + std::exp(value) + (
                    gfm = 1. + std::exp(-value)
                ));
                return 1. / gfm;
            }

            void update(float scale, const uint64* arr)
            {
                resolution = 1 / scale;
                std::memcpy(metric, arr, h * w * sizeof(uint64));
            }
    };
}
