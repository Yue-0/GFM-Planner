/* @Author YueLin */

#pragma once

#include <cstring>
#include <algorithm>

#include <Eigen/Eigen>

#include "utils/interpolation.h"

class ESDF
{
    private:
        int h, w;
        float* data;
    
    public:
        float scale = 0;
    
    public:
        ~ESDF() {delete[] data;}
        
        ESDF(int height, int width): h(height), w(width)
        {
            data = new float[height * width];
        }
    
    private:
        int index(int x, int y) {return w * y + x;}
    
    public:

        /* Get the ESDF value */
        double get(const Eigen::Vector3d& se2)
        {
            /* Linear interpolation */
            double u1, u2, v1, v2;
            int x1, y1, x2, y2;
            interpolation2(
                clip(se2[0] * scale, w), &x1, &x2, &u1, &u2, 
                clip(se2[1] * scale, h), &y1, &y2, &v1, &v2
            );
            
            /* Return the value of ESDF */
            return u1 * v1 * data[index(x2, y2)]
                 + u1 * v2 * data[index(x2, y1)]
                 + u2 * v1 * data[index(x1, y2)]
                 + u2 * v2 * data[index(x1, y1)];
        }

        /* Get the ESDF value and gradient */
        double get(const Eigen::Vector3d& se2, Eigen::Vector3d& grad)
        {
            /* Linear interpolation */
            double u1, u2, v1, v2;
            int x1, y1, x2, y2;
            interpolation2(
                clip(se2[0] * scale, w), &x1, &x2, &u1, &u2, 
                clip(se2[1] * scale, h), &y1, &y2, &v1, &v2
            );
            int x1y1 = index(x1, y1), x2y2 = index(x2, y2),
                x1y2 = index(x1, y2), x2y1 = index(x2, y1);

            /* Get the gradient of ESDF */
            grad[0] = v1 * data[x2y2] + v2 * data[x2y1]
                    - v1 * data[x1y2] - v2 * data[x1y1];
            grad[1] = u1 * data[x2y2] + u2 * data[x1y2]
                    - u1 * data[x2y1] - u2 * data[x1y1];
            grad[2] = 0;
            
            /* Return the value of ESDF */
            return u1 * v1 * data[x2y2]
                 + u1 * v2 * data[x2y1]
                 + u2 * v1 * data[x1y2]
                 + u2 * v2 * data[x1y1];
        }

        /* Update ESDF values */
        void update(float resolution, const float* array)
        {
            scale = 1. / resolution;
            std::memcpy(data, array, h * w * sizeof(float));
        }
};