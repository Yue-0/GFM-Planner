/* @author: YueLin */

#pragma once

#include <cmath>

const double PI = std::acos(-1);

inline bool zero(double n)
{
    return std::fabs(n) < 1e-9;
}

inline double clip(double rad)
{
    if(rad < 0 || rad >= 2 * PI)
        rad -= 2 * PI * std::floor(rad / (2 * PI));
    return rad;
}

class SE2
{
    public:
        double x, y, yaw;
    
    public:
        SE2() = default;

        SE2(double x0, double y0, double theta)
        {
            set(x0, y0, theta);
        }

        SE2 operator+()
        {
            return SE2(x, y, yaw);
        }

        SE2 operator-()
        {
            return SE2(-x, -y, -yaw);
        }

        SE2 operator+(const SE2& other)
        {
            return SE2(x + other.x, y + other.y, yaw + other.yaw);
        }

        SE2 operator-(const SE2& other)
        {
            return SE2(x - other.x, y - other.y, yaw - other.yaw);
        }

        SE2 operator*(const double n)
        {
            return SE2(n * x, n * y, n * yaw);
        }

        SE2 operator/(const double n)
        {
            return *this * (1. / n);
        }

        friend SE2 operator*(const double n, SE2 se2)
        {
            return se2 * n;
        }

        void set(double x0, double y0, double theta)
        {
            x = x0; y = y0; yaw = clip(theta);
        }

        void integer(double ratio, int* x0, int* y0)
        {
            *x0 = std::round(x * ratio);
            *y0 = std::round(y * ratio);
        }

        void integer(double ratio, int* x0, int* y0, int* n)
        {
            integer(ratio, x0, y0);
            *n = std::round(32 * yaw / PI);
        }
};
