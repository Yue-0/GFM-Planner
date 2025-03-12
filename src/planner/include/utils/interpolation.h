/* @author: YueLin */

#ifndef __INTER
#define __INTER

#include <math.h>

inline double clip(double n, double m)
{
    m -= 2;
    return n > 0? (n > m? m: n): 0;
}

inline void interpolation1(double x, int* x1, int* x2, double* u, double* v)
{
    *x1 = floor(x); *x2 = ceil(x); *u = x - *x1; *v = 1 - *u;
}

inline void interpolation2(double x, int* x1, int* x2, double* u, double* u_,
                           double y, int* y1, int* y2, double* v, double* v_)
{
    interpolation1(x, x1, x2, u, u_);
    interpolation1(y, y1, y2, v, v_);
}

inline void interpolation3(double x, int* x1, int* x2, double* u, double* u_,
                           double y, int* y1, int* y2, double* v, double* v_,
                           double z, int* z1, int* z2, double* w, double* w_)
{
    interpolation1(z, z1, z2, w, w_);
    interpolation2(x, x1, x2, u, u_, y, y1, y2, v, v_);
}

inline void interpolation4(double x, int* x1, int* x2, double* u, double* u_,
                           double y, int* y1, int* y2, double* v, double* v_,
                           double z, int* z1, int* z2, double* w, double* w_,
                           double t, int* t1, int* t2, double* s, double* s_)
{
    interpolation2(x, x1, x2, u, u_, y, y1, y2, v, v_);
    interpolation2(z, z1, z2, w, w_, t, t1, t2, s, s_);
}

#endif