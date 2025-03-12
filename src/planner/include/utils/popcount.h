/* @author: YueLin */

#ifndef popcount
#define popcount count1

#include <bits/types.h>

typedef __uint64_t uint64;

const uint64 H55 = 0x5555555555555555;
const uint64 H33 = 0x3333333333333333;
const uint64 H0F = 0x0F0F0F0F0F0F0F0F;
const uint64 H01 = 0x0101010101010101;

inline int popcount(uint64 n)
{
    n -= (n >> 1) & H55;
    n = (n & H33) + ((n >> 2) & H33);
    n = (n + (n >> 4)) & H0F;
    return (n * H01) >> 56;
}

// int popcount(uint64 n)
// {
//     int count = 0;
//     while(n)
//     {
//         ++count;
//         n &= n - 1;
//     }
//     return count;
// }

inline int count0(uint64 n)
{
    return 64 - popcount(n);
}

#endif