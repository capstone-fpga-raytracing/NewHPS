#include <stdint.h>
#include <math.h>

typedef unsigned uint;
typedef unsigned char byte;

#define FIP_MIN (-2147483647 - 1) // -32768.0 (min)
#define FIP_MAX 2147483647 // 32767.99998 (max)
#define FIP_ONE 0x00010000 // 1


typedef struct Ray
{
    int origin[3];
    int dir[3];
} Ray;

inline int fip_div(int x, int y)
{
    int64_t temp_dividend = (int64_t)x << 16;
    int64_t temp_res = temp_dividend / y;
    return (int)temp_res;
}

inline int fip_mult(int x, int y)
{
    int64_t temp_res = (int64_t)x * (int64_t)y;
    return (int)(temp_res >> 16);
}

uint fip_sqrt(uint x)
{
    float val = sqrtf((float)x / 65536);
    return (uint)(val * (1 << 16));
}

// saturate (i.e. limit to fip bounds).
inline int fip_sat(int64_t val)
{
    if (val > FIP_MAX) {
        return FIP_MAX;
    }
    else if (val < FIP_MIN) {
        return FIP_MIN;
    }
    return (int)val;
}

inline int fip_sat_div(int x, int y)
{
    int64_t temp = (int64_t)x << 16;
    int64_t res = temp / y;
    return fip_sat(res);
}

inline int fip_sat_mult(int x, int y)
{
    int64_t res = (int64_t)x * (int64_t)y;
    return fip_sat(res >> 16);
}

inline int fip_sat_add(int x, int y)
{
    int64_t res = (int64_t)x + y;
    return fip_sat(res);
}

int fip_det(const int v0[3], const int v1[3], const int v2[3]) {
    /*
    |a b c| v0
    |d e f| v1
    |g h i| v2
    det = a(ei-fh) + b(fg-di) + c(dh-eg)
    */
    int sub1, sub2, sub3;
    sub1 = fip_mult(v1[1], v2[2]) - fip_mult(v1[2], v2[1]);
    sub2 = fip_mult(v1[2], v2[0]) - fip_mult(v1[0], v2[2]);
    sub3 = fip_mult(v1[0], v2[1]) - fip_mult(v1[1], v2[0]);
    return fip_mult(v0[0], sub1) + fip_mult(v0[1], sub2) + fip_mult(v0[2], sub3);

}

bool new_ray_intersect_tri(const int* vs, const Ray* ray, int* pt)
{
    // solve the system by Cramer's rule:
    // [T1, T2, -D] |a|
    //              |b| = E - P0, where
    //              |t|
    // Triangle = aT1 + bT2 + P0 for a >= 0, b >=0, a + b <= 1.

    const int v[3][3] = {
        { vs[0], vs[1], vs[2] },
        { vs[3], vs[4], vs[5] },
        { vs[6], vs[7], vs[8] },
    };

    const int sys[4][3] = {
        { v[1][0] - v[0][0], v[1][1] - v[0][1], v[1][2] - v[0][2] }, // vert[1] - vert[0]
        { v[2][0] - v[0][0], v[2][1] - v[0][1], v[2][2] - v[0][2] }, // vert[2] - vert[0]
        { -ray->dir[0], -ray->dir[1], -ray->dir[2] }, // -ray.dir
        { ray->origin[0] - v[0][0], ray->origin[1] - v[0][1], ray->origin[2] - v[0][2] } // ray.origin - vert[0]
    };

    int det_coeffs = fip_det(sys[0], sys[1], sys[2]);
    if (det_coeffs == 0) // no unique soln (very unlikely)
        return false;

    int a = fip_sat_div(fip_det(sys[3], sys[1], sys[2]), det_coeffs);
    int b = fip_sat_div(fip_det(sys[0], sys[3], sys[2]), det_coeffs);
    int t = fip_sat_div(fip_det(sys[0], sys[1], sys[3]), det_coeffs);

    if (a >= 0 && b >= 0 && fip_sat_add(a, b) <= FIP_ONE && t >= 0)
    {
        *pt = t;
        return true;
    }
    else return false;
}

int main()
{
    


    return 0;
}