#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "arm_memmap.h"

typedef unsigned uint;
typedef unsigned char byte;

extern void* SDRAM;
extern void* LWBRIDGE;

#define FIP_MIN (-2147483647 - 1) // -32768.0 (min)
#define FIP_MAX 2147483647 // 32767.99998 (max)
#define FIP_ONE 0x00010000 // 1

#define RTINTR_SYSFS "/sys/bus/platform/drivers/fpga_rtintr/fpga_rtintr"

/* Function Macros */
#define ABS(x) (((x) > 0) ? (x) : -(x))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))


inline void VOLATILE_MEMCPY32(volatile int* dest, const int* src, uint count)
{
    for (uint i = 0; i < count; ++i)
        dest[i] = src[i];
}

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

void fip_cross(const int v0[3], const int v1[3], int output[3]) {
    /*
    |a b c| v0
    |d e f| v1
    o_product = |bf-ce cd-af ae-bd|
    */
    output[0] = fip_mult(v0[1], v1[2]) - fip_mult(v0[2], v1[1]);
    output[1] = fip_mult(v0[2], v1[0]) - fip_mult(v0[0], v1[2]);
    output[2] = fip_mult(v0[0], v1[1]) - fip_mult(v0[1], v1[0]);
}

// this previously used fip_invsqrt(), but that had overflow issues.
// This version fixes the blue/white triangles in the render.
void fip_normalize(int vec[3])
{
    if (vec[0] == 0 && vec[1] == 0 && vec[2] == 0)
        return;

    uint norm = fip_sqrt(
        fip_mult(vec[0], vec[0]) +
        fip_mult(vec[1], vec[1]) +
        fip_mult(vec[2], vec[2]));

    if (norm != 0) {
        vec[0] = fip_div(vec[0], norm);
        vec[1] = fip_div(vec[1], norm);
        vec[2] = fip_div(vec[2], norm);
    }
    else {
        // unlikely. retry. may recurse!
        vec[0] <<= 2;
        vec[1] <<= 2;
        vec[2] <<= 2;
        fip_normalize(vec);
    }
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

typedef struct Camera
{
    int eye[3];
    // precalculate
    int base_dir[3];
    int incr_diru[3];
    int incr_dirv[3];
}
Camera;

typedef struct Ray
{
    int origin[3];
    int dir[3];
} Ray;

void init_camera(const int* p, Camera* cam, int resX, int resY)
{
    int u[3], v[3], w[3];
    memcpy(cam->eye, p, sizeof(int) * 3); p += 3;
    memcpy(u, p, sizeof(int) * 3);   p += 3;
    memcpy(v, p, sizeof(int) * 3);   p += 3;
    memcpy(w, p, sizeof(int) * 3);   p += 3;

    int focal_len = p[0];
    int width = p[1], height = p[2];

    int world_du = fip_div(width, resX << 16);
    int world_dv = fip_div(height, resY << 16);
    int aspratio = fip_div(resX << 16, resY << 16);

    int base_u = fip_mult(aspratio >> 1, world_du - width);
    int incr_u = fip_mult(aspratio, world_du);

    int base_v = (height - world_dv) >> 1;
    int incr_v = -world_dv;

    cam->base_dir[0] = fip_mult(base_u, u[0]) + fip_mult(base_v, v[0]) - fip_mult(focal_len, w[0]);
    cam->base_dir[1] = fip_mult(base_u, u[1]) + fip_mult(base_v, v[1]) - fip_mult(focal_len, w[1]);
    cam->base_dir[2] = fip_mult(base_u, u[2]) + fip_mult(base_v, v[2]) - fip_mult(focal_len, w[2]);

    cam->incr_diru[0] = fip_mult(incr_u, u[0]);
    cam->incr_diru[1] = fip_mult(incr_u, u[1]);
    cam->incr_diru[2] = fip_mult(incr_u, u[2]);

    cam->incr_dirv[0] = fip_mult(incr_v, v[0]);
    cam->incr_dirv[1] = fip_mult(incr_v, v[1]);
    cam->incr_dirv[2] = fip_mult(incr_v, v[2]);
}

bool ray_intersect_box(const Ray* ray, const int* pbbox)
{
    int t_entry = FIP_MIN, t_exit = FIP_MAX;

    int i;
    for (i = 0; i < 3; i++)
    {
        if (ray->dir[i] == 0)
            continue; // no intersection in this component

        int t1 = fip_div(pbbox[i] - ray->origin[i], ray->dir[i]);
        int t2 = fip_div(pbbox[i + 3] - ray->origin[i], ray->dir[i]);

        if (ray->dir[i] > 0) {
            t_entry = MAX(t_entry, t1);
            t_exit = MIN(t_exit, t2);
        }
        else {
            t_entry = MAX(t_entry, t2);
            t_exit = MIN(t_exit, t1);
        }
    }
    return t_exit >= t_entry && t_exit >= 0;
}

#define FIP_AMB 0x00002000 // 0.125
#define MATS_ELEM_SIZE 13
#define LIGHTS_ELEM_SIZE 6
#define FIP_ALMOST_ONE 0x0000ffff // largest below 1

void new_blinn_phong_shading(
    int* const o_total_light, // size=3
    const struct Ray* i_ray,
    const int hit_tri_id,
    const int t, // hit distance
    int num_lights,
    const int* lights,
    const int* mats,
    const int* verts
) {
    int normal[3];
    const int* vs = &verts[9 * hit_tri_id];
    const int v[3][3] = {
        { vs[0], vs[1], vs[2] },
        { vs[3], vs[4], vs[5] },
        { vs[6], vs[7], vs[8] },
    };
    const int edges[2][3] = {
        { v[1][0] - v[0][0], v[1][1] - v[0][1], v[1][2] - v[0][2] }, // vert[1] - vert[0]
        { v[2][0] - v[0][0], v[2][1] - v[0][1], v[2][2] - v[0][2] }, // vert[2] - vert[0]
    };
    fip_cross(edges[0], edges[1], normal);
    fip_normalize(normal);

    // material
    int mat_idx = hit_tri_id * MATS_ELEM_SIZE;
    const int ka[3] = { mats[mat_idx], mats[mat_idx + 1], mats[mat_idx + 2] };
    const int kd[3] = { mats[mat_idx + 3], mats[mat_idx + 4], mats[mat_idx + 5] };
    //int ks[3] = {i_ptrs->Mats[mat_idx[6], i_ptrs->Mats[mat_idx[7]], i_ptrs->Mats[mat_idx[8]]};

    int hit_point[3];
    hit_point[0] = i_ray->origin[0] + fip_mult(t, i_ray->dir[0]);
    hit_point[1] = i_ray->origin[1] + fip_mult(t, i_ray->dir[1]);
    hit_point[2] = i_ray->origin[2] + fip_mult(t, i_ray->dir[2]);

    // ambient
    o_total_light[0] = fip_mult(ka[0], FIP_AMB);
    o_total_light[1] = fip_mult(ka[1], FIP_AMB);
    o_total_light[2] = fip_mult(ka[2], FIP_AMB);

    int lidx;
    for (lidx = 0; lidx < num_lights * LIGHTS_ELEM_SIZE; lidx += LIGHTS_ELEM_SIZE)
    {
        const int* source = &lights[lidx]; // size=3
        const int* color = source + 3; // size=3

        // source - hit_point
        int l_dir[3];
        l_dir[0] = source[0] - hit_point[0];
        l_dir[1] = source[1] - hit_point[1];
        l_dir[2] = source[2] - hit_point[2];
        // normalize
        fip_normalize(l_dir);

        // shadow: ignored for now

        // diffuse
        int diff_light_term = MAX(fip_mult(normal[0], l_dir[0]) +
            fip_mult(normal[1], l_dir[1]) +
            fip_mult(normal[2], l_dir[2]), 0);
        int diff_light[3];
        diff_light[0] = fip_mult(diff_light_term, fip_mult(kd[0], color[0]));
        diff_light[1] = fip_mult(diff_light_term, fip_mult(kd[1], color[1]));
        diff_light[2] = fip_mult(diff_light_term, fip_mult(kd[2], color[2]));

        // specular: ignored for now

        // update total_light
        o_total_light[0] += diff_light[0];
        o_total_light[1] += diff_light[1];
        o_total_light[2] += diff_light[2];
    }

    // cut color > 1
    o_total_light[0] = MIN(o_total_light[0], FIP_ALMOST_ONE);
    o_total_light[1] = MIN(o_total_light[1], FIP_ALMOST_ONE);
    o_total_light[2] = MIN(o_total_light[2], FIP_ALMOST_ONE);
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

int raytrace(unsigned* data, int size, char** pimg, int* pimg_size)
{
    volatile int* sdram = (int*)(SDRAM);
    volatile uint8_t* rtdev = (uint8_t*)(LWBRIDGE + RAYTRACE_BASEOFF);
    /*
    // ray
    sdram[0] = 0; sdram[1] = 0; sdram[2] = 1 << 16;
    sdram[3] = 0; sdram[4] = 0; sdram[5] = -1 << 16;
    
    #define NTRIS 3

    const int tris[NTRIS][9] = {
        { 0, 2 << 16, -2 << 16, -2 << 16, -2 << 16, -2 << 16, 2 << 16, -2 << 16, -2 << 16 },
        { 0, 2 << 16, 0, -2 << 16, -2 << 16, 0, 2 << 16, -2 << 16, 0 },
        { 0, 2 << 16, 0, -2 << 16, 2 << 16, 0, 2 << 16, 2 << 16, 0 }
    };

    sdram[6] = NTRIS;
    
    int sdpos = 7;
    for (int i = 0; i < NTRIS; ++i) {
        VOLATILE_MEMCPY32(sdram + sdpos, tris[i], 9);
        sdpos += 9;
    }
    
    *rtdev = 1;
    
    int intrfd = open(RTINTR_SYSFS, O_RDONLY);
    if (intrfd == -1) {
        perror("intr sysfs open failed\n");
        //goto fail_rt;
        return -1;
    }
    uint8_t rtstat;
    if (read(intrfd, &rtstat, 1) == -1)
    {
        perror("intr sysfs read failed\n");
        close(intrfd);
        return -1;
        // todo: read may also fail due to Ctrl+c. In this
        // case we probably want to reset the FPGA
    }
    close(intrfd);
    
    int hit = sdram[6];
    int t = sdram[7];
    int tri_id = sdram[8];
    
    printf("hit: %d, t: %d, tri_id: %d\n\n", hit, t, tri_id);
    return -1;
    */


    
    int resX = data[1], resY = data[2];
    *pimg_size = resX * resY * 3;

    Camera cam;
    init_camera(&data[data[5]], &cam, resX, resY);

    Ray ray;
    memcpy(ray.origin, cam.eye, 3 * sizeof(int));
    memcpy(ray.dir, cam.base_dir, 3 * sizeof(int));

    int numBV = data[4];
    int numL = data[3];

    const int* const FVtop = &data[data[7]];
    const int* const Ltop = &data[data[10]];
    const int* const FMtop = &data[data[9]];
    const int* const BVtop = &data[data[6]];

    const int* Bvp = BVtop; // start of each BV
    const int* Vp = FVtop; // verts in each BV

    byte* pixelBuf = (byte*)malloc(resX * resY * 3);

    //auto tbeg = std::chrono::high_resolution_clock::now(); // todo

    printf("Start raytracing...\n");

    int nbatches = 0;
    int nfailedbatches = 0;

    for (int i = 0; i < resY; ++i)
    {
        for (int j = 0; j < resX; ++j)
        {
            int min_t = FIP_MAX;
            int min_tri_id = -1;

            Vp = FVtop;
            Bvp = BVtop;

            VOLATILE_MEMCPY32(sdram, ray.origin, 3);
            VOLATILE_MEMCPY32(sdram + 3, ray.dir, 3);

            for (int k = 0; k < numBV; ++k)
            {
                if (ray_intersect_box(&ray, Bvp))
                {
                    //printf("Processing BV %d\n", k);
                    const int* vs = Vp;
                    int bv_ntris = Bvp[6];

                    int cpu_min_t = FIP_MAX;
                    int cpu_min_id = -1;
                    int cpu_hit = 0;

                    for (int l = 0; l < bv_ntris; ++l)
                    {
                        int t;
                        if (new_ray_intersect_tri(vs, &ray, &t) && t < cpu_min_t) {
                            cpu_hit = 1;
                            cpu_min_t = t;
                            cpu_min_id = (vs - FVtop) / 9;
                        }
                        vs += 9;
                    }

                    // ---------- fgpa ----------------

                    sdram[6] = bv_ntris;
                    // Copy data to SDRAM.
                    // this is slow. there are faster ways of doing this:
                    // https://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/HPS_peripherials/FPGA_addr_index.html
                    VOLATILE_MEMCPY32(sdram + 7, Vp, bv_ntris * 9);

                    *rtdev = 1; // start intersection

                    // Wait for interrupt from FPGA
                    int intrfd = open(RTINTR_SYSFS, O_RDONLY);
                    if (intrfd == -1) {
                        perror("intr sysfs open failed\n");
                        goto fail_rt;
                    }
                    uint8_t rtstat;
                    if (read(intrfd, &rtstat, 1) == -1)
                    {
                        perror("intr sysfs read failed\n");
                        close(intrfd);
                        goto fail_rt;
                        // todo: read may also fail due to Ctrl+c. In this
                        // case we probably want to reset the FPGA
                    }
                    close(intrfd);

                    int batch_hit = sdram[6];
                    int batch_t = sdram[7];
                    int batch_tri_id = sdram[8] + ((Vp - FVtop) / 9);

                    if (sdram[8] >= bv_ntris) {
                        printf("batch: %d, bad index of %d, %d\n", k, sdram[8], batch_tri_id);
                        goto fail_rt;
                    }

                    if (batch_hit && batch_t < min_t) {
                        min_t = batch_t;
                        min_tri_id = batch_tri_id;
                    }

                    if (cpu_hit != batch_hit || cpu_min_t != batch_t || cpu_min_id != batch_tri_id) {
                        printf("Failed batch id: %d\n", k);
                        printf("ray: %d %d %d %d %d %d\n", ray.origin[0], ray.origin[1], ray.origin[2], ray.dir[0], ray.dir[1], ray.dir[2]);
                        printf("cpu_hit: %d, cpu_t: %d, cpu_tri_id: %d\n", cpu_hit, cpu_min_t, cpu_min_id);
                        printf("fpga_hit: %d, fpga_t: %d, fpga_tri_id: %d\n\n", batch_hit, batch_t, batch_tri_id);
                        nfailedbatches++;
                    }

                    nbatches++;
                }

                Vp += (Bvp[6] * 9);
                Bvp += 7;
            }

            int light[3] = { 17990, 17990, 17990 }; // gray
            if (min_tri_id != -1) {
                new_blinn_phong_shading(light, &ray, min_tri_id, min_t, numL, Ltop, FMtop, FVtop);
            }

            pixelBuf[3 * (resX * i + j) + 0] = (byte)(light[0] >> 8);
            pixelBuf[3 * (resX * i + j) + 1] = (byte)(light[1] >> 8);
            pixelBuf[3 * (resX * i + j) + 2] = (byte)(light[2] >> 8);

            ray.dir[0] += cam.incr_diru[0];
            ray.dir[1] += cam.incr_diru[1];
            ray.dir[2] += cam.incr_diru[2];
            // rdir += incr_diru;
        }

        ray.dir[0] -= (resX * cam.incr_diru[0]);
        ray.dir[1] -= (resX * cam.incr_diru[1]);
        ray.dir[2] -= (resX * cam.incr_diru[2]);
        // rdir -= (sc.R.first * incr_diru); // reset

        ray.dir[0] += cam.incr_dirv[0];
        ray.dir[1] += cam.incr_dirv[1];
        ray.dir[2] += cam.incr_dirv[2];
        // rdir += incr_dirv;
    }

    printf("Failed %d out of %d batches\n", nfailedbatches, nbatches);

    printf("Finished raytracing\n");
    *pimg = (char*)pixelBuf;
    return 0;

fail_rt:
    free(pixelBuf);
    return -1;



    //for (int i = 1; i < nrecv; ++i) {
    //    sdram[i] = data[i];
    //}
    //free(recvbuf);
    //recvbuf = NULL;
    //
    //
    //*rtdev = 1;
    //
    //// Wait for interrupt from FPGA
    //int intrfd = open(RTINTR_SYSFS, O_RDONLY);
    //if (intrfd == -1) {
    //    perror("intr sysfs open failed\n");
    //    goto fail_rt;
    //}
    //uint8_t rtstat;
    //if (read(intrfd, &rtstat, 1) == -1)
    //{
    //    perror("intr sysfs read failed\n");
    //    close(intrfd);
    //    goto fail_rt;
    //    // todo: read may also fail due to Ctrl+c. In this
    //    // case we probably want to reset the FPGA
    //}
    //close(intrfd);
    //
    //QUIT_IF_SIGINT
    //
    //    unsigned ncopy = nbytes_img / 4;
    //// align to 32-bit, unaligned reads from SDRAM are not supported
    //if ((nbytes_img % 4) != 0) {
    //    ncopy++;
    //}
    //
    //unsigned* sendbuf = (unsigned*)malloc(ncopy);
    //for (unsigned i = 0; i < ncopy; ++i) {
    //    sendbuf[i] = sdram[i];
    //}
}
