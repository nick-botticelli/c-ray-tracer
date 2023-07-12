#pragma once

#include <stdbool.h>
#include <math.h>

// TODO: Switch (float *) usage to use Vec3
typedef struct Vec3 {
    float x, y, z;
} Vec3;

/**
 Form v3 from a to b
 */
void f3_from_points(float *dst, float *a, float *b);

/**
 Add vectors a and b and store result in dst
 */
void f3_add(float *dst, float *a, float *b);

/**
 Subtract vectors b from a and store result in dst
 */
void f3_subtract(float *dst, float *a, float *b);

/**
 Dot product of vectors a and b
 */
float f3_dot(float *a, float *b);

/**
 Cross product of vectors a and b, with the result stored in dst
 */
void f3_cross(float *dst, float *a, float *b);

/**
 Scale the vector dst by s amount
 */
void f3_scale(float *dst, float s);

/**
 Angle between a and b
 */
float f3_angle(float *a, float *b);

/**
 Reflection v about n
 */
void f3_reflect(float *dst, float *v, float *n);

/**
 Length of vector a
 */
float f3_length(float *a);

/**
 Normalize vector dst to length of 1
 */
void f3_normalize(float *dst, float *a);

/**
 Test if two vectors, a and b, are equal within the specified tolerance
 */
bool f3_equals(float *a, float *b, float tolerance);

/**
 Test if two floats, a and b, are equal within the specified tolerance
 */
bool f_equals(float a, float b, float tolerance);

/**
 Clamps value d between the values min and max
 */
float f_clamp(float d, float min, float max);

/**
 Converts degrees to radians
 */
float f_to_radians(float degrees);



// ########################################################################
// ##################    Inlined function definitions    ##################
// ########################################################################

inline void f3_from_points(float *dst, float *a, float *b) {
    f3_subtract(dst, b, a);
}

inline void f3_add(float *dst, float *a, float *b) {
    dst[0] = a[0] + b[0];
    dst[1] = a[1] + b[1];
    dst[2] = a[2] + b[2];
}

inline void f3_subtract(float *dst, float *a, float *b) {
    dst[0] = a[0] - b[0];
    dst[1] = a[1] - b[1];
    dst[2] = a[2] - b[2];
}

inline float f3_dot(float *a, float *b) {
    float result;

    result =  a[0] * b[0];
    result += a[1] * b[1];
    result += a[2] * b[2];

    return result;
}

inline void f3_cross(float *dst, float *a, float *b) {
    int x = 0, y = 1, z = 2;
    float dstTmp[3] = {};
    
    dstTmp[x] = (a[y] * b[z]) - (a[z] * b[y]);
    dstTmp[y] = (a[z] * b[x]) - (a[x] * b[z]);
    dstTmp[z] = (a[x] * b[y]) - (a[y] * b[x]);
    
    dst[0] = dstTmp[0];
    dst[1] = dstTmp[1];
    dst[2] = dstTmp[2];
}

inline void f3_scale(float *dst, float s) {
    dst[0] *= s;
    dst[1] *= s;
    dst[2] *= s;
}

inline float f3_angle(float *a, float *b) {
    return acosf(f3_dot(a, b) / (f3_length(a) * f3_length(b)));
}

inline void f3_reflect(float *dst, float *v, float *n) {
    float nTmp[3] = { n[0], n[1], n[2] };

    float scale = -2 * f3_dot(v, nTmp);
    f3_scale(nTmp, scale);
    f3_add(dst, nTmp, v);
}

inline float f3_length(float *a) {
    int x = 0, y = 1, z = 2;

    return sqrtf(a[x]*a[x] + a[y]*a[y] + a[z]*a[z]);
}

inline void f3_normalize(float *dst, float *a) {
    float lengthInverse = 1 / f3_length(a);
    
    dst[0] = a[0] * lengthInverse;
    dst[1] = a[1] * lengthInverse;
    dst[2] = a[2] * lengthInverse;
}

inline bool f3_equals(float *a, float *b, float tolerance) {
    float dst[3] = {};
    
    f3_subtract(dst, a, b);
    
    dst[0] = fabsf(dst[0]);
    dst[1] = fabsf(dst[1]);
    dst[2] = fabsf(dst[2]);
    
    return dst[0] <= tolerance && dst[1] <= tolerance && dst[2] <= tolerance;
}

inline bool f_equals(float a, float b, float tolerance) {
    return fabsf(a - b) <= tolerance;
}

inline float f_clamp(float d, float min, float max) {
    float clampedMin = d < min ? min : d;

    return clampedMin > max ? max : clampedMin;
}

inline float f_to_radians(float degrees) {
    return degrees * (M_PI / 180);
}
