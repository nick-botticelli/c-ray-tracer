#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ppmrw.h"
#include "v3math.h"

#define OBJECT_LIMIT 128
#define INPUT_BUFFER_SIZE 32

#define DEFAULT_NS 20.0f

#define OUTSIDE_IOR 1.00029 // Air (can also be 1.0)

#define RECURSION_DEPTH 32

typedef enum {
    PLANE   = 0,
    SPHERE  = 1,
    QUADRIC = 2
} ObjectType;

typedef enum {
    POINT = 0,
    SPOT  = 1
} LightType;

typedef struct {
    float a, b, c, d, e, f, g, h, i, j;
} QuadricVariables;

// TODO: Ray struct?
typedef struct {
    float R0[3];
    float Rd[3];
} Ray;

typedef struct {
    ObjectType type;
    PixelN diffuseColor, specularColor;
    float reflectivity, refractivity, ior, ns;
    
    union {
        // Plane properties
        struct {
            float pn[3]; // A, B, C (plane normal)
            float d;     // D
        };
        
        // Sphere properties
        struct {
            float center[3];
            float radius;
        };
        
        // Quadric properties
        struct {
            QuadricVariables quadricVars;
        };
    };
} Object;

typedef struct {
    LightType type;
    float position[3];
    float direction[3];
    PixelN color;
    float radialA0, radialA1, radialA2, angularA0, theta, cosTheta;
} Light;

typedef struct {
    int imageWidth, imageHeight;
    float vpWidth, vpHeight;
    float vpDistance;
    float origin[3];
} Camera;

// TODO: Use realloc to dynamically resize objects? Should be fine on stack
typedef struct {
    Camera camera;
    
    Object objects[OBJECT_LIMIT];
    size_t numObjects;
    
    Light lights[OBJECT_LIMIT];
    size_t numLights;
} SceneData;

extern inline float raycastQuadric(float *R0, float *Rd, QuadricVariables variables, bool largestT);

/**
 Calculate ray-plane intersection where
 R0 is the 3D origin ray,
 Rd is the normalized 3D ray direction,
 plane is the 3D array representing the plane's normal unit Pn (A, B, C), and
 d is the distance from the origin to the plane (D)
 */
extern inline float raycastPlane(float *R0, float *Rd, float *pn, float d);

/**
 Calculate ray-sphere intersection where
 R0 is the 3D origin ray,
 Rd is the normalized 3D ray direction,
 sphereCenter is the 3D coordinates of the center of the sphere, and
 radius is the radius of the sphere
 */
extern inline float raycastSphere(float *R0, float *Rd, float *sphereCenter, float radius,
                                  bool largestT);

extern inline void calculateNormalVector(Object *object, float *point, float *Rd, float *N);

extern inline void getIntersectionPoint(float *R0, float *Rd, float t, float *intersectionPoint);

extern inline float calculateIllumination(float radialAtt, float angularAtt, float diffuseColor,
                                          float specularColor, float lightColor, float *L,
                                          float *N, float *R, float *V, float ns);

extern inline PixelN illuminate(SceneData *sceneData, Object *object, float *point,
                                PixelN reflectionColor, PixelN refractionColor);

extern inline void raytrace(SceneData *sceneData, Object *object, float *point, float *Rd,
                            int iterationNum, int x, int y, PixelN *reflectionColorOut,
                            PixelN *refractionColorOut);

extern inline Object *raycast(SceneData *sceneData, float *R0, float *Rd, Object *ignoredObject,
                              bool largestT, float *nearestT);

extern inline void renderScene(SceneData *sceneData, Pixel *image);

extern inline void parseSceneInput(FILE *inputFile, SceneData *sceneData);
