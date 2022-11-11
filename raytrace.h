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

typedef enum {
    PLANE,
    SPHERE,
#ifdef QUADRICS
    QUADRIC
#endif
} ObjectType;

typedef enum {
    POINT,
    SPOT
} LightType;

typedef struct Object {
    ObjectType type;
    PixelN diffuseColor, specularColor;
    
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
            float ns;
        };
        
        // Quadric properties
#ifdef QUADRICS
        struct {
            float constants[10];
        };
#endif
    };
} Object;

typedef struct Light {
    LightType type;
    float position[3];
    float direction[3];
    PixelN color;
    float radialA0, radialA1, radialA2, angularA0, theta, cosTheta;
} Light;

typedef struct Camera {
    int imageWidth, imageHeight;
    float vpWidth, vpHeight;
    float vpDistance;
    float origin[3];
} Camera;

// typedef struct SceneData {
//     Camera camera;
    
//     Object *objects;
//     size_t numObjects;
    
//     Light *lights;
//     size_t numLights;
// } SceneData;

// TODO: Use realloc to dynamically resize objects? Should be fine on stack
typedef struct SceneData {
    Camera camera;
    
    Object objects[OBJECT_LIMIT];
    size_t numObjects;
    
    Light lights[OBJECT_LIMIT];
    size_t numLights;
} SceneData;

// TODO: Ray struct?
// typedef struct Ray {
//     float R0[3];
//     float Rd[3];
// }

#ifdef QUADRICS
extern inline float raycastQuadric(float *R0, float *Rd, float *constants);
#endif

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
extern inline float raycastSphere(float *R0, float *Rd, float *sphereCenter, float radius);

extern inline float calculateIllumination(float radialAtt, float angularAtt, float diffuseColor,
                                          float specularColor, float lightColor, float *L,
                                          float *N, float *R, float *V, float ns);

extern inline PixelN illuminate(float *cameraOrigin, float *point, Object *objects,
                                size_t numObjects, Object *object, Light *lights,
                                size_t numLights);

extern inline void getIntersectionPoint(float *R0, float *Rd, float t, float *intersectionPoint);

extern inline Object *raycast(float *R0, float *Rd, Object *objects, size_t numObjects,
                              Object *ignoredObject, float *nearestT);

extern inline void renderScene(SceneData *sceneData, Pixel *image);

extern inline void parseSceneInput(FILE *inputFile, SceneData *sceneData);
