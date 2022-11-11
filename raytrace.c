#include "raytrace.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ppmrw.h"
#include "v3math.h"

#ifdef QUADRICS
inline float raycastQuadric(float *R0, float *Rd, float *constants) {
    float x0 = R0[0];
    float y0 = R0[1];
    float z0 = R0[2];
    
    float xd = Rd[0];
    float yd = Rd[1];
    float zd = Rd[2];
    
    float A = constants[0];
    float B = constants[1];
    float C = constants[2];
    float D = constants[3];
    float E = constants[4];
    float F = constants[5];
    float G = constants[6];
    float H = constants[7];
    float I = constants[8];
    float J = constants[9];
    
    // Axd2 + Byd2 + Czd2 + Dxdyd + Exdzd + Fydzd
    float Aq = (A * (xd * xd)) + (B * (yd * yd)) + (C * (zd * zd)) + (D * (xd * yd))
             + (E * (xd * zd)) + (F * (yd * zd));
    
    // 2*Axoxd + 2*Byoyd + 2*Czozd + D(xoyd + yoxd) + E(xozd + zoxd) + F(yozd + ydzo) + Gxd + Hyd
    //   + Izd
    float Bq = (2 * A * x0 * xd) + (2 * B * y0 * yd) + (2 * C * z0 * zd)
             + (D * (x0 * yd + y0 * xd)) + (E * (x0 * zd + z0 * xd)) + (F * (y0 * zd + yd * z0))
             + (G * xd) + (H * yd) + (I * zd);
    
    // Axo2 + Byo2 + Czo2 + Dxoyo + Exozo + Fyozo + Gxo + Hyo + Izo + J
    float Cq = (A * (x0 * x0)) + (B * (y0 * y0)) + (C * (z0 * z0)) + (D * x0 * y0) + (E * x0 * z0)
             + (F * y0 * z0) + (G * z0) + (H * y0) + (I * z0) + J;
    
    // If Aq is zero, then t = -Cq / Bq
    if (Aq == 0)
        return -Cq / Bq;
    
    // Bq2 - 4AqCq
    float discriminant = (Bq * Bq) - (4 * Aq * Cq);
    
    if (discriminant < 0)
        return 0;
    
    // ( - Bq - ((Bq2 - 4AqCq))^0.5)/ 2Aq
    float t0 = (-Bq - sqrtf(discriminant)) / (2 * Aq);
    
    if (t0 > 0)
        return t0;
    
    // ( - Bq + ((Bq2 - 4AqCq))^0.5)/ 2Aq
    float t1 = (-Bq + sqrtf(discriminant)) / (2 * Aq);
    return t1;
}
#endif

inline float raycastPlane(float *R0, float *Rd, float *pn, float d) {
    // Rd = [Xd, Yd, Zd]
    // R(t) = R0 + t*Rd, t > 0
    // plane equation : Ax + By + Cz + D = 0
    // intersection is A(X0 + Xd * t) + B(Y0 + Yd * t) + (Z0 + Zd * t) + D = 0
    // t = -(AX0 + BY0 = CZ0 + D) / (AXd + BYd + CZd) = -(Pn * R0 + D) / (Pn * Rd)
    // plane = [A B C D]

    // calculate pn
    float vD = v3_dot_product(pn, Rd);

    // If vD is zero, then the ray is parallel to the plane
    if (vD == 0) {
        return 0;
    }

    float t = -(v3_dot_product(pn, R0) + d) / vD;
    return t;
}

inline float raycastSphere(float *R0, float *Rd, float *sphereCenter, float radius) {
    // Used in multiple calculations
    // R0 minus center
    float R0mC[3] = {};
    v3_subtract(R0mC, R0, sphereCenter);
    
    float B = 2 * v3_dot_product(Rd, R0mC); // R0mC = (X0-Xc)
    
    float C = v3_dot_product(R0mC, R0mC); // (X0-Xc)^2
    C -= radius * radius; // Sr^2
    
    float discriminant = (B * B) - (4 * C);
    
    // If discriminant is negative, there is no intersection
    if (discriminant < 0)
        return 0;
    
    // 1 or 2 intersections (discriminant = 0 is 1 tangential intersection)
    
    float t0 = (-B - sqrtf(discriminant)) / 2;
    
    // Return t0 if positive, otherwise calculate and return t1
    if (t0 >= 0)
        return t0;
    
    // TODO: This code never runs?
    float t1 = (-B + sqrtf(discriminant)) / 2;
    return t1;
}

inline void getIntersectionPoint(float *R0, float *Rd, float t, float *intersectionPoint) {
    // Ri = [xi, yi, zi] = [x0 + xd * ti ,  y0 + yd * ti,  z0 + zd * ti]
    intersectionPoint[0] = R0[0] + Rd[0] * t;
    intersectionPoint[1] = R0[1] + Rd[1] * t;
    intersectionPoint[2] = R0[2] + Rd[2] * t;
}

inline void calculateNormalVector(Object *object, float *point, float *N) {
    switch (object->type) {
        case PLANE:
            N[0] = object->pn[0];
            N[1] = object->pn[1];
            N[2] = object->pn[2];
            break;
        case SPHERE:
            v3_from_points(N, object->center, point);
            v3_normalize(N, N);
            break;
    }
}

inline float calculateIllumination(float radialAtt, float angularAtt, float diffuseColor,
                                   float specularColor, float lightColor, float *L, float *N,
                                   float *R, float *V, float ns) {
    float NdotL = v3_dot_product(N, L);
    float VdotR = v3_dot_product(V, R);

    // f1,rad_atten * f1,ang_atten * (kd * Il * (N dot L) + ks * Il * (R * V)^n)
    float illumination = radialAtt * angularAtt
           * (diffuseColor * lightColor * NdotL);

    if (VdotR > 0 && NdotL > 0)
        illumination += specularColor * lightColor * powf(VdotR, ns);

    return illumination;
}

inline PixelN illuminate(float *cameraOrigin, float *Rd, float *point, Object *objects,
                         size_t numObjects, Object *object, Light *lights, size_t numLights, PixelN reflectionColor) {
    // point  - the point we are coloring
    // object - the object the point is on
    // Rd     - the view vector to the point
    
    PixelN color = { 0, 0, 0 };
    PixelN reflectedColor = { 0, 0, 0 };
    float reflectRefractModifier = 1 - object->reflectivity - object->refractivity;
    //float reflectRefractModifier = 1;
    
    for (size_t index = 0; index < numLights; index++) {
        Light *light = &lights[index];

        // Light position - point
        float Rd[3] = {};
        v3_subtract(Rd, light->position, point);
        v3_normalize(Rd, Rd);

        float nearestT;
        raycast(point, Rd, objects, numObjects, object, &nearestT);

        // Length from point to light
        float pointLightVector[3] = { 0, 0, 0 };
        v3_from_points(pointLightVector, point, light->position);
        float distance = v3_length(pointLightVector);
        
        if (nearestT > 0 && nearestT < distance)
            continue;

        // Point to light vector
        float L[3] = {};
        v3_normalize(L, pointLightVector);

        // Surface normal vector
        float N[3] = {};
        calculateNormalVector(object, point, &N);

        float V[3] = {};
        v3_from_points(V, point, cameraOrigin);
        v3_normalize(V, V);

        float VO[3] = { -L[0], -L[1], -L[2] };

        float R[3] = { 0, 0, 0 };
        v3_reflect(R, VO, N); // L instead of VO as per Palmer's advice; so why does that not work?
        //v3_reflect(R, L, N);
        v3_normalize(R, R);

        float VL[3] = {};

        float radialAtt = 1 / (light->radialA0 + (light->radialA1 * distance)
                                + (light->radialA2 * (distance * distance)));

        float angularAtt = 0;
        if (light->type == SPOT) {
            v3_from_points(VL, light->position, light->direction);
            v3_normalize(VL, VL);

            float VOdotVL = v3_dot_product(VO, VL);

            // If theta is zero, bad things may happen
            if (light->theta == 0) {
                // Set L to 0 vector to stop generating specular light
                L[0] = 0;
                L[1] = 0;
                L[2] = 0;
            }
            else {
                if (VOdotVL < light->cosTheta) {
                    angularAtt = 0;

                    // Set L to 0 vector to stop generating specular light
                    L[0] = 0;
                    L[1] = 0;
                    L[2] = 0;
                }
                else {
                    angularAtt = powf(VOdotVL, light->angularA0);
                }
            }
        }
        else {
            v3_from_points(VL, light->position, point);
            v3_normalize(VL, VL);
            angularAtt = 1;
        }

        color.r += clamp(reflectRefractModifier
                         * calculateIllumination(radialAtt, angularAtt, object->diffuseColor.r,
                                                 object->specularColor.r, light->color.r, L, N, R,
                                                 V, object->ns)
                         + reflectionColor.r, 0, 1);
        color.g += clamp(reflectRefractModifier
                         * calculateIllumination(radialAtt, angularAtt, object->diffuseColor.g,
                                                 object->specularColor.g, light->color.g, L, N, R,
                                                 V, object->ns)
                         + reflectionColor.g, 0, 1);
        color.b += clamp(reflectRefractModifier
                         * calculateIllumination(radialAtt, angularAtt, object->diffuseColor.b,
                                                 object->specularColor.b, light->color.b, L, N, R,
                                                 V, object->ns)
                         + reflectionColor.b, 0, 1);
    }

    //color += ambient;

    return color;
}

// Returns reflection color
inline PixelN raytrace(Object *object, float *point, float *Rd, float *cameraOrigin,
                       Object *objects, size_t numObjects, Light *lights, size_t numLights,
                       int iterationNum) {
    PixelN reflectionColor = { 0, 0, 0 }; 

    if (iterationNum > ITERATIONS_MAX)
        return reflectionColor; // Black
    
    //printf("%p, %p, %p, %p, %u, %p, %p, %u, %i", cameraOrigin, Rd, point, objects, numObjects, object, lights, numLights, iterationNum);
    float pointNormal[3] = { 0, 0, 0 };
    //printf("\tpointNormal addr: %p\n", pointNormal);
    calculateNormalVector(object, point, pointNormal);

    // Get reflected ray direction from intersected point
    float reflectedRay[3] = { 0, 0, 0 };
    v3_reflect(reflectedRay, Rd, pointNormal);
    v3_normalize(reflectedRay, reflectedRay);

    // Get the new object and new nearest t from reflected ray
    float newNearestT;
    Object *newObject = raycast(point, reflectedRay, objects, numObjects, object, &newNearestT);

    // If null, then there are no other objects to raytrace
    if (newObject == NULL)
        return reflectionColor; // Black

    // Calculate intersection point at new object
    float newPoint[3] = { 0, 0, 0 };
    getIntersectionPoint(point, reflectedRay, newNearestT, newPoint);

    //printf("\tbefore recursion (%i)\n", iterationNum);

    reflectionColor = raytrace(newObject, newPoint, reflectedRay, cameraOrigin, objects,
                               numObjects, lights, numLights, iterationNum + 1);
    
    reflectionColor.r *= object->reflectivity;
    reflectionColor.g *= object->reflectivity;
    reflectionColor.b *= object->reflectivity;

    // Recursion
    return illuminate(cameraOrigin, reflectedRay, newPoint, objects, numObjects, newObject, lights,
                      numLights, reflectionColor);
}

inline Object *raycast(float *R0, float *Rd, Object *objects, size_t numObjects,
                       Object *ignoredObject, float *nearestT) {
    Object *curNearestObject = NULL;
    float curNearestT = INFINITY;
    
    for (size_t index = 0; index < numObjects; index++) {
        Object *object = &objects[index];
        float t = 0;

        if (object == ignoredObject)
            continue;
        
        switch (object->type) {
            case PLANE:
                t = raycastPlane(R0, Rd, object->pn, object->d);
                break;
            case SPHERE:
                t = raycastSphere(R0, Rd, object->center, object->radius);
                break;
#ifdef QUADRICS
            case QUADRIC:
                t = raycastQuadric(R0, Rd, object->constants);
                break;
#endif
        }
        
        // If intersection exists (not 0) and is positive (in front of camera), set it to nearest
        if (t > 0 && t < curNearestT) {
            curNearestObject = object;
            curNearestT = t;
        }
    }
    
    *nearestT = curNearestT;
    return curNearestObject;
}

inline void renderScene(SceneData *sceneData, Pixel *image) {
    Camera camera = sceneData->camera;
    float *R0 = camera.origin;
    float dX = camera.vpWidth / camera.imageWidth;
    float dY = camera.vpHeight / camera.imageHeight;
    float PxInitial = (camera.vpWidth * -.5) + (dX * .5);
    float PyInitial = (camera.vpHeight * .5) + (dY * .5);
    float Pz = -camera.vpDistance;
    
#ifdef OPENMP
#pragma omp parallel for firstprivate(sceneData, R0, dX, dY, PxInitial, PyInitial, Pz) \
                         private(camera)
#endif
    for (int y = 0; y < sceneData->camera.imageHeight; y++) {
        camera = sceneData->camera;

        //float Px = PxInitial + (dX * x);
        float Py = PyInitial - (dY * y);
        int rowIndex = y * camera.imageWidth;
        
        for (int x = 0; x < camera.imageWidth; x++) {
            // Construct R0 and Rd vectors
            
            //float Py = PyInitial - (dY * y);
            float Px = PxInitial + (dX * x);
            float P[3] = { Px, Py, Pz };
            
            // P - R0
            float Rd[3] = {};
            v3_subtract(Rd, P, R0);
            v3_normalize(Rd, Rd);
            
            float nearestT;
            Object *nearestObject = raycast(R0, Rd, sceneData->objects, sceneData->numObjects,
                                            NULL, &nearestT);

            float intersectionPoint[3] = {};
            getIntersectionPoint(R0, Rd, nearestT, &intersectionPoint);
            
            // If nearestObject is not null, there is at least one intersection
            if (nearestObject != NULL) {
                // PixelN pixelColorN = illuminate(sceneData->camera.origin, Rd, intersectionPoint,
                //                                 sceneData->objects, sceneData->numObjects,
                //                                 nearestObject, sceneData->lights,
                //                                 sceneData->numLights);
                
                PixelN pixelColorN = raytrace(nearestObject, intersectionPoint, Rd, camera.origin,
                                              sceneData->objects, sceneData->numObjects,
                                              sceneData->lights, sceneData->numLights, 1);

                // Convert from PixelN to Pixel for PPM output
                Pixel pixelColor;
                pixelColor.r = pixelColorN.r * 255;
                pixelColor.g = pixelColorN.g * 255;
                pixelColor.b = pixelColorN.b * 255;

                image[rowIndex + x] = pixelColor;
            }
        }
    }
}

inline void parseSceneInput(FILE *inputFile, SceneData *sceneData) {
    Object *curObject;
    Light *curLight;
    size_t objIndex = 0, lightIndex = 0;
    char inputBuf[INPUT_BUFFER_SIZE];

    // objIndex should equal the length after this loop
    // TODO: Check for OBJECT_LIMIT (array max length)
    while (fscanf(inputFile, "%s", inputBuf) == 1) {
        curObject = &sceneData->objects[objIndex];
        curLight = &sceneData->lights[lightIndex];

        curObject->refractivity = 0;
        
        if (strcmp(inputBuf, "camera,") == 0) {
            const int numProperties = 2;
            for (int i = 0; i < numProperties; i++) {
                fscanf(inputFile, "%s", inputBuf);
                
                if (strcmp(inputBuf, "width:")) {
                    fscanf(inputFile, " %f", &sceneData->camera.vpWidth);
                }
                else if (strcmp(inputBuf, "height:")) {
                    // TODO: Why does this leak 3 times with ASAN?
                    fscanf(inputFile, " %f", &sceneData->camera.vpHeight);
                }
                
                // Skip comma
                // TODO: Macro this?
                if (i != numProperties - 1)
                    fscanf(inputFile, "%s", inputBuf);
            }
        }
        else if (strcmp(inputBuf, "plane,") == 0) {
            float position[3];
            
            const int numProperties = 5;
            for (int i = 0; i < 5; i++) {
                fscanf(inputFile, "%s", inputBuf);

                if (strcmp(inputBuf, "normal:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curObject->pn[0],
                           &curObject->pn[1], &curObject->pn[2]);
                }
                else if (strcmp(inputBuf, "position:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &position[0], &position[1], &position[2]);
                }
                else if (strcmp(inputBuf, "diffuse_color:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curObject->diffuseColor.r,
                           &curObject->diffuseColor.g, &curObject->diffuseColor.b);
                }
                else if (strcmp(inputBuf, "specular_color:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curObject->specularColor.r,
                           &curObject->specularColor.g, &curObject->specularColor.b);
                }
                else if (strcmp(inputBuf, "reflectivity:") == 0) {
                    fscanf(inputFile, " %f", &curObject->reflectivity);
                }
                
                // Skip comma
                if (i != numProperties - 1)
                    fscanf(inputFile, "%s", inputBuf);
            }

            PixelN specularColor = { 0, 0, 0 };
            
            curObject->type = PLANE;
            curObject->d = -v3_dot_product(position, curObject->pn);
            curObject->specularColor = specularColor;
            objIndex++;
        }
        else if (strcmp(inputBuf, "sphere,") == 0) {
            bool hasNs = false;

            const int numProperties = 8;
            for (int i = 0; i < numProperties; i++) {
                fscanf(inputFile, "%s", inputBuf);

                if (strcmp(inputBuf, "radius:") == 0) {
                    fscanf(inputFile, " %f", &curObject->radius);
                }
                else if (strcmp(inputBuf, "position:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curObject->center[0],
                           &curObject->center[1], &curObject->center[2]);
                }
                else if (strcmp(inputBuf, "diffuse_color:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curObject->diffuseColor.r,
                           &curObject->diffuseColor.g, &curObject->diffuseColor.b);
                }
                else if (strcmp(inputBuf, "specular_color:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curObject->specularColor.r,
                    &curObject->specularColor.g, &curObject->specularColor.b);
                }
                else if (strcmp(inputBuf, "ns:") == 0) {
                    fscanf(inputFile, " %f", &curObject->ns);
                    hasNs = true;
                }
                else if (strcmp(inputBuf, "reflectivity:") == 0) {
                    fscanf(inputFile, " %f", &curObject->reflectivity);
                }
                else if (strcmp(inputBuf, "refractivity:") == 0) {
                    fscanf(inputFile, " %f", &curObject->refractivity);
                }
                else if (strcmp(inputBuf, "ior:") == 0) {
                    fscanf(inputFile, " %f", &curObject->ior);
                }
                
                // Check for existence of comma before optional ns property
                if (i == numProperties - 2) {
                    char curChar = fgetc(inputFile);

                    // If there is no comma, there are only 4 (no ns)
                    if (curChar != ',') {
                        ungetc(curChar, inputFile);
                        break;
                    }
                }
                // Skip comma
                else if (i != numProperties - 1)
                    fscanf(inputFile, "%s", inputBuf);
            }

            if (!hasNs)
                curObject->ns = DEFAULT_NS;
            hasNs = false;
            
            curObject->type = SPHERE;
            // TODO: Increment & set new pointer with 1 statemnent (macro/function?)
            objIndex++;
        }
#ifdef QUADRICS
        else if (strcmp(inputBuf, "quadric,") == 0) {
            const int numProperties = 2;
            for (int i = 0; i < numProperties; i++) {
                fscanf(inputFile, "%s", inputBuf);

                if (strcmp(inputBuf, "diffuse_color:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curObject->diffuseColor.r,
                           &curObject->diffuseColor.g, &curObject->diffuseColor.b);
                }
                else if (strcmp(inputBuf, "constants:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
                           &curObject->constants[0], &curObject->constants[1],
                           &curObject->constants[2], &curObject->constants[3],
                           &curObject->constants[4], &curObject->constants[5],
                           &curObject->constants[6], &curObject->constants[7],
                           &curObject->constants[8], &curObject->constants[9]);
                }
                
                // Skip comma
                if (i != numProperties - 1)
                    fscanf(inputFile, "%s", inputBuf);
            }
            
            curObject->type = QUADRIC;
            objIndex++;
        }
#endif
        else if (strcmp(inputBuf, "light,") == 0) {
            bool hasDirection = false;
            
            const int numProperties = 8;
            for (int i = 0; i < numProperties; i++) {
                fscanf(inputFile, "%s", inputBuf);

                if (strcmp(inputBuf, "position:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curLight->position[0],
                           &curLight->position[1], &curLight->position[2]);
                }
                else if (strcmp(inputBuf, "color:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curLight->color.r, &curLight->color.g,
                           &curLight->color.b);
                }
                else if (strcmp(inputBuf, "direction:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curLight->direction[0],
                           &curLight->direction[1], &curLight->direction[2]);
                    hasDirection = true;
                }
                else if (strcmp(inputBuf, "theta:") == 0) {
                    float theta;
                    fscanf(inputFile, " %f", &theta);
                    curLight->theta = toRadians(theta);
                    curLight->cosTheta = cosf(curLight->theta);
                }
                else if (strcmp(inputBuf, "radial-a0:") == 0) {
                    fscanf(inputFile, " %f", &curLight->radialA0);
                }
                else if (strcmp(inputBuf, "radial-a1:") == 0) {
                    fscanf(inputFile, " %f", &curLight->radialA1);
                }
                else if (strcmp(inputBuf, "radial-a2:") == 0) {
                    fscanf(inputFile, " %f", &curLight->radialA2);
                }
                else if (strcmp(inputBuf, "angular-a0:") == 0) {
                    fscanf(inputFile, " %f", &curLight->angularA0);
                }

                // Check for existence of comma at index 6 - 1 (8 - 3)
                if (i == numProperties - 3) {
                    char curChar = fgetc(inputFile);

                    // If there is no comma, there are only 6 properties -> point light
                    if (curChar != ',') {
                        ungetc(curChar, inputFile);
                        break;
                    }
                }
                // Skip comma
                else if (i != numProperties - 1)
                    fscanf(inputFile, "%s", inputBuf);
            }
            
            curLight->type = hasDirection ? SPOT : POINT;
            hasDirection = false;
            lightIndex++;
        }
    }

    fclose(inputFile);

    float camerOrigin[3] = { 0, 0, 0 };

    sceneData->numObjects = objIndex;
    sceneData->numLights = lightIndex;
    sceneData->camera.origin[0] = camerOrigin[0];
    sceneData->camera.origin[1] = camerOrigin[1];
    sceneData->camera.origin[2] = camerOrigin[2];
}

int main(int argc, const char *argv[]) {
    if (argc != 5) {
        fprintf(stderr, "Error: Wrong number of arguments!\n");
        return EXIT_FAILURE;
    }

    const int width = atoi(argv[1]);
    const int height = atoi(argv[2]);
    const char *inputFileName = argv[3];
    const char *outputFileName = argv[4];
    
    Pixel *image = calloc(width * height, sizeof(Pixel));
    FILE *inputFile = fopen(inputFileName, "r");

    if (inputFile == NULL) {
        fprintf(stderr, "Error: Could not open input file \"%s\"!\n", inputFileName);
        return EXIT_FAILURE;
    }

    SceneData sceneData = {};
    sceneData.camera.imageWidth = width;
    sceneData.camera.imageHeight = height;
    sceneData.camera.vpDistance = 1;
    parseSceneInput(inputFile, &sceneData);
    
    renderScene(&sceneData, image);

    PPM outputPpm;
    outputPpm.format = 6;
    outputPpm.maxColorVal = 255;
    outputPpm.width = width;
    outputPpm.height = height;
    outputPpm.imageData = image;

    writeImage(outputPpm, outputPpm.format, outputFileName);

    free(image);

    return EXIT_SUCCESS;
}
