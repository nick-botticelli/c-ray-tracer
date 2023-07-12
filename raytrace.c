#include "raytrace.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ppmrw.h"
#include "v3math.h"

inline float raycastQuadric(float *R0, float *Rd, QuadricVariables variables, bool largestT) {
    float x0 = R0[0];
    float y0 = R0[1];
    float z0 = R0[2];
    
    float xd = Rd[0];
    float yd = Rd[1];
    float zd = Rd[2];
    
    float A = variables.a;
    float B = variables.b;
    float C = variables.c;
    float D = variables.d;
    float E = variables.e;
    float F = variables.f;
    float G = variables.g;
    float H = variables.h;
    float I = variables.i;
    float J = variables.j;
    
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

    if (largestT) {
        float t1 = (-Bq + sqrtf(discriminant)) / (2 * Aq);
        return fmaxf(t0, t1);
    }
    else {
        if (t0 > 0)
            return t0;
        
        // ( - Bq + ((Bq2 - 4AqCq))^0.5)/ 2Aq
        float t1 = (-Bq + sqrtf(discriminant)) / (2 * Aq);
        return t1;
    }
}

inline float raycastPlane(float *R0, float *Rd, float *pn, float d) {
    // Rd = [Xd, Yd, Zd]
    // R(t) = R0 + t*Rd, t > 0
    // plane equation : Ax + By + Cz + D = 0
    // intersection is A(X0 + Xd * t) + B(Y0 + Yd * t) + (Z0 + Zd * t) + D = 0
    // t = -(AX0 + BY0 = CZ0 + D) / (AXd + BYd + CZd) = -(Pn * R0 + D) / (Pn * Rd)
    // plane = [A B C D]

    // calculate pn
    float vD = f3_dot(pn, Rd);

    // If vD is zero, then the ray is parallel to the plane
    if (vD == 0) {
        return 0;
    }

    float t = -(f3_dot(pn, R0) + d) / vD;
    return t;
}

inline float raycastSphere(float *R0, float *Rd, float *sphereCenter, float radius,
                           bool largestT) {
    // Used in multiple calculations
    // R0 minus center
    float R0mC[3] = {};
    f3_subtract(R0mC, R0, sphereCenter);
    
    float B = 2 * f3_dot(Rd, R0mC); // R0mC = (X0-Xc)
    
    float C = f3_dot(R0mC, R0mC); // (X0-Xc)^2
    C -= radius * radius; // Sr^2
    
    float discriminant = (B * B) - (4 * C);
    
    // If discriminant is negative, there is no intersection
    if (discriminant < 0)
        return 0;
    
    // 1 or 2 intersections (discriminant = 0 is 1 tangential intersection)
    
    float t0 = (-B - sqrtf(discriminant)) / 2;

    if (largestT) {
        // Get the farthest intersection t (for cases in which the intersection point is outside
        //   the sphere)
        float t1 = (-B + sqrtf(discriminant)) / 2;
        return fmaxf(t0, t1);
    }
    else {
        // Return t0 if positive, otherwise calculate and return t1
        if (t0 >= 0)
            return t0;
        
        // TODO: This code never runs?
        float t1 = (-B + sqrtf(discriminant)) / 2;
        return t1;
    }
}

inline void getIntersectionPoint(float *R0, float *Rd, float t, float *intersectionPoint) {
    // Ri = [xi, yi, zi] = [x0 + xd * ti ,  y0 + yd * ti,  z0 + zd * ti]
    intersectionPoint[0] = R0[0] + Rd[0] * t;
    intersectionPoint[1] = R0[1] + Rd[1] * t;
    intersectionPoint[2] = R0[2] + Rd[2] * t;
}

inline void calculateNormalVector(Object *object, float *point, float *Rd, float *N) {
    switch (object->type) {
        case PLANE:
            N[0] = object->pn[0];
            N[1] = object->pn[1];
            N[2] = object->pn[2];
            break;
        case SPHERE:
            f3_from_points(N, object->center, point);
            f3_normalize(N, N);
            break;
        case QUADRIC:
            N[0] = (2 * object->quadricVars.a * point[0]) + (object->quadricVars.d * point[1])
                 + (object->quadricVars.e * point[2]) + object->quadricVars.g;
            N[1] = (2 * object->quadricVars.b * point[1]) + (object->quadricVars.d * point[0])
                 + (object->quadricVars.f * point[2]) + object->quadricVars.h;
            N[2] = (2 * object->quadricVars.c * point[2]) + (object->quadricVars.e * point[0])
                 + (object->quadricVars.f * point[1]) + object->quadricVars.i;
            f3_normalize(N, N);

            // If Rn dot Rd > 0, reverse Rn
            // if (f3_dot(N, Rd) > 0)
            //     f3_scale(N, -1);

            break;
    }
}

inline float calculateIllumination(float radialAtt, float angularAtt, float diffuseColor,
                                   float specularColor, float lightColor, float *L, float *N,
                                   float *R, float *V, float ns) {
    float NdotL = f3_dot(N, L);
    float VdotR = f3_dot(V, R);

    // f1,rad_atten * f1,ang_atten * (kd * Il * (N dot L) + ks * Il * (R * V)^n)
    float illumination = radialAtt * angularAtt
           * (diffuseColor * lightColor * NdotL);

    if (VdotR > 0 && NdotL > 0)
        illumination += specularColor * lightColor * powf(VdotR, ns);

    return illumination;
}

inline PixelN illuminate(SceneData *sceneData, Object *object, float *point,
                         PixelN reflectionColor, PixelN refractionColor) {
    // point  - the point we are coloring
    // object - the object the point is on
    // Rd     - the view vector to the point
    
    PixelN color = { 0, 0, 0 };
    // float reflectModifier = 1 - object->refractivity; // TODO: Pre-compute? TODO: Is this wrong?
    float reflectModifier = object->reflectivity;
    // float refractModifier = 1 - reflectModifier; // Is this wrong? I have no idea anymore
    float refractModifier = object->refractivity;
    // printf("reflect/refract: %f vs. %f\n", object->reflectivity, object->refractivity);
    float illuminationModifier = 1 - reflectModifier - refractModifier; // TODO: Clamp?
    
    for (size_t index = 0; index < sceneData->numLights; index++) {
        Light *light = &sceneData->lights[index];

        // Light position - point
        float Rd[3] = {};
        f3_subtract(Rd, light->position, point);
        f3_normalize(Rd, Rd);

        float nearestT;
        raycast(sceneData, point, Rd, object, false, &nearestT);

        // Length from point to light
        float pointLightVector[3] = { 0, 0, 0 };
        f3_from_points(pointLightVector, point, light->position);
        float distance = f3_length(pointLightVector);
        
        if (nearestT > 0 && nearestT < distance)
            continue;

        // Point to light vector
        float L[3] = {};
        f3_normalize(L, pointLightVector);

        // Surface normal vector
        float N[3] = {};
        calculateNormalVector(object, point, Rd, &N);

        float V[3] = {};
        f3_from_points(V, point, sceneData->camera.origin);
        f3_normalize(V, V);

        float VO[3] = { -L[0], -L[1], -L[2] };

        float R[3] = { 0, 0, 0 };
        f3_reflect(R, VO, N); // TODO: L instead of VO as per Palmer's advice; so why does that not work?
        //f3_reflect(R, L, N);
        f3_normalize(R, R);

        float VL[3] = {};

        float radialAtt = 1 / (light->radialA0 + (light->radialA1 * distance)
                                + (light->radialA2 * (distance * distance)));

        float angularAtt = 0;
        if (light->type == SPOT) {
            f3_from_points(VL, light->position, light->direction);
            f3_normalize(VL, VL);

            float VOdotVL = f3_dot(VO, VL);

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
            f3_from_points(VL, light->position, point);
            f3_normalize(VL, VL);
            angularAtt = 1;
        }

        color.r += f_clamp(illuminationModifier
                           * calculateIllumination(radialAtt, angularAtt, object->diffuseColor.r,
                                                   object->specularColor.r, light->color.r, L, N,
                                                   R, V, object->ns)
                           + reflectModifier * reflectionColor.r
                           + refractModifier * refractionColor.r,
                         0, 1);
        color.g += f_clamp(illuminationModifier
                           * calculateIllumination(radialAtt, angularAtt, object->diffuseColor.g,
                                                   object->specularColor.g, light->color.g, L, N,
                                                   R, V, object->ns)
                           + reflectModifier * reflectionColor.g
                           + refractModifier * refractionColor.g,
                         0, 1);
        color.b += f_clamp(illuminationModifier
                           * calculateIllumination(radialAtt, angularAtt, object->diffuseColor.b,
                                                   object->specularColor.b, light->color.b, L, N,
                                                   R, V, object->ns)
                           + reflectModifier * reflectionColor.b
                           + refractModifier * refractionColor.b,
                         0, 1);
    }

    //color += ambient;

    // printf("illuminate(): color: (%f, %f, %f); ", color.r, color.g, color.b); // TODO: Remove

    return color;
}

#ifndef NDEBUG
int highestIteration = 0;
#endif

// Returns reflection color
inline void raytrace(SceneData *sceneData, Object *object, float *point, float *Rd,
                     int iterationNum, int x, int y, PixelN *reflectionColorOut,
                     PixelN *refractionColorOut) {
    PixelN reflectionColor = { 0, 0, 0 };
    PixelN refractionColor = { 0, 0, 0 };

#ifndef NDEBUG
    if (highestIteration < iterationNum)
        highestIteration = iterationNum;
#endif

    if (iterationNum > RECURSION_DEPTH) {
        // Black
        // printf("raytrace(): # of iterations exceeded maximum depth (%d vs. %d); returning black", iterationNum, RECURSION_DEPTH); // TODO: Remove
        *reflectionColorOut = reflectionColor;
        *refractionColorOut = refractionColor;
        return;
    }
    
    float pointNormal[3] = { 0, 0, 0 };
    calculateNormalVector(object, point, Rd, pointNormal);

    // Get reflected ray direction from intersected point
    float reflectedRay[3] = { 0, 0, 0 };
    f3_reflect(reflectedRay, Rd, pointNormal);
    f3_normalize(reflectedRay, reflectedRay);

    // Get the new object and new nearest t from reflected ray
    float newNearestT;
    Object *newObject = raycast(sceneData, point, reflectedRay, object, false, &newNearestT);

    // If null, then there are no other objects to raytrace
    if (newObject == NULL) {
        // Black
        // printf("raytrace(): no more objects to raytrace; returning black"); // TODO: Remove
        *reflectionColorOut = reflectionColor;
        *refractionColorOut = refractionColor;
        return;
    }

    // Calculate intersection point at new object
    float newPoint[3] = { 0, 0, 0 };
    getIntersectionPoint(point, reflectedRay, newNearestT, &newPoint); // TODO: Remove?

    // Recursion
    PixelN newReflectionColor, newRefractionColor;
    raytrace(sceneData, newObject, newPoint, reflectedRay, iterationNum + 1, x, y,
             &newReflectionColor, &newRefractionColor);
    
    newReflectionColor.r *= object->reflectivity;
    newReflectionColor.g *= object->reflectivity;
    newReflectionColor.b *= object->reflectivity;

    *reflectionColorOut = illuminate(sceneData, newObject, newPoint, newReflectionColor, newRefractionColor);

    // // Snell's Law
    // // puts("Snell's law!");

    // // Opposite direction for inside normal
    // // float insideRd[3] = {};
    // // f3_scale(Rd, -1);

    // // float insideN[3] = {};
    // // calculateNormalVector(nearestObject, intersectionPoint, insideRd, insideN);

    // float N[3]= {};
    // calculateNormalVector(nearestObject, intersectionPoint, Rd, N);

    // float a[3] = {};
    // f3_cross(a, N, Rd);

    // float b[3] = {};
    // f3_cross(b, a, N);

    // float rhoR = OUTSIDE_IOR;        // IOR of external medium
    // float rhoT = nearestObject->ior; // IOR of inside medium

    // // TODO: Should sinPhi be absolute value?
    // float sinPhi = (rhoR / rhoT) * (f3_dot(Rd, b));
    // float cosPhi = sqrtf(1 - (sinPhi * sinPhi));

    // // ut = −ncosϕ + bsinϕ
    // // Also known as T in Scratchapixel notes
    // float ut[3] = { N[0], N[1], N[2] };
    // f3_scale(ut, -cosPhi);

    // float bSinPhi[3] = { b[0], b[1], b[2] };
    // f3_scale(bSinPhi, sinPhi);

    // f3_add(ut, ut, bSinPhi);
    // // TODO: Normalize?
    // //f3_normalize(ut, ut);

    // // Raycast inside object in direction ut
    // // TODO: Should other objects be intersected if inside object?
    // float largestT;
    // raycast(intersectionPoint, ut, nearestObject, 1, NULL, true, &largestT);

    // float newIntersectionPoint[3] = {};
    // getIntersectionPoint(intersectionPoint, ut, largestT, &newIntersectionPoint);

    // // Set color?
    // PixelN reflectionColorNOut;
    // PixelN refractionColorNOut;
    // raytrace(nearestObject, newIntersectionPoint, Rd, camera.origin,
    //             sceneData->objects, sceneData->numObjects, sceneData->lights,
    //             sceneData->numLights, 1, x, y, &pixelColorNRefracted,
    //             &pixelColorNRefracted);

    // TODO: Fresnel (Scratchapixel)
    // float cosi = f_clamp(-1, 1, dotProduct(I, N));
    // float etai = 1, etat = ior;
    // if (cosi > 0) { std::swap(etai, etat); }
    // // Compute sini using Snell's law
    // float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // // Total internal reflection
    // if (sint >= 1) {
    //     kr = 1;
    // }
    // else {
    //     float cost = sqrtf(std::max(0.f, 1 - sint * sint));
    //     cosi = fabsf(cosi);
    //     float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
    //     float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
    //     kr = (Rs * Rs + Rp * Rp) / 2;
    // }
}

inline Object *raycast(SceneData *sceneData, float *R0, float *Rd,
                       Object *ignoredObject, bool largestT, float *nearestT) {
    Object *curNearestObject = NULL;
    float curNearestT = INFINITY;
    
    for (size_t index = 0; index < sceneData->numObjects; index++) {
        Object *object = &sceneData->objects[index];
        float t = 0;

        if (object == ignoredObject)
            continue;
        
        switch (object->type) {
            case PLANE:
                t = raycastPlane(R0, Rd, object->pn, object->d);
                break;
            case SPHERE:
                t = raycastSphere(R0, Rd, object->center, object->radius, largestT);
                break;
            case QUADRIC:
                t = raycastQuadric(R0, Rd, object->quadricVars, largestT);
                break;
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

// TODO: Is this ifdef needed anymore?
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
            f3_subtract(Rd, P, R0);
            f3_normalize(Rd, Rd);
            
            float nearestT;
            Object *nearestObject = raycast(sceneData, R0, Rd, NULL, false, &nearestT);

            // printf("raycast nearestT: %f; ", nearestT); // TODO: Remove

            float intersectionPoint[3] = {};
            getIntersectionPoint(R0, Rd, nearestT, &intersectionPoint);

            // printf("intersectionPoint: (%f, %f, %f); ", intersectionPoint[0], intersectionPoint[1], intersectionPoint[2]); // TODO: Remove
            
            // If nearestObject is not null, there is at least one intersection.
            if (nearestObject != NULL) {
                PixelN pixelColorN = {}, pixelColorNRefracted = {}, finalPixelColorN = {};

                if (nearestObject->reflectivity > 0 || nearestObject->refractivity > 0) {
                    raytrace(sceneData, nearestObject, intersectionPoint, Rd, 1, x, y,
                             &pixelColorN, &pixelColorNRefracted);
                }

                // Only raytrace if object is reflective
                if (nearestObject->reflectivity > 0) {
                    pixelColorN.r *= nearestObject->reflectivity;
                    pixelColorN.g *= nearestObject->reflectivity;
                    pixelColorN.b *= nearestObject->reflectivity;
                }
                else {
                    pixelColorN.r = 0;
                    pixelColorN.g = 0;
                    pixelColorN.b = 0;
                }

                // TODO: Refraction
                if (nearestObject->refractivity > 0) { // TODO: > 1?
                    // pixelColorN.r *= nearestObject->refractivity;
                }
                else {
                    pixelColorNRefracted.r = 0;
                    pixelColorNRefracted.g = 0;
                    pixelColorNRefracted.b = 0;
                }

                // Repeat last step in raytrace function here since no more recursion (TODO)
                finalPixelColorN = illuminate(sceneData, nearestObject, intersectionPoint,
                                              pixelColorN, pixelColorNRefracted);

                // Convert from PixelN to Pixel for PPM output
                Pixel pixelColor;
                pixelColor.r = finalPixelColorN.r * 255;
                pixelColor.g = finalPixelColorN.g * 255;
                pixelColor.b = finalPixelColorN.b * 255;

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
        curObject->ior = 0;

#ifndef NDEBUG
        printf("parseSceneInput: \"%s\"\n", inputBuf);
#endif
        
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
            bool hasNs = false;
            
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
                
                // Check for existence of comma before optional specular and ns properties
                if (i == numProperties - 3 || i == numProperties - 2) {
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

            PixelN specularColor = { 0, 0, 0 };
            
            curObject->type = PLANE;
            curObject->d = -f3_dot(position, curObject->pn);
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

                // Check for existence of comma before optional refractivity, ior, and ns
                //   properties
                if (i == numProperties - 4 || i == numProperties - 3 || i == numProperties - 2) {
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
        else if (strcmp(inputBuf, "quadric,") == 0) {
            bool hasNs = false;

            const int numProperties = 7;
            for (int i = 0; i < numProperties; i++) {
                fscanf(inputFile, "%s", inputBuf);

                if (strcmp(inputBuf, "diffuse_color:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curObject->diffuseColor.r,
                           &curObject->diffuseColor.g, &curObject->diffuseColor.b);
                }
                else if (strcmp(inputBuf, "specular_color:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f]", &curObject->specularColor.r,
                    &curObject->specularColor.g, &curObject->specularColor.b);
                }
                else if (strcmp(inputBuf, "constants:") == 0) {
                    fscanf(inputFile, " [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
                           &curObject->quadricVars.a, &curObject->quadricVars.b,
                           &curObject->quadricVars.c, &curObject->quadricVars.d,
                           &curObject->quadricVars.e, &curObject->quadricVars.f,
                           &curObject->quadricVars.g, &curObject->quadricVars.h,
                           &curObject->quadricVars.i, &curObject->quadricVars.j);
                }
                else if (strcmp(inputBuf, "reflectivity:") == 0) {
                    fscanf(inputFile, " %f", &curObject->reflectivity);
                }
                else if (strcmp(inputBuf, "ns:") == 0) {
                    fscanf(inputFile, " %f", &curObject->ns);
                    hasNs = true;
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

            // TODO: Causes problems (black screen)?
            if (!hasNs)
                curObject->ns = DEFAULT_NS;
            hasNs = false;
            
            curObject->type = QUADRIC;
            objIndex++;

            // __builtin_dump_struct(curObject, &printf);
        }
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
                    curLight->theta = f_to_radians(theta);
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

    float cameraOrigin[3] = { 0, 0, 0 };

    sceneData->numObjects = objIndex;
    sceneData->numLights = lightIndex;
    sceneData->camera.origin[0] = cameraOrigin[0];
    sceneData->camera.origin[1] = cameraOrigin[1];
    sceneData->camera.origin[2] = cameraOrigin[2];
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

#ifndef NDEBUG
    printf("Highest iterationNum: %i\n", highestIteration);
#endif

    return EXIT_SUCCESS;
}
