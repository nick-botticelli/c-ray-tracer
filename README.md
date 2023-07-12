# C Ray Tracer

![Example output image](example.png)

A very fast ray tracer in C featuring a custom `.scene` file implementation, basic PPM file
implementation, and recursive ray tracing with lighting, reflection, and three object primitives.

Details:
* `.scene` file implementation:
    * Custom file implementation
    * Basic format describing a 3D scene to render
    * [Example `.scene` file](input.scene)
* PPM implementation:
    * Portable PixMap (`.ppm`)
    * ASCII and binary formats (P3 and P6 respectively)
* Ray tracing:
    * Features recursive ray tracing function with ray casting
    * Allows for reflection, lighting, and more (e.g., refraction)
    * Very fast
        * 5.2s to render [input.scene](input.scene) at 10000x10000 on Apple MacBook Air (Late 2020)
        with Clang 16
        * OpenMP to easily utilize multithreading scalable to any system
        * Aggressive inlining to minimize function overhead
        * SIMD-friendly code

# Authors
* Nicholas Botticelli
* Aiden Halili

# Usage
Run the program with five arguments: width, height, input scene file path, and output PPM path.

Example:
`./raytrace 1000 1000 input.scene output.ppm`

# Known Issues
* Potentially imperfect reflection
* Non-working refraction
