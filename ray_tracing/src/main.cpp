// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <sys/resource.h>
#include <Eigen/Dense>
#include <time.h>

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"
#include "shape.h"
//#include <tbb/task_scheduler_init.h>

// Shortcut to avoid Eigen:: and std:: everywhere, DO NOT USE IN .h
using namespace std;
using namespace Eigen;

vector<Shape*> shape_vectors;


void extend_stack() {
    const rlim_t kStackSize = 16 * 1024 * 1024;   // min stack size = 16 MB
    struct rlimit rl;
    int result;

    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0)
    {
        if (rl.rlim_cur < kStackSize)
        {
            rl.rlim_cur = kStackSize;
            result = setrlimit(RLIMIT_STACK, &rl);
            if (result != 0)
            {
                fprintf(stderr, "setrlimit returned result = %d\n", result);
            }
        }
    } else {
        fprintf(stderr, "getrlimit returned result = %d\n", result);
    }
}


int main()
{
    clock_t tStart = clock();
    // The default stack limit is 8M; extend it to 16M just in case.
    extend_stack();

    string filename("../data/bumpy_cube.off");
    string filename2("../data/bunny.off");

    Mesh *m = new Mesh(filename, LAMBERTIAN_SHADING, COLOR_GREEN);
    shape_vectors.push_back(m);

    Mesh *m2 = new Mesh(filename2, LAMBERTIAN_SHADING, COLOR_GOLD);
    shape_vectors.push_back(m2);

    // Vector3d center(-3.0, 1.0, 1.2);
    // Vector3d center2(-2.8, -1.6, -1.0);
    Vector3d center(-5.2, 3.3, -0.6);
    Vector3d center2(-4.6, 4.9, -0.3);//4.6
    Vector3d center3(8.0, 1.0, 2.5);
    // Vector3d center4(2.0, -2.0, 3.0);
    // Vector3d center5(-2.5, -3.2, 2.0);
    Sphere *s = new Sphere(center, 0.8, BLINN_PHONG_SHADING, COLOR_BLUE);
    Sphere *s2 = new Sphere(center2, 0.8, BLINN_PHONG_SHADING, COLOR_NYU);
    Sphere *s3 = new Sphere(center3, 1.2, BLINN_PHONG_SHADING, COLOR_NYU);
    // Sphere *s4 = new Sphere(center4, 0.7);
    // Sphere *s5 = new Sphere(center5, 0.3, BLINN_PHONG_SHADING);
    shape_vectors.push_back(s);
    shape_vectors.push_back(s2);
    shape_vectors.push_back(s3);
    // shape_vectors.push_back(s4);
    // shape_vectors.push_back(s5);

    // Vector3d center11(-1.8, 0.5, 0.9);
    // Vector3d center12(1.8, 1.6, 2.0);
    // Vector3d center13(-1.0, 1.0, 0.);
    // Vector3d center14(2.0, -3.0, 1.0);
    // Vector3d center15(-3.0, 0., 1.0);
    // Sphere *s11 = new Sphere(center11, 0.3);
    // Sphere *s12 = new Sphere(center12, 0.3, BLINN_PHONG_SHADING);
    // Sphere *s13 = new Sphere(center13, 0.5);
    // Sphere *s14 = new Sphere(center14, 0.3);
    // Sphere *s15 = new Sphere(center15, 0.6, BLINN_PHONG_SHADING);
    // shape_vectors.push_back(s11);
    // shape_vectors.push_back(s12);
    // shape_vectors.push_back(s13);
    // // shape_vectors.push_back(s14);
    // shape_vectors.push_back(s15);

    Vector3d plane_p(0., -3.0, 0.);
    Vector3d plane_norm(0., 1.0, 0.);
    Plane *pln = new Plane(plane_p, plane_norm, LAMBERTIAN_SHADING, COLOR_GREY);
    // shape_vectors.push_back(pln);

    compute_scene();

    for (vector<Shape*>::iterator it = shape_vectors.begin();
              it != shape_vectors.end(); ++it) {
      delete *it;
    }

#ifdef CG_USE_TBB
    int mib[4];
    int numCPU;
    std::size_t len = sizeof(numCPU);

    /* set the mib for hw.ncpu */
    mib[0] = CTL_HW;
    mib[1] = HW_AVAILCPU;  // alternatively, try HW_NCPU;

    /* get the number of CPUs from the system */
    sysctl(mib, 2, &numCPU, &len, NULL, 0);

    if (numCPU < 1)
    {
        mib[1] = HW_NCPU;
        sysctl(mib, 2, &numCPU, &len, NULL, 0);
        if (numCPU < 1)
            numCPU = 1;
    }
    cout << "program execution time: "
         << ((double)(clock() - tStart)/(CLOCKS_PER_SEC*numCPU))
         << "s."<< endl;
#else
    cout << "program execution time: "
         << ((double)(clock() - tStart)/CLOCKS_PER_SEC)
         << "s."<< endl;
#endif
    return 0;
}
