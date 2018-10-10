#ifndef UTILS_H
#define UTILS_H


#include <istream>
#include <sstream>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <sys/resource.h>
#include <Eigen/Core>
#include <algorithm>

using namespace std;
using namespace Eigen;

bool get_size(const string& filename, int& vert_number, int& tri_number);
bool read_mesh_data(const std::string& filename, Ref<MatrixXd> vert_matrix, Ref<MatrixXi> tri_matrix);

struct struct_sphere {
    explicit struct_sphere(double x=0., double y=0., double z=0., double r=0.9) {
        origin << x, y, z;
        radius = r;
    }

    Vector3d origin;
    double radius;
};



unsigned char double_2_unsignedchar(const double d);
void write_matrix_to_png(const Eigen::MatrixXd& R, const Eigen::MatrixXd& G, const Eigen::MatrixXd& B, const Eigen::MatrixXd& A, const std::string& filename);


#endif
