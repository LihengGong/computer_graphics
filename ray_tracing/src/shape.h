#ifndef CG_HW1_SHAPE_H
#define CG_HW1_SHAPE_H

// #define CG_USE_TBB

#include <vector>

#ifdef CG_USE_TBB
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#endif


#ifdef CG_USE_TBB
using namespace tbb;
#endif

enum Shading{
    LAMBERTIAN_SHADING,
    BLINN_PHONG_SHADING,
    AMBIENT_SHADING,
    MIRROR_SHADING
};

enum View{
    ORTHOGRAPHIC_VIEW,
    PERSPECTIVE_VIEW,
};

enum Color {
  COLOR_RED,
  COLOR_GREEN,
  COLOR_BLUE,
  COLOR_GOLD,
  COLOR_NYU,
  COLOR_GREY,
  COLOR_BLACK
};

struct Configuration {
  Color color_enum;
  Shading shading;
  double col_value;
  explicit Configuration(Color c=COLOR_BLACK,
                Shading s=LAMBERTIAN_SHADING, double v=0.) :
  color_enum(c), shading(s), col_value(v) {}
};

const double BLACK_COLOR = 0.;

const double EPSILON = 0.0000001;
const double SHADOW_EPSILON = 0.6;

/*
Moler-Trumbore is the fastest one.
This method is adapted from the code in this thesis:
https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
*/
inline bool MolerTrumbore(const Eigen::Ref<const Eigen::Vector3d> rayOrigin,
                          const Eigen::Ref<const Eigen::Vector3d> rayVector,
                          const Eigen::Ref<const Eigen::Vector3d> vert0,
                          const Eigen::Ref<const Eigen::Vector3d> vert1,
                          const Eigen::Ref<const Eigen::Vector3d> vert2,
                          Eigen::Ref<Eigen::Vector3d> intersection,
                          double& solution_t) {

  Eigen::Vector3d edge1 = vert1 - vert0;
  Eigen::Vector3d edge2 = vert2 - vert0;

  Eigen::Vector3d pvec = rayVector.cross(edge2);
  double det = edge1.dot(pvec);

  if (det > - EPSILON && det < EPSILON) {
      return false;
  }

  double inv_det = 1 / det;
  Eigen::Vector3d tvec = rayOrigin - vert0;
  float u = (tvec.dot(pvec)) * inv_det;
  if (u < 0. || u > 1.0) {
      return false;
  }

  Eigen::Vector3d qvec = tvec.cross(edge1);
  double v = (rayVector.dot(qvec)) * inv_det;
  if (v < 0. || u + v > 1.0) {
      return false;
  }

  solution_t = (edge2.dot(qvec)) * inv_det;

  intersection = rayOrigin + rayVector * solution_t;
  return true;
}

void compute_scene();


class Shape {
public:
  Shape(Shading s, Color clr=COLOR_BLACK) {
    shading = s;
    unit_normal << 0., 0., 0.;
    color_enum = clr;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  static Eigen::MatrixXd C_R_mat;
  static Eigen::MatrixXd C_G_mat;
  static Eigen::MatrixXd C_B_mat;
  static Eigen::MatrixXd A_mat;
  static Eigen::MatrixXd BK_mat;
  static long x_len, y_len;
  static bool** pixel_is_shading_bitmap;
  static Eigen::Vector3d pixel_origin;
  static Eigen::Vector3d x_displacement;
  static Eigen::Vector3d y_displacement;
  static Eigen::Vector3d scene_ray_origin;
  static Eigen::Vector3d scene_ray_direction;
  static Eigen::Vector3d orthview_direction;
  static Eigen::Vector3d persview_origin;
  static double scene_width, scene_height;
  static double closest_pixel_in_ray;
  static double camera_z_axis;
  static double background_color;
  static Eigen::Vector3d light_position;
  static Eigen::Vector3d second_light_position;
  Eigen::Vector3d unit_normal;
  static Eigen::Vector3d surface_normal;
  static Eigen::Vector3d final_intersection;
  static int light_number;
  static View view;
  static bool is_draw_shadow;
  Color color_enum;
  Shading shading;

  static inline void generate_camera_rays(int x, int y) {
    Shape::scene_ray_origin = Shape::pixel_origin + double(x) * Shape::x_displacement +
                                double(y) * Shape::y_displacement;
    if (Shape::view == ORTHOGRAPHIC_VIEW) {
      Shape::scene_ray_direction = Shape::orthview_direction;
    } else {
      Shape::scene_ray_direction = Shape::scene_ray_origin - Shape::persview_origin;
      Shape::scene_ray_origin = Shape::persview_origin;
    }
  }

  virtual inline bool hit_slow(const Eigen::Ref<const Eigen::Vector3d> point_vec,
           const Eigen::Ref<const Eigen::Vector3d> direction,
           double& solution_t,
           Eigen::Ref<Eigen::Vector3d> unit_normal) = 0;


  virtual inline bool hit(const Eigen::Ref<const Eigen::Vector3d> point_vec,
           const Eigen::Ref<const Eigen::Vector3d> direction,
           double& solution_t) = 0;

  virtual inline bool render_shape(int x, int y) {return true;}

  virtual ~Shape() {}
};

class Sphere: public Shape {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Sphere(const Eigen::Ref<const Eigen::Vector3d> c, double r,
         Shading s=LAMBERTIAN_SHADING, Color clr=COLOR_BLACK)
  : Shape(s, clr) {
    center = c;
    radius = r;
  }

  Eigen::Vector3d center;
  double radius;

  inline bool hit(const Eigen::Ref<const Eigen::Vector3d> point_vec,
           const Eigen::Ref<const Eigen::Vector3d> direction,
           double& solution_t) override {
    Eigen::Vector3d e_minus_c = point_vec - center;
    double res1 = direction.dot(e_minus_c);
    double square_direction = direction.dot(direction);
    double determinate = res1 * res1 - square_direction *
                                       (e_minus_c.dot(e_minus_c) - radius * radius);

    if (determinate > 0) {
      double solution_t = (-res1 - sqrt(determinate)) / square_direction;
      if (solution_t > 0.0001) {
        return true;
      }
    }

    return false;
  }

  inline bool hit_slow(const Eigen::Ref<const Eigen::Vector3d> point_vec,
           const Eigen::Ref<const Eigen::Vector3d> direction,
           double& solution_t,
           Eigen::Ref<Eigen::Vector3d> unit_norm) override {

    Eigen::Vector3d e_minus_c = point_vec - center;
    double res1 = direction.dot(e_minus_c);
    double square_direction = direction.dot(direction);
    double determinate = res1 * res1 - square_direction *
                                      (e_minus_c.dot(e_minus_c) - radius * radius);

    if (determinate > 0) {
      solution_t = (-res1 - sqrt(determinate)) / square_direction;
      unit_norm = (point_vec + solution_t * direction - center) / radius;
      return true;
    }

    return false;
  }

};


class Mesh: public Shape {
public:
  Eigen::Vector3d p_vec;
  Eigen::Vector3d dir;
  explicit Mesh(const std::string& filename,
                Shading s=LAMBERTIAN_SHADING, Color clr=COLOR_BLACK)
  : Shape(s, clr), ARAT(*this) {
    load_mesh_data(filename);
    if (tri_number > 0) {
      solution_array = new double[tri_number];
      is_intersection_arry = new bool[tri_number];
      for (int ind = 0; ind < tri_number; ++ind) {
        solution_array[ind] = std::numeric_limits<double>::infinity();
        is_intersection_arry[ind] = false;
      }
    } else {
      solution_array = NULL;
    }
    p_vec << 0., 0., 0.;
    dir << 0., 0., 0.;
  }

  virtual ~Mesh() override {
    delete[] solution_array;
    solution_array = NULL;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Eigen::MatrixXd vertices_matrix;
  Eigen::MatrixXi triangles_matrix;
  double *solution_array;
  bool *is_intersection_arry;
  long vert_number, tri_number;
  void load_mesh_data(const std::string& filename);

  class ApplyRenderAllTriangles {
    Mesh& mesh;
  public:
    ApplyRenderAllTriangles(Mesh& m) : mesh(m){}
#ifdef CG_USE_TBB
    void operator() (const blocked_range<size_t>& r) const {
      for (size_t ind = r.begin(); ind != r.end(); ++ind) {
        //////do the job.
        Eigen::Vector3i cur_triangle_vertices = mesh.triangles_matrix.row(ind);
        Eigen::Vector3d vertice0 = mesh.vertices_matrix.row(cur_triangle_vertices(0));
        Eigen::Vector3d vertice1 = mesh.vertices_matrix.row(cur_triangle_vertices(1));
        Eigen::Vector3d vertice2 = mesh.vertices_matrix.row(cur_triangle_vertices(2));

        double cur_solution = std::numeric_limits<double>::infinity();
        Eigen::Vector3d intersection(0., 0., 0.);

        bool is_intersect = MolerTrumbore(mesh.p_vec,
                                          mesh.dir,
                                          vertice0, vertice1, vertice2,
                                          intersection, cur_solution);
        mesh.is_intersection_arry[ind] = is_intersect;
        mesh.solution_array[ind] = cur_solution;
      }
    }
#endif
  }ARAT;


  inline bool hit_slow(const Eigen::Ref<const Eigen::Vector3d> point_vec,
           const Eigen::Ref<const Eigen::Vector3d> direction,
           double& solution_t,
           Eigen::Ref<Eigen::Vector3d> unit_norm) override {

    Eigen::Vector3d intersection(0., 0., 0.);
    bool is_intersect= false;
    bool is_shading = false;
    double temp_solution = std::numeric_limits<double>::infinity();
    int final_index = -1;

    p_vec = point_vec;
    dir = direction;

#ifdef CG_USE_TBB
    parallel_for(blocked_range<size_t>(0, tri_number),
                 ApplyRenderAllTriangles(*this));

    for (unsigned int ind = 0; ind < tri_number; ind++) {
      if (is_intersection_arry[ind] &&
            solution_array[ind] < temp_solution) {

          temp_solution = solution_array[ind];
          final_index = ind;
          is_shading = true;
      }
    }
#else
    for (unsigned int ind = 0; ind < tri_number; ind++) {
      Eigen::Vector3i cur_triangle_vertices = triangles_matrix.row(ind);
      Eigen::Vector3d vertice0 = vertices_matrix.row(cur_triangle_vertices(0));
      Eigen::Vector3d vertice1 = vertices_matrix.row(cur_triangle_vertices(1));
      Eigen::Vector3d vertice2 = vertices_matrix.row(cur_triangle_vertices(2));

      double cur_solution;

      is_intersect = MolerTrumbore(point_vec,
                                        direction,
                                        vertice0, vertice1, vertice2,
                                        intersection, cur_solution);

      if (is_intersect && cur_solution < temp_solution) {
        temp_solution = cur_solution;
        if (temp_solution < 0) {
          std::cout << "waning! negative solution" << std::endl;
        }
        final_index = ind;
        is_shading = true;
      }
    }
#endif

    if (is_shading) {
      Eigen::Vector3i cur_triangle_vertices = triangles_matrix.row(final_index);
      Eigen::Vector3d vertice0 = vertices_matrix.row(cur_triangle_vertices(0));
      Eigen::Vector3d vertice1 = vertices_matrix.row(cur_triangle_vertices(1));
      Eigen::Vector3d vertice2 = vertices_matrix.row(cur_triangle_vertices(2));
      unit_norm = ((vertice1-vertice0).cross(vertice2-vertice0));
      unit_norm = unit_norm.normalized();
      //if (unit_normal(2) < 0) {
      if(unit_norm.dot(direction) > 0) {
        unit_norm = - unit_norm;
      }
      solution_t = temp_solution;
      return true;
    }

    return false;
  }

  inline bool render_shape(int x, int y) override {
    return render_all_triangles(x, y);
  }

  inline bool hit(const Eigen::Ref<const Eigen::Vector3d> point_vec,
           const Eigen::Ref<const Eigen::Vector3d> direction,
           double& nearest_solution_t) override {

    Eigen::Vector3d intersection(0., 0., 0.);
    for (unsigned t = 0; t < tri_number; t++) {
      Eigen::Vector3i cur_triangle_vertices = triangles_matrix.row(t);
      Eigen::Vector3d vertice0 = vertices_matrix.row(cur_triangle_vertices(0));
      Eigen::Vector3d vertice1 = vertices_matrix.row(cur_triangle_vertices(1));
      Eigen::Vector3d vertice2 = vertices_matrix.row(cur_triangle_vertices(2));

      double solution_t;

      bool is_intersect = MolerTrumbore(point_vec,
                                         direction,
                                         vertice0, vertice1, vertice2,
                                         intersection, solution_t);
      if (is_intersect) {
        if (solution_t > EPSILON) {
          return true;
        } else if (solution_t < 0) {
          // cout << "Found negative solution!!!" << endl;
        }
      }
    }

    return false;
  }

  inline bool render_all_triangles(int x, int y) {
    return true;
  }
};


const double PLANE_RATIO = -3.15;
class Plane: public Shape {
public:
  Eigen::Vector3d plane_point;
  Eigen::Vector3d plane_normal;
  Plane(const Eigen::Ref<const Eigen::Vector3d> p,
        const Eigen::Ref<const Eigen::Vector3d> n,
        Shading s=LAMBERTIAN_SHADING, Color clr=COLOR_GREY)
    : Shape(s, clr), plane_normal(n), plane_point(p) {}
  inline bool hit_slow(const Eigen::Ref<const Eigen::Vector3d> point_vec,
             const Eigen::Ref<const Eigen::Vector3d> direction,
             double& solution_t,
             Eigen::Ref<Eigen::Vector3d> unit_normal) override {
    // if (mirror_normal.dot(direction)>0.00001 ||
    //      mirror_normal.dot(direction) < -0.00001) {
    //  double d = ((mirror_point - point_vect).dot(mirror_normal)) /
    //             (mirror_normal.dot(direction));
    //  Vector3d mirror_intersection = point_vect + d * direction;
    //  color += compute_raycolor(mirror_intersection, vector_r, depth + 1);
    //  return color;
    // }
    ///// If the dot product of plane normal and ray direction is 0,
    ///// ray is parallel to plane and there is no intersection.
    double dot_product_res = plane_normal.dot(direction);
    // if (dot_product_res > EPSILON || dot_product_res < -EPSILON) {
    // if (dot_product_res < -EPSILON ) {
    if (dot_product_res + direction(1) < -EPSILON) {
      double val_d = (plane_point - point_vec).dot(plane_normal) / dot_product_res;
      solution_t = val_d;
      //// val_d * direction + point_vec gives the intersection
      unit_normal = plane_normal;

      return true;
    }

    return false;
  }


  inline bool hit(const Eigen::Ref<const Eigen::Vector3d> point_vec,
             const Eigen::Ref<const Eigen::Vector3d> direction,
             double& solution_t) override {
               double dot_product_res = plane_normal.dot(direction);
    //if (dot_product_res > EPSILON || dot_product_res < -EPSILON) {
    // if (dot_product_res < -EPSILON ) {
    if (dot_product_res + direction(1) < -EPSILON) {
      double val_d = (plane_point - point_vec).dot(plane_normal) / dot_product_res;
      solution_t = val_d;
      //// val_d * direction + point_vec gives the intersection
      //unit_normal = plane_normal;
      return true;
    }

    return false;
             }
};

#endif
