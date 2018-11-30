#ifndef SHADER_H
#define SHADER_H

#include <iostream>
#include <unordered_map>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#define GL_SILENCE_DEPRECATION

#ifdef _WIN32
#  include <windows.h>
#  undef max
#  undef min
#  undef DrawText
#endif

#ifndef __APPLE__
#  define GLEW_STATIC
#  include <GL/glew.h>
#endif

#ifdef __APPLE__
#   include <OpenGL/gl3.h>
#   define __gl_h_ /* Prevent inclusion of the old gl.h */
#else
#   ifdef _WIN32
#       include <windows.h>
#   endif
#   include <GL/gl.h>
#endif


const double PI  =3.141592653589793238463;
const double EPSILON = 0.0000001;


enum RenderObject {
    OBJECT_CUBE,
    OBJECT_BUMPY_CUBE,
    OBJECT_BUNNY,
    OBJECT_SPHERE_BIG,
    OBJECT_SPHERE_SMALL,
    OBJECT_SPHERE_CENTER,
    OBJECT_TOTAL_NUM
};

enum TransformMode {
    MODE_DO_NOTHING,
    MODE_PAN_LEFT,
    MODE_PAN_RIGHT,
    MODE_PAN_UP,
    MODE_PAN_DOWN,
    MODE_PAN_NEAR,
    MODE_PAN_FAR,
    MODE_ROTATE_X,
    MODE_ROTATE_Y,
    MODE_ROTATE_Z,
    MODE_ROTATE_ARB,
    MODE_SCALE_UP,
    MODE_SCALE_DOWN
};

enum ViewMode {
    ORTHOGRAPHIC_VIEW,
    PERSPECTIVE_VIEW
};

enum CamDirection {
    CAM_DIR_NO_MOVE,
    CAM_DIR_UP,
    CAM_DIR_DOWN,
    CAM_DIR_LEFT,
    CAM_DIR_RIGHT
};

enum ShadingMode {
    WIRE_FRAME_SHADING,
    FLAT_SHADING,
    PHONG_SHADING
};

class VertexArrayObject
{
public:
    unsigned int id;

    VertexArrayObject() : id(0) {}

    // Create a new VAO
    void init();

    // Select this VAO for subsequent draw calls
    void bind();

    // Release the id
    void free();
};

class VertexBufferObject
{
public:
    typedef unsigned int GLuint;
    typedef int GLint;

    GLuint id;
    GLuint rows;
    GLuint cols;

    VertexBufferObject() : id(0), rows(0), cols(0) {}

    // Create a new empty VBO
    void init();

    // Updates the VBO with a matrix M
    void update(const Eigen::MatrixXf& M);

    void update(const Eigen::MatrixXi& M, GLuint buffer_type);

    // Select this VBO for subsequent draw calls
    void bind();

    void bind(GLuint buffer_type);

    // Release the id
    void free();
};

// This class wraps an OpenGL program composed of two shaders
class Program
{
public:
  typedef unsigned int GLuint;
  typedef int GLint;

  GLuint vertex_shader;
  GLuint fragment_shader;
  GLuint program_shader;

  Program() : vertex_shader(0), fragment_shader(0), program_shader(0) { }

  // Create a new shader from the specified source strings
  bool init(const std::string &vertex_shader_string,
  const std::string &fragment_shader_string,
  const std::string &fragment_data_name);

  // Select this shader for subsequent draw calls
  void bind();

  // Release all OpenGL objects
  void free();

  // Return the OpenGL handle of a named shader attribute (-1 if it does not exist)
  GLint attrib(const std::string &name) const;

  // Return the OpenGL handle of a uniform attribute (-1 if it does not exist)
  GLint uniform(const std::string &name) const;

  // Bind a per-vertex array attribute
  GLint bindVertexAttribArray(const std::string &name, VertexBufferObject& VBO) const;
  GLint bindVertexAttribArray(const std::string &name,
                              VertexBufferObject& VBO,
                              GLuint stride, const GLvoid* start_p) const;

  GLuint create_shader_helper(GLint type, const std::string &shader_string);

};

// From: https://blog.nobel-joergensen.com/2013/01/29/debugging-opengl-using-glgeterror/
void _check_gl_error(const char *file, int line);

///
/// Usage
/// [... some opengl calls]
/// glCheckError();
///
#define check_gl_error() _check_gl_error(__FILE__,__LINE__)

inline bool MolerTrumbore(const Eigen::Ref<const Eigen::Vector3d> rayOrigin,
                          const Eigen::Ref<const Eigen::Vector3d> rayVector,
                          const Eigen::Ref<const Eigen::Vector3d> vert0,
                          const Eigen::Ref<const Eigen::Vector3d> vert1,
                          const Eigen::Ref<const Eigen::Vector3d> vert2,
                          Eigen::Ref<Eigen::Vector3d> intersection,
                          double& solution_t);

class Shape {
public:
    static Program program;
    std::string name;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    static Eigen::Vector3f light_orig;

    static Eigen::MatrixXf Vertices;
    static Eigen::MatrixXf Normals;
    static Eigen::MatrixXi Indices;
    static Eigen::MatrixXf VertFlat;
    static Eigen::MatrixXf NormFlat;
    static VertexArrayObject VAO;
    static VertexBufferObject VBO;
    static VertexBufferObject VBO_NORM;
    // static VertexBufferObject VBO_I;
    static VertexBufferObject VBO_Flt;
    static VertexBufferObject VBO_NormFlt;
    static int tri_number;
    static int vert_number;
    static int total_shapes;
    static ShadingMode shading_mode;

    static const GLchar* vertex_shader;

    static const GLchar* fragment_shader;

    Shape() {}
    virtual ~Shape() {
    }

    virtual void render(const Eigen::Matrix4f& mvp)=0;
    virtual void render_new(const Eigen::Matrix4f& vp,
                            const Eigen::Matrix4f& model,
                            bool is_selected, RenderObject obj_type)=0;

    static void init() {
        VAO.init();
        VAO.bind();
        VBO.init();
        VBO_Flt.init();
        VBO_NORM.init();
        VBO_NormFlt.init();
        program.init(vertex_shader, fragment_shader, "outColor");
        program.bind();

        light_orig << 2.2, 2.2, 2.2;
        std::cout << "init done\n";
    }
};

enum NUM_INDEX {
    VERT_START_INDEX_INDEX,
    TRI_START_INDEX_INDEX,
    TRI_NUM_INDEX,
    VERT_NUM_INDEX
};

class Mesh: public Shape {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Mesh() {}
    Mesh(const std::string& filename, float scale_fac);
    bool load_mesh_data_new(const std::string& filename);
    void render(const Eigen::Matrix4f& vp) override {}
    void render_new(const Eigen::Matrix4f& vp,
                    const Eigen::Matrix4f& model,
                    bool is_selected, RenderObject obj_type) override;
    bool is_hit_new(const Eigen::Ref<const Eigen::Vector3f> origin,
                const Eigen::Ref<const Eigen::Vector3f> direction);

    void compute_normals();
    static std::unordered_map<std::string, Eigen::Vector3f> mesh_hashmap;
    static std::unordered_map<std::string, std::vector<int>> mesh_numbers;
    static int mesh_tri_number;
    static int mesh_vert_number;
    Eigen::Vector3f centroid;
    GLuint vert_start_index;
    GLuint tri_start_index;
    GLuint tri_num;
    GLuint vert_num;
};

class OglObject {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Vector3f centroid;
    Eigen::Vector3f origin;
    Eigen::Matrix4f model;
    std::string name;
    RenderObject obj_type;
    Mesh mesh;
    bool is_selected;
    TransformMode trans_mode;
    OglObject(const std::string& filename, float scale_fac, RenderObject type);
    void render(const Eigen::Matrix4f& vp);

    void translate(float x, float y, float z);
    void translate(const Eigen::Ref<const Eigen::Vector3f> new_pos);
    void move_to(const Eigen::Ref<const Eigen::Vector3f> new_pos);
    virtual void move_to(float x, float y, float z);
    void pan_up() { move_to(origin(0), origin(1) + 0.1, origin(2)); } //key w
    void pan_down() { move_to(origin(0), origin(1) - 0.1, origin(2)); } // key s
    void pan_left() { move_to(origin(0) - 0.1, origin(1), origin(2)); } // key a
    void pan_right() { move_to(origin(0) + 0.1, origin(1), origin(2)); } // key d
    void pan_near() { move_to(origin(0), origin(1), origin(2) + 0.1); } // key q
    void pan_far() { move_to(origin(0), origin(1), origin(2) - 0.1); } // key e

    void rotate_x(float angle);
    void rotate_y(float angle);
    void rotate_z(float angle);

    bool is_hit(const Eigen::Ref<const Eigen::Vector3f> origin,
                const Eigen::Ref<const Eigen::Vector3f> direction);

    void scale(float scale_x, float scale_y, float scale_z);
    void scale(float scale_factor);
    void scale_up() { scale(1.2); }
    void scale_down() { scale(0.8); }
};


class Sphere: public OglObject {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Matrix4f sphere_model;
    int sphere_color;
    Sphere(const std::string& filename, float scale_fac, RenderObject type);
    void move_to(float x, float y, float z) override;
};
#endif
