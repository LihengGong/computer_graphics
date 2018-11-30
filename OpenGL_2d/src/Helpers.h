#ifndef SHADER_H
#define SHADER_H

#include <string>
#include <vector>
#include <Eigen/Core>

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

enum Op_mode {
    INIT_MODE,
    INSERTION_MODE,
    TRANSLATION_MODE,
    DELETE_MODE,
};

enum TRANSLATION_SUBMODE {
    INIT_SUB_MODE,
    ROTATE_CLOCKWISE_MODE,
    ROTATE_COUNTERCLOCKWISE_MODE,
    SCALE_UP_MODE,
    SCALE_DOWN_MODE
};

// struct Point {
//     double x;
//     double y;
//     bool is_valid;
//     Point(double _x=-1.0, double _y=-1.0, bool v=false) : x(_x), y(_y), is_valid(v) {}
// };

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

    // Select this VBO for subsequent draw calls
    void bind();

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

#endif
