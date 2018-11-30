#include "Helpers.h"

#include <iostream>
#include <fstream>
#include <exception>
#include <system_error>

void VertexArrayObject::init()
{
  glGenVertexArrays(1, &id);
  check_gl_error();
}

void VertexArrayObject::bind()
{
  glBindVertexArray(id);
  check_gl_error();
}

void VertexArrayObject::free()
{
  glDeleteVertexArrays(1, &id);
  check_gl_error();
}

void VertexBufferObject::init()
{
  glGenBuffers(1,&id);
  check_gl_error();
}

void VertexBufferObject::bind()
{
  glBindBuffer(GL_ARRAY_BUFFER,id);
  check_gl_error();
}

void VertexBufferObject::bind(GLuint buffer_type) {
  glBindBuffer(buffer_type, id);
  check_gl_error();
}

void VertexBufferObject::free()
{
  glDeleteBuffers(1,&id);
  check_gl_error();
}

void VertexBufferObject::update(const Eigen::MatrixXf& M)
{
  assert(id != 0);
  glBindBuffer(GL_ARRAY_BUFFER, id);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float)*M.size(), M.data(), GL_DYNAMIC_DRAW);
  rows = M.rows();
  cols = M.cols();
  check_gl_error();
}

void VertexBufferObject::update(const Eigen::MatrixXi& M, GLuint buffer_type) {
  assert(id != 0);
  glBindBuffer(buffer_type, id);
  glBufferData(buffer_type, sizeof(GLuint)*M.size(), M.data(), GL_DYNAMIC_DRAW);
  rows = M.rows();
  cols = M.cols();
  check_gl_error();
}

bool Program::init(
  const std::string &vertex_shader_string,
  const std::string &fragment_shader_string,
  const std::string &fragment_data_name)
{
  using namespace std;
  vertex_shader = create_shader_helper(GL_VERTEX_SHADER, vertex_shader_string);
  fragment_shader = create_shader_helper(GL_FRAGMENT_SHADER, fragment_shader_string);

  if (!vertex_shader || !fragment_shader)
    return false;

  program_shader = glCreateProgram();

  glAttachShader(program_shader, vertex_shader);
  glAttachShader(program_shader, fragment_shader);

  glBindFragDataLocation(program_shader, 0, fragment_data_name.c_str());
  glLinkProgram(program_shader);

  GLint status;
  glGetProgramiv(program_shader, GL_LINK_STATUS, &status);

  if (status != GL_TRUE)
  {
    char buffer[512];
    glGetProgramInfoLog(program_shader, 512, NULL, buffer);
    cerr << "Linker error: " << endl << buffer << endl;
    program_shader = 0;
    return false;
  }

  check_gl_error();
  return true;
}

void Program::bind()
{
  glUseProgram(program_shader);
  check_gl_error();
}

GLint Program::attrib(const std::string &name) const
{
  return glGetAttribLocation(program_shader, name.c_str());
}

GLint Program::uniform(const std::string &name) const
{
  return glGetUniformLocation(program_shader, name.c_str());
}

GLint Program::bindVertexAttribArray(const std::string &name,
              VertexBufferObject& VBO,
              GLuint stride, const GLvoid* start_p) const {
  GLint id = attrib(name);
  if (id < 0)
    return id;
  if (VBO.id == 0)
  {
    glDisableVertexAttribArray(id);
    return id;
  }
  VBO.bind();
  glEnableVertexAttribArray(id);
  glVertexAttribPointer(id, 3, GL_FLOAT, GL_FALSE, stride, start_p);
  check_gl_error();

  return id;
}

GLint Program::bindVertexAttribArray(
        const std::string &name, VertexBufferObject& VBO) const
{
  return bindVertexAttribArray(name, VBO, 0, 0);
}

void Program::free()
{
  if (program_shader)
  {
    glDeleteProgram(program_shader);
    program_shader = 0;
  }
  if (vertex_shader)
  {
    glDeleteShader(vertex_shader);
    vertex_shader = 0;
  }
  if (fragment_shader)
  {
    glDeleteShader(fragment_shader);
    fragment_shader = 0;
  }
  check_gl_error();
}

GLuint Program::create_shader_helper(GLint type, const std::string &shader_string)
{
  using namespace std;
  if (shader_string.empty())
    return (GLuint) 0;

  GLuint id = glCreateShader(type);
  const char *shader_string_const = shader_string.c_str();
  glShaderSource(id, 1, &shader_string_const, NULL);
  glCompileShader(id);

  GLint status;
  glGetShaderiv(id, GL_COMPILE_STATUS, &status);

  if (status != GL_TRUE)
  {
    char buffer[512];
    if (type == GL_VERTEX_SHADER)
      cerr << "Vertex shader:" << endl;
    else if (type == GL_FRAGMENT_SHADER)
      cerr << "Fragment shader:" << endl;
    else if (type == GL_GEOMETRY_SHADER)
      cerr << "Geometry shader:" << endl;
    cerr << shader_string << endl << endl;
    glGetShaderInfoLog(id, 512, NULL, buffer);
    cerr << "Error: " << endl << buffer << endl;
    return (GLuint) 0;
  }
  check_gl_error();

  return id;
}

void _check_gl_error(const char *file, int line)
{
  GLenum err (glGetError());

  while(err!=GL_NO_ERROR)
  {
    std::string error;

    switch(err)
    {
      case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
      case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
      case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
      case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
      case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
    }

    std::cerr << "GL_" << error.c_str() << " - " << file << ":" << line << std::endl;
    err = glGetError();
  }
}


const GLuint TOTAL_TRIANGLES = 80000;
// const GLuint TOTAL_TRIANGLES = 2000;
Program Shape::program;
Eigen::MatrixXf Shape::Vertices = Eigen::MatrixXf::Zero(3, TOTAL_TRIANGLES * 3);
Eigen::MatrixXi Shape::Indices = Eigen::MatrixXi::Zero(3, TOTAL_TRIANGLES * 3);
Eigen::MatrixXf Shape::VertFlat = Eigen::MatrixXf::Zero(3, TOTAL_TRIANGLES * 3);
Eigen::MatrixXf Shape::Normals = Eigen::MatrixXf::Zero(3, TOTAL_TRIANGLES * 3);
Eigen::MatrixXf Shape::NormFlat = Eigen::MatrixXf::Zero(3, TOTAL_TRIANGLES * 3);

// Eigen::Matrix4f sphere_model = Eigen::Matrix4f::Identity();

extern float cam_x, cam_y, cam_z;

Eigen::Vector3f Shape::light_orig;

VertexArrayObject Shape::VAO;
VertexBufferObject Shape::VBO;
VertexBufferObject Shape::VBO_NORM;
VertexBufferObject Shape::VBO_NormFlt;

std::unordered_map<std::string, Eigen::Vector3f> Mesh::mesh_hashmap;
int Mesh::mesh_tri_number = 0;
int Mesh::mesh_vert_number = 0;
std::unordered_map<std::string, std::vector<int>> Mesh::mesh_numbers;
// VertexBufferObject Shape::VBO_I;
VertexBufferObject Shape::VBO_Flt;
int Shape::tri_number = 0;
int Shape::vert_number = 0;
int Shape::total_shapes = 0;
ShadingMode Shape::shading_mode = FLAT_SHADING;

const GLchar* Shape::vertex_shader =
            "#version 150 core\n"
                    "in vec3 flat_position;\n"
                    "in vec3 flat_unit_normal;\n"
                    "in vec3 unit_normal;\n"
                    "out vec3 frag_pos;\n"
                    "out vec3 UNormal;\n"
                    "out mat4 vmmat;"
                    "uniform mat4 view;\n"
                    "uniform mat4 model;\n"
                    "uniform float is_flat;\n"
                    "uniform mat4 sphere_model;\n"
                    "void main()\n"
                    "{\n"
                    "    mat3 normMat = transpose(inverse(mat3(model)));"
                    "    vmmat = view * model;"
                    "    if(is_flat < 0)        {\n"
                    "        frag_pos = flat_position;\n"
                    "        UNormal = normalize(normMat * flat_unit_normal);\n"
                    "        gl_Position = vmmat * vec4(flat_position, 1.0);\n"
                    "    } else if (is_flat < 500){             \n"
                    "        frag_pos = flat_position;\n"
                    "        UNormal = normalize(normMat * unit_normal);\n"
                    "        gl_Position = vmmat * vec4(flat_position, 1.0);\n"
                    "    } else {\n"
                    "        frag_pos = flat_position;\n"
                    "        UNormal = unit_normal;\n"
                    "        gl_Position = sphere_model * vec4(flat_position, 1.0);\n"  
                    "    }\n"
                    "}\n";


const GLchar* Shape::fragment_shader =
            "#version 150 core\n"
                    "in vec3 UNormal;"
                    "in vec3 frag_pos;"
                    "in mat4 vmmat;"
                    "out vec4 outColor;"
                    "uniform vec3 light_pos;"
                    "uniform vec3 obj_color;"
                    "uniform float alpha;"
                    "void main()"
                    "{"
                    "    float ambient = 0.08;"
                    "    vec4 light_pos_1 = vmmat * vec4(light_pos.x, light_pos.y, light_pos.z, 1.0);"
                    "    vec3 light_dir = normalize(light_pos_1.xyz - vec3(vmmat * vec4(frag_pos, 1.0)));"
                    "    float shading_val = clamp(abs(dot(light_dir, UNormal)), 1, 0);"
                    "    vec3 f_color = (shading_val + ambient) * (obj_color);"
                    "    outColor.rgb = f_color;"
                    "    if (alpha < 0) {"
                    "        outColor.a = 0.1;"
                    "    } else {"
                    "        outColor.a = 1.0;"
                    "    }"
                    "}";

/*
Moler-Trumbore is the fastest one.
This method is adapted from the code in this thesis:
https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
*/
inline bool MolerTrumbore(const Eigen::Ref<const Eigen::Vector3f> rayOrigin,
                          const Eigen::Ref<const Eigen::Vector3f> rayVector,
                          const Eigen::Ref<const Eigen::Vector3f> vert0,
                          const Eigen::Ref<const Eigen::Vector3f> vert1,
                          const Eigen::Ref<const Eigen::Vector3f> vert2) {//,
                          // Eigen::Ref<Eigen::Vector3d> intersection,
                          // double& solution_t) {

  Eigen::Vector3f edge1 = vert1 - vert0;
  Eigen::Vector3f edge2 = vert2 - vert0;

  Eigen::Vector3f pvec = rayVector.cross(edge2);
  float det = edge1.dot(pvec);

  if (det > - EPSILON && det < EPSILON) {
      return false;
  }

  float inv_det = 1 / det;
  Eigen::Vector3f tvec = rayOrigin - vert0;
  float u = (tvec.dot(pvec)) * inv_det;
  if (u < 0. || u > 1.0) {
      return false;
  }

  Eigen::Vector3f qvec = tvec.cross(edge1);
  float v = (rayVector.dot(qvec)) * inv_det;
  if (v < 0. || u + v > 1.0) {
      return false;
  }

  // solution_t = (edge2.dot(qvec)) * inv_det;

  // intersection = rayOrigin + rayVector * solution_t;
  return true;
}


Mesh::Mesh(const std::string& filename, float scale_fac) {
  name = filename;
  centroid = Eigen::Vector3f(0., 0., 0.);

  if (!load_mesh_data_new(filename)) {
    centroid = mesh_hashmap[filename];
    vert_start_index = mesh_numbers[filename][VERT_START_INDEX_INDEX];
    tri_start_index = mesh_numbers[filename][TRI_START_INDEX_INDEX];
    tri_num = mesh_numbers[filename][TRI_NUM_INDEX];
    vert_num = mesh_numbers[filename][VERT_NUM_INDEX];
    return;
  }

  std::cout << "new mesh " << name << " loaded. Start to update VBOs" << std::endl;
  VBO.update(VertFlat);
  VBO_NORM.update(Normals);

  VBO_Flt.update(VertFlat);
  VBO_NormFlt.update(NormFlat);

  program.bindVertexAttribArray("unit_normal", VBO_NORM,
              3 * sizeof(GLfloat), (void*)0);
  program.bindVertexAttribArray("flat_position", VBO_Flt,
              3 * sizeof(GLfloat), (void*)0);
  program.bindVertexAttribArray("flat_unit_normal", VBO_NormFlt,
              3 * sizeof(GLfloat), (void*)0);
  // origin = centroid;
  // move_to(0., 0., 0.);
  // scale(scale_fac);
  // std::cout << "in Mesh(): origin is now: " << centroid.transpose() << std::endl;
}

void Mesh::render_new(const Eigen::Matrix4f& vp,
                      const Eigen::Matrix4f& model,
                      bool is_selected, RenderObject obj_type) {
  if (obj_type == OBJECT_SPHERE_BIG) {
    // sphere_model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f scl = Eigen::Matrix4f::Identity();
    scl(0, 0) = 0.2;
    scl(1, 1) = 0.2;
    scl(2, 2) = 0.2;
    trans.col(3) << -0.72, -0.72, 0., 1.0;
    Eigen::Matrix4f sphere_model = trans * scl * model;
    glUniform3f(program.uniform("light_pos"), light_orig(0), light_orig(1), light_orig(2));
    glUniformMatrix4fv(Shape::program.uniform("sphere_model"), 1,
                                    GL_FALSE, sphere_model.data());
    glUniform3f(program.uniform("obj_color"), (87.0/255.0), (6.0/255.0), (140.0/255.0));
    // glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glUniform1f(program.uniform("is_flat"), 800.0);
    glUniform1f(program.uniform("alpha"), -100.0);
    glDrawArrays(GL_TRIANGLES, 3 * (tri_start_index), 3 * tri_num);
  } else if (obj_type == OBJECT_SPHERE_SMALL){
    // sphere_model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f scl = Eigen::Matrix4f::Identity();
    scl(0, 0) = 0.09;
    scl(1, 1) = 0.09;
    scl(2, 2) = 0.09;
    // trans.col(3) << -0.32, -0.32, 0., 1.0;
    trans.col(3) << cam_x, cam_y, cam_z, 1.0;
    // std::cout << "cam: x=" << cam_x << ", y=" << cam_y << ", z=" << cam_z << "\n";
    Eigen::Matrix4f sphere_model = trans * scl * model;
    glUniform3f(program.uniform("light_pos"), light_orig(0), light_orig(1), light_orig(2));
    glUniformMatrix4fv(Shape::program.uniform("sphere_model"), 1,
                                    GL_FALSE, sphere_model.data());
    glUniform3f(program.uniform("obj_color"), (255.0/255.0), (99.0/255.0), (71.0/255.0));
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glUniform1f(program.uniform("is_flat"), 800.0);
    glUniform1f(program.uniform("alpha"), 100.0);
    glDrawArrays(GL_TRIANGLES, 3 * (tri_start_index), 3 * tri_num);
  } else if (obj_type == OBJECT_SPHERE_CENTER){
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f scl = Eigen::Matrix4f::Identity();
    scl(0, 0) = 0.08;
    scl(1, 1) = 0.08;
    scl(2, 2) = 0.08;
    // trans.col(3) << -0.32, -0.32, 0., 1.0;
    trans.col(3) << -0.667, -0.667, 0., 1.0;
    // std::cout << "cam: x=" << cam_x << ", y=" << cam_y << ", z=" << cam_z << "\n";
    Eigen::Matrix4f sphere_model = trans * scl * model;
    glUniform3f(program.uniform("light_pos"), light_orig(0), light_orig(1), light_orig(2));
    glUniformMatrix4fv(Shape::program.uniform("sphere_model"), 1,
                                    GL_FALSE, sphere_model.data());
    glUniform3f(program.uniform("obj_color"), (0.0/255.0), (250.0/255.0), (154.0/255.0));
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glUniform1f(program.uniform("is_flat"), 800.0);
    glUniform1f(program.uniform("alpha"), 100.0);
    glDrawArrays(GL_TRIANGLES, 3 * (tri_start_index), 3 * tri_num);
  }
  else {
    glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.data());
    glUniform3f(program.uniform("light_pos"), light_orig(0), light_orig(1), light_orig(2));
    glUniform1f(program.uniform("alpha"), 100.0);
    if (is_selected) {
      glUniform3f(program.uniform("obj_color"), 1.0 - 1.0, 1.0 - 0.8431, 1.0 - 0.);
    } else {
      glUniform3f(program.uniform("obj_color"), 1.0, 0.8431, 0.);
      // glUniform3f(program.uniform("obj_color"), (87.0/255.0), (6.0/255.0), (140.0/255.0));
    }

    switch (shading_mode) {
      case WIRE_FRAME_SHADING:
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glUniform1f(program.uniform("is_flat"), 100.0);
        glDrawArrays(GL_TRIANGLES, 3 * (tri_start_index), 3 * tri_num);
        break;
      case PHONG_SHADING:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glUniform1f(program.uniform("is_flat"), 100.0);
        glDrawArrays(GL_TRIANGLES, 3 * (tri_start_index ), 3 * tri_num);
        break;
      case FLAT_SHADING:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glUniform1f(program.uniform("is_flat"), -100.0);
        glDrawArrays(GL_TRIANGLES, 3 * (tri_start_index), 3 * tri_num);
        break;
    }
  }
}

bool Mesh::load_mesh_data_new(const std::string& filename) {
  // If the mesh file has already been loaded, then no need to load again.
  if (mesh_hashmap.find(filename) != mesh_hashmap.end()) {
    std::cout << "Mesh already loaded, return directly\n";
    return false;
  }

  std::cout << "Loading mesh " << filename << "\n";

  mesh_hashmap[filename] = Eigen::Vector3f(0., 0., 0.);
  mesh_numbers[filename] = std::vector<int>(VERT_NUM_INDEX + 1, 0);
  vert_start_index = mesh_vert_number;
  tri_start_index = mesh_tri_number;
  mesh_numbers[filename][VERT_START_INDEX_INDEX] = vert_start_index;
  mesh_numbers[filename][TRI_START_INDEX_INDEX] = tri_start_index;

  std::string line;
  std::ifstream infile;
  int cur_vert_num, cur_tri_num;

  infile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  try {
    infile.open(filename);
  } catch (std::system_error &e) {
    std::clog << e.what() << ", " << strerror(errno) << std::endl;
  }

  const int VERTICES_EACH_SHAPE = 3;
  const int NUM_AXIS = 3;

  if (! infile.eof()) {
      std::getline(infile, line);
      if (line.substr(0,3) != "OFF")
      {
          std::cout << "line is:" << line << std::endl;
          fprintf(stderr, "wrong file format.\n");
          return false;
      }
  }

  if (! infile.eof()) {
      std::getline(infile, line);
      std::stringstream stream(line);
      if (! stream.eof()) {
          stream >> cur_vert_num;
      } else {
          return false;
      }
      if (! stream.eof()) {
          stream >> cur_tri_num;
      } else {
          return false;
      }
      if (! stream.eof()) {
          int temp;
          stream >> temp;
      } else {
          return false;
      }
  }

  vert_num = cur_vert_num;
  mesh_numbers[filename][VERT_NUM_INDEX] = cur_vert_num;
  tri_num = cur_tri_num;
  mesh_numbers[filename][TRI_NUM_INDEX] = cur_tri_num;

  for (int line_number = mesh_vert_number; line_number < mesh_vert_number + cur_vert_num; line_number++) {
      getline(infile, line);
      std::stringstream stream(line);
      for (int index = 0; index < NUM_AXIS; index++) {
          stream >> Vertices(index, line_number);
      }
  }

  for (int line_number = mesh_tri_number; line_number < mesh_tri_number + cur_tri_num; line_number++) {
      getline(infile, line);
      std::stringstream stream(line);
      int temp;
      stream >> temp;
      for (int index = 0; index < VERTICES_EACH_SHAPE; index++) {
          stream >> Indices(index, line_number);
      }
  }

  centroid = Eigen::Vector3f(0., 0., 0.);
  compute_normals();

  mesh_vert_number += cur_vert_num;
  mesh_tri_number += cur_tri_num;
  return true;
}

// #define SIMPLE_CENTROID
//// Methods for computing centroid
//// https://stackoverflow.com/questions/9325303/centroid-of-convex-polyhedron
//// https://github.com/libigl/libigl/blob/master/include/igl/centroid.cpp
//// http://wwwf.imperial.ac.uk/~rn/centroid.pdf

void Mesh::compute_normals() {
  // compute normals and centroid
  // light_orig << 2.2, 2.2, 2.2;
  // std::cout << "tri_number=" << tri_number << ", vert_number=" << vert_number << "\n";

  Eigen::MatrixXf cur_normals = Eigen::MatrixXf::Zero(3, 3 * tri_num);

  double volume = 0.;
  for (int ind = tri_start_index; ind < tri_start_index + tri_num; ++ind) {
    Eigen::Vector3i cur_inds = Indices.col(ind);
    Eigen::Vector3f vert0 = Vertices.col(cur_inds(0) + mesh_vert_number);
    Eigen::Vector3f vert1 = Vertices.col(cur_inds(1) + mesh_vert_number);
    Eigen::Vector3f vert2 = Vertices.col(cur_inds(2) + mesh_vert_number);
    int cur_ind_flat = ind * 3 + 0;
    VertFlat.col(cur_ind_flat + 0) = vert0;
    VertFlat.col(cur_ind_flat + 1) = vert1;
    VertFlat.col(cur_ind_flat + 2) = vert2;
    Eigen::Vector3f edge1 = vert0 - vert1;
    Eigen::Vector3f edge2 = vert1 - vert2;
    Eigen::Vector3f normal = edge1.cross(edge2);
    volume += normal.dot(vert0) / 6.0;
#ifdef SIMPLE_CENTROID
    centroid += (vert0 + vert1 + vert2);
#else
    centroid.array() += normal.array() * ((vert0 + vert1).array().square() +
                        (vert1 + vert2).array().square() +
                        (vert0 + vert2).array().square());
#endif
    Eigen::Vector3f unit_normal = normal.normalized();
    NormFlat.col(cur_ind_flat + 0) = unit_normal;
    NormFlat.col(cur_ind_flat + 1) = unit_normal;
    NormFlat.col(cur_ind_flat + 2) = unit_normal;
    cur_normals.col(cur_inds(0)) += unit_normal;
    cur_normals.col(cur_inds(1)) += unit_normal;
    cur_normals.col(cur_inds(2)) += unit_normal;
  }

  // Compute centroid
#ifdef SIMPLE_CENTROID
  centroid /= (cur_tri_num * 6.0);
#else
  centroid *= 1.0 / (24.0 * 2.0 * volume);
#endif

  std::cout << "after load data, centroid=" << centroid.transpose() << std::endl;
  mesh_hashmap[name] = centroid;

  // Compute avarage normal for Phone shading
  for (int ind = mesh_tri_number; ind < mesh_tri_number + tri_num; ++ind) {
    Eigen::Vector3i cur_inds = Indices.col(ind);
    int cur_ind_flat = ind * 3 + 0;
    Normals.col(cur_ind_flat + 0) = cur_normals.col(cur_inds(0)).normalized();
    Normals.col(cur_ind_flat + 1) = cur_normals.col(cur_inds(1)).normalized();
    Normals.col(cur_ind_flat + 2) = cur_normals.col(cur_inds(2)).normalized();
  }
}

bool Mesh::is_hit_new(const Eigen::Ref<const Eigen::Vector3f> origin,
                const Eigen::Ref<const Eigen::Vector3f> direction) {
  Eigen::Vector3f normal_dir = direction.normalized();
  std::cout << name << ", cur_tri_num=" << mesh_tri_number << "\n";
  std::cout << name << ", cur_vert_num=" << mesh_vert_number << "\n";
  for (GLuint ind = tri_start_index; ind < tri_start_index + tri_num; ++ind) {
    Eigen::Vector3i cur_triangle_vertices = Indices.col(ind);
    int cur_ind_flat = ind * 3 + 0;

    Eigen::Vector3f vertice0 = VertFlat.col(cur_ind_flat + 0);
    Eigen::Vector3f vertice1 = VertFlat.col(cur_ind_flat + 1);
    Eigen::Vector3f vertice2 = VertFlat.col(cur_ind_flat + 2);

    bool is_intersect = MolerTrumbore(origin, normal_dir,
                                      vertice0, vertice1, vertice2);
    if (is_intersect) {
      return true;
    }
  }

  return false;
}

OglObject::OglObject(const std::string& filename, float scale_fac, RenderObject type) {
  mesh = Mesh(filename, scale_fac);
  is_selected = false;
  model = Eigen::Matrix4f::Identity();

  trans_mode = MODE_DO_NOTHING;
  obj_type = type;
  centroid << mesh.centroid(0), mesh.centroid(1), mesh.centroid(2);
  origin = Eigen::Vector3f(0., 0., 0.);
  origin = centroid;
  move_to(0., 0., 0.);
  scale(scale_fac);
  std::cout << "in shape, centroid = " << centroid.transpose() << std::endl;
}

void OglObject::render(const Eigen::Matrix4f& vp) {
  // std::cout << "type: " << obj_type << "\n";
  if (obj_type == OBJECT_SPHERE_BIG) {
    mesh.render_new(vp, model, is_selected, obj_type);
  } else {
    mesh.render_new(vp, model, is_selected, obj_type);
  }
}

bool OglObject::is_hit(const Eigen::Ref<const Eigen::Vector3f> origin,
                const Eigen::Ref<const Eigen::Vector3f> direction) {
  return mesh.is_hit_new(origin, direction);
}

void OglObject::translate(float dis_x, float dis_y, float dis_z) {
  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  trans.col(3) << dis_x, dis_y, dis_z, 1.0;
  Eigen::Vector4f ext_cent;
  ext_cent << centroid(0), centroid(1), centroid(2), 1.0;
  ext_cent = trans * ext_cent;
  centroid << ext_cent(0), ext_cent(1), ext_cent(2);
  origin = centroid;

  model = trans * model;
}

void OglObject::translate(const Eigen::Ref<const Eigen::Vector3f> new_pos) {
  translate(new_pos(0), new_pos(1), new_pos(2));
}

void OglObject::move_to(const Eigen::Ref<const Eigen::Vector3f> new_pos) {
  std::cout << "before moving, model=\n" << model << std::endl;
  std::cout << "before moving, centroid=" << centroid.transpose() << std::endl;
  translate(new_pos(0) - centroid(0), new_pos(1) - centroid(1), new_pos(2) - centroid(2));
  std::cout << "after moving, centroid=" << centroid.transpose() << std::endl;
  std::cout << "after moving, model=\n" << model << std::endl;
}

void OglObject::move_to(float x, float y, float z) {
  Eigen::Vector3f new_pos(x, y, z);
  move_to(new_pos);
}

void OglObject::scale(float scale_x, float scale_y, float scale_z) {
    if (scale_x < EPSILON || scale_y < EPSILON || scale_z < EPSILON) {
        std::cerr << "invalid scale factors" << std::endl;
        return;
    }

    Eigen::Vector3f old_orig = origin;
    move_to(0., 0., 0.);

    Eigen::Matrix4f scale_mat = Eigen::Matrix4f::Identity();
    scale_mat(0, 0) = scale_x;
    scale_mat(1, 1) = scale_y;
    scale_mat(2, 2) = scale_z;
    model = scale_mat * model;

    std::cout << "scale:" << scale_x << std::endl;

    move_to(old_orig);

    std::cout << "after scale, centroid=" << centroid.transpose() << std::endl;
}

void OglObject::scale(float scale_factor) {
    scale(scale_factor, scale_factor, scale_factor);
}

void OglObject::rotate_x(float angle) {
    Eigen::Vector3f old_orig;
    std::cout << "centroid:" << centroid.transpose() << std::endl;
    old_orig << origin;
    std::cout << "old centroid:" << old_orig.transpose() << std::endl;
    move_to(0., 0., 0.);
    // X axis
    double degree = angle * PI / 180;
    float cosine = cos(degree);
    float sine = sin(degree);
    Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
    rotate(1, 1) = cosine;
    rotate(1, 2) = -sine;
    rotate(2, 1) = sine;
    rotate(2, 2) = cosine;
    model = rotate * model;

    move_to(old_orig);

    std::cout << "after rotate x, centroid=" << centroid.transpose() << std::endl;
}

void OglObject::rotate_y(float angle) {
    // Y axis
    Eigen::Vector3f old_origin;
    std::cout << "centroid:" << centroid.transpose() << std::endl;
    old_origin = origin;
    std::cout << "old_centroid:" << old_origin.transpose() << std::endl;
    move_to(0., 0., 0.);
    double degree = angle * PI / 180;
    float cosine = cos(degree);
    float sine = sin(degree);
    Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
    rotate(0, 0) = cosine;
    rotate(0, 2) = -sine;
    rotate(2, 0) = sine;
    rotate(2, 2) = cosine;
    model = rotate * model;

    move_to(old_origin);

    std::cout << "after rotate y, centroid=" << centroid.transpose() << std::endl;
}

void OglObject::rotate_z(float angle) {
    Eigen::Vector3f old_origin;
    std::cout << "centroid:" << centroid.transpose() << std::endl;
    old_origin = origin;
    std::cout << "old_origin:" << origin.transpose() << std::endl;
    move_to(0., 0., 0.);
    // Z axis
    double degree = angle * PI / 180;
    float cosine = cos(degree);
    float sine = sin(degree);
    Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
    rotate(0, 0) = cosine;
    rotate(0, 1) = -sine;
    rotate(1, 0) = sine;
    rotate(1, 1) = cosine;
    model = rotate * model;

    move_to(old_origin);
    std::cout << "after rotate z, centroid=" << centroid.transpose() << std::endl;
}

Sphere::Sphere(const std::string& filename, float scale_fac, RenderObject type): OglObject(filename, scale_fac, type)  {
        model = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f scl = Eigen::Matrix4f::Identity();
        scl(0, 0) = 0.2;
        scl(1, 1) = 0.2;
        scl(2, 2) = 0.2;
        trans.col(3) << -0.72, -0.72, 0., 1.0;
        model = trans * scl * model;
}

void Sphere::move_to(float x, float y, float z) {
  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  trans.col(3) << x, y, z, 1.0;
  Eigen::Vector4f ext_cent;
  ext_cent << centroid(0), centroid(1), centroid(2), 1.0;
  ext_cent = trans * ext_cent;
  centroid << ext_cent(0), ext_cent(1), ext_cent(2);
  origin = centroid;

  model = trans * model;
}