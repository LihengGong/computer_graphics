// This example is heavily based on the tutorial at https://open.gl

// OpenGL Helpers to reduce the clutter
#include "Helpers.h"

#include <iostream>
#include <cmath>
#include <string>
#include <fstream>
#include <istream>
#include <vector>

// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>

// Linear Algebra Library
#include <Eigen/Core>
#include <Eigen/Dense>

// Timer
#include <chrono>


std::string obj_names[] = {
    "../data/cube.off",
    "../data/bumpy_cube.off",
    "../data/bunny.off",
    "../data/sphere.off"
};

RenderObject cur_obj = OBJECT_BUNNY;
ViewMode cur_view_mode = PERSPECTIVE_VIEW;
bool is_recompute_proj = true;

bool is_need_scale = true;

bool is_camera_control = false;
CamDirection cam_dir = CAM_DIR_NO_MOVE;

double view_near;

float cam_x = -0.661, cam_y = -0.661, cam_z = 0.;

int selected = -1;
Eigen::Matrix4f projection;
Eigen::Matrix4f camview;
Eigen::Matrix4f common_model;
Eigen::Vector3f eye_position;

Eigen::Matrix4f sphere_vp = Eigen::Matrix4f::Identity();
OglObject *pSphere;
OglObject *psmallSphere;

OglObject* cur_selected_obj;

std::vector<OglObject*> oglObj_vectors;

void build_camera_view(const Eigen::Ref<const Eigen::Vector3f> eye_pos,
                       const Eigen::Ref<const Eigen::Vector3f> center,
                       const Eigen::Ref<const Eigen::Vector3f> up_dir,
                       Eigen::Ref<Eigen::Matrix4f> cam_view);

void translate_common_model(float dis_x, float dis_y, float dis_z) {
  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
  trans.col(3) << dis_x, dis_y, dis_z, 1.0;

  common_model = trans * common_model;
}

void screen_to_world_distortion(GLFWwindow* window, OglObject *pOglObj,
                                Eigen::Ref<Eigen::Vector4f> vec_ray,
                                Eigen::Ref<Eigen::Vector4f> ray_orig) {
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    Eigen::Vector4f p_canonical((xpos / width) * 2.0 - 1.0,
                                1.0 - ((1 + ypos) / height) * 2.0, 1.0, 1.0);

    Eigen::Vector4f p_ndc_near((xpos / width) * 2.0 - 1.0,
                                1.0 - ((1 + ypos) / height) * 2.0, -1.0, 1.0);

    Eigen::Vector4f p_ndc_far((xpos / width) * 2.0 - 1.0,
                                1.0 - ((1 + ypos) / height) * 2.0, 1.0, 1.0);

    std::cout << "canonical of cursor: " << p_canonical.transpose() 
              << ", pshape" << pOglObj->name << std::endl;
    ///// projection * camview * model
    Eigen::Matrix4f inverse_mat;
    if (cur_view_mode == PERSPECTIVE_VIEW) {
        inverse_mat = (projection * camview * common_model * pOglObj->model).inverse();
        vec_ray = inverse_mat * p_canonical;
    } else {
        inverse_mat = (projection * camview * common_model * pOglObj->model).inverse();
        Eigen::Vector4f cam_near = projection.inverse() * p_ndc_near;
        Eigen::Vector4f cam_far = projection.inverse() * p_ndc_far;

        cam_near /= cam_near(3);
        cam_far /= cam_far(3);

        Eigen::Vector4f obj_near = (camview * common_model * pOglObj->model).inverse() * cam_near;
        Eigen::Vector4f obj_far = (camview * common_model * pOglObj->model).inverse() * cam_far;
        ray_orig = obj_near;
        vec_ray = obj_far - obj_near;
    }    
}

void scale(float scale_x, float scale_y, float scale_z) {
    Eigen::Matrix4f scale_mat = Eigen::Matrix4f::Identity();
    scale_mat(0, 0) = scale_x;
    scale_mat(1, 1) = scale_y;
    scale_mat(2, 2) = scale_z;

    common_model = scale_mat * common_model;
}

void scale(float s_factor) {
    scale(s_factor, s_factor, s_factor);
}

void scale_up() {
    scale(1.2);
}

void scale_down() {
    scale(0.8);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    // Get the position of the mouse in the window
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        Eigen::Vector4f vec_ray = Eigen::Vector4f::Zero();
        Eigen::Vector4f origin = Eigen::Vector4f::Zero();
        Eigen::Vector3f direction = Eigen::Vector3f::Zero();
        Eigen::Vector4f inv_origin = Eigen::Vector4f::Zero();
        Eigen::Vector3f new_orig = Eigen::Vector3f::Zero();
        Eigen::Vector4f ray_orig = Eigen::Vector4f::Zero();
        for (std::vector<OglObject*>::iterator it = oglObj_vectors.begin();
            it != oglObj_vectors.end(); ++it) {

            screen_to_world_distortion(window, (*it), vec_ray, ray_orig);
            if (cur_view_mode == PERSPECTIVE_VIEW) {
                Eigen::Vector4f pos_world = vec_ray;
                origin << eye_position(0), eye_position(1), eye_position(2), 1.0;
                inv_origin = ((common_model * (*it)->model).inverse()) * origin;
                new_orig << inv_origin(0), inv_origin(1), inv_origin(2);
                direction << pos_world(0), pos_world(1), pos_world(2);
                std::cout << "xorig=" << inv_origin(0) << ", yorig=" << inv_origin(1) << std::endl;
            }
            else {
                new_orig << ray_orig(0), ray_orig(1), ray_orig(2);
                direction << vec_ray(0), vec_ray(1), vec_ray(2);
            }

            std::cout << "xworld=" << new_orig(0) << ", yworld=" << new_orig(1) << std::endl;

            bool res = false;
            cur_selected_obj = NULL;

            res = (*it)->is_hit(new_orig, direction);
            (*it)->is_selected = res;

            if (res) {
                std::cout << "mesh " << (*it)->name << "\n";
                cur_selected_obj = *it;
                std::cout << "selected\n";
                break;
            }
        }
    }
}

void camera_track_ball() {
    if (is_camera_control) {
        static float x = eye_position(0);
        static float y = eye_position(1);
        static float z = eye_position(2);

        static float radius = sqrt(x*x + y*y + z*z);
        static float theta = acos(z / radius);
        static float phi = (x < EPSILON && x > -EPSILON) ? PI/2 : atan(y / x);

        std::cout << "x=" << x << ", y=" << y << ", z=" << z
                  << ", r=" << radius << ", theta=" << theta
                  << ", phi=" << phi << "\n";

        is_camera_control = false;
        switch (cam_dir) {
            case CAM_DIR_UP:
                phi += PI / 20.0;
            break;
            case CAM_DIR_DOWN:
                phi -= PI / 20.0;
            break;
            case CAM_DIR_LEFT:
                theta += PI / 20.0;
            break;
            case CAM_DIR_RIGHT:
                theta -= PI / 20.0;
            break;
            default:
            break;
        }
        cam_dir = CAM_DIR_NO_MOVE;

        x = radius * sin(theta) * cos(phi);
        y = radius * sin(theta) * sin(phi);
        z = radius * cos(theta);

        // trans.col(3) << -0.72, -0.72, 0., 1.0;
        // scl(0, 0) = 0.2;
        float cam_s_r = 0.166;
        float r_x = -0.661, r_y = -0.664, r_z = 0.;

        cam_x = r_x + cam_s_r * sin(theta) * cos(phi);
        cam_y = r_y + cam_s_r * sin(theta) * sin(phi);
        cam_z = r_z + cam_s_r * cos(theta);
        std::cout << "cam: x=" << cam_x << ", y=" << cam_y << ", z=" << cam_z << "\n";

        eye_position << x, y, z;
        Eigen::Vector3f origin;
        origin << 0., 0., 0.;
        Eigen::Vector3f up;
        up << 0., 1.0, 0.;

        std::cout << "x=" << x << ", y=" << y << ", z=" << z
                  << ", r=" << radius << ", theta=" << theta
                  << ", phi=" << phi << "\n";

        // psmallSphere->move_to(x, y, z);
        build_camera_view(eye_position, origin, up, camview);
    }
}

void create_object(RenderObject obj_type, float scl_f) {
    std::cout << "add " << obj_names[obj_type] << std::endl;
    OglObject *p_oglObj = new OglObject(obj_names[obj_type], scl_f, OBJECT_TOTAL_NUM);
    oglObj_vectors.push_back(p_oglObj);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_RELEASE) {
        return;
    }
    Eigen::Vector3f origin;
    origin << 0., 0., 0.;
    Eigen::Vector3f up;
    up << 0., 1.0, 0.;
    // Update the position of the first vertex if the keys 1,2, or 3 are pressed
    switch (key)
    {
        case GLFW_KEY_1:
            // add a cube
            cur_obj = OBJECT_CUBE;
            create_object(cur_obj, 1.0);
            break;
        case GLFW_KEY_2:
            // add a bumpy cube
            cur_obj = OBJECT_BUMPY_CUBE;
            create_object(cur_obj, 0.3);
            break;
        case GLFW_KEY_3:
            // add a bunny
            cur_obj = OBJECT_BUNNY;
            create_object(cur_obj, 20.0);
            break;
        case GLFW_KEY_4:
            translate_common_model(-0.5, 0., 0.);
            break;
        case GLFW_KEY_5:
            translate_common_model(0.5, 0., 0.);
            break;
        case GLFW_KEY_6:
            translate_common_model(0., -0.5, 0.);
            break;
        case GLFW_KEY_7:
            translate_common_model(0., 0.5, 0.);
            break;
        case GLFW_KEY_8:
            translate_common_model(0., 0., -0.5);
            break;
        case GLFW_KEY_9:
            translate_common_model(0., 0., 0.5);
            break;
        case GLFW_KEY_P:
            cur_view_mode = PERSPECTIVE_VIEW;
            is_recompute_proj = true;
            std::cout << "Change to perspective view\n";
            break;
        case GLFW_KEY_O:
            cur_view_mode = ORTHOGRAPHIC_VIEW;
            is_recompute_proj = true;
            std::cout << "Change to orthographic view\n";
            break;
        case GLFW_KEY_Z:
            if (true) {
                Shape::shading_mode = WIRE_FRAME_SHADING;
                std::cout << "Wire Frame Shading\n";
            }
            break;
        case GLFW_KEY_X:
            if (true) {
                Shape::shading_mode = FLAT_SHADING;
                std::cout << "Flat Shading\n";
            }
            break;
        case GLFW_KEY_C:
            if (true) {
                Shape::shading_mode = PHONG_SHADING;
                std::cout << "Flat Shading\n";
            }
            break;
        case GLFW_KEY_W:
            if (cur_selected_obj) {
                cur_selected_obj->pan_up();
                std::cout << "Pan up\n";
            }
            break;
        case GLFW_KEY_S:
            if (cur_selected_obj) {
                cur_selected_obj->pan_down();
                std::cout << "Obj Pan down\n";
            }
            break;
        case GLFW_KEY_A:
            if (cur_selected_obj) {
                cur_selected_obj->pan_left();
                std::cout << "Obj Pan left\n";
            }
            break;
        case GLFW_KEY_D:
            if (cur_selected_obj) {
                cur_selected_obj->pan_right();
                std::cout << "Obj Pan right\n";
            }
            break;
        case GLFW_KEY_Q:
            if (cur_selected_obj) {
                cur_selected_obj->pan_near();
                std::cout << "Obj pan near\n";
            }
            break;
        case GLFW_KEY_E:
            if (cur_selected_obj) {
                cur_selected_obj->pan_far();
                std::cout << "Obj pan far\n";
            }
            break;
        case GLFW_KEY_UP:
            if (cur_selected_obj) {
                cur_selected_obj->rotate_z(10.0);
            }
            break;
        case GLFW_KEY_DOWN:
            if (cur_selected_obj) {
                cur_selected_obj->rotate_z(-10.0);
            }
            break;
        case GLFW_KEY_LEFT:
            if (cur_selected_obj) {
                cur_selected_obj->rotate_y(10.0);
            }
            break;
        case GLFW_KEY_RIGHT:
            if (cur_selected_obj) {
                cur_selected_obj->rotate_y(-10.0);
            }
            break;
        case GLFW_KEY_COMMA:
            if (cur_selected_obj) {
                cur_selected_obj->rotate_x(10.0);
            }
            break;
        case GLFW_KEY_PERIOD:
            if (cur_selected_obj) {
                cur_selected_obj->rotate_x(-10.0);
            }
            break;
        case GLFW_KEY_EQUAL:
            if (cur_selected_obj) {
                cur_selected_obj->scale_up();
            }
            break;
        case GLFW_KEY_MINUS:
            if (cur_selected_obj) {
                cur_selected_obj->scale_down();
            }
            break;
        case GLFW_KEY_H:
            cam_dir = CAM_DIR_UP;
            is_camera_control = true;
            camera_track_ball();
            break;
        case GLFW_KEY_J:
            cam_dir = CAM_DIR_DOWN;
            is_camera_control = true;
            camera_track_ball();
            break;
        case GLFW_KEY_K:
            cam_dir = CAM_DIR_LEFT;
            is_camera_control = true;
            camera_track_ball();
            break;
        case GLFW_KEY_L:
            cam_dir = CAM_DIR_RIGHT;
            is_camera_control = true;
            camera_track_ball();
            break;
        case GLFW_KEY_M:
            scale_up();
            break;
        case GLFW_KEY_N:
            scale_down();
            break;
        default:
            break;
    }
}

void build_camera_view(const Eigen::Ref<const Eigen::Vector3f> eye_pos,
                       const Eigen::Ref<const Eigen::Vector3f> center,
                       const Eigen::Ref<const Eigen::Vector3f> up_dir,
                       Eigen::Ref<Eigen::Matrix4f> cam_view) {
    Eigen::Vector3f w = (eye_pos - center).normalized();
    Eigen::Vector3f u = up_dir.normalized().cross(w).normalized();
    Eigen::Vector3f v = w.cross(u);

    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat.block<3, 4>(0, 0) << u, v, w, eye_pos;
    cam_view = mat.inverse().eval();
    std::cout << "camview=" << cam_view << std::endl;
}

void build_ortho(const float& left, const float& right,
                 const float& bottom, const float& top,
                 const float& zNear, const float& zFar,
                 Eigen::Ref<Eigen::Matrix4f> orth_proj) {
    orth_proj = Eigen::Matrix4f::Identity();
    orth_proj(0, 0) = 2.0 / (right - left);
    orth_proj(1, 1) = 2.0 / (top - bottom);
    orth_proj(2, 2) = -2.0 / (zFar - zNear);
    orth_proj(0, 3) = -(right + left) / (right - left);
    orth_proj(1, 3) = -(top + bottom) / (top - bottom);
    orth_proj(2, 3) = -(zFar + zNear) / (zFar - zNear);

    // std::cout << "ortho=" << orth_proj << std::endl;
}

void build_pers(const float& fov, const float& aspect,
                const float& znear, const float& zfar,
                Eigen::Ref<Eigen::Matrix4f> pers_proj) {
    float f = 1.0 / (float)tan(fov * PI / 180.0);
    pers_proj = Eigen::Matrix4f::Identity();
    pers_proj(0, 0) = f / aspect;
    pers_proj(1, 1) = f;
    pers_proj(2, 2) = (zfar + znear) / (znear - zfar);
    pers_proj(3, 2) = -1.0;
    pers_proj(2, 3) = 2.0 * zfar * znear / (znear - zfar);
    pers_proj(3, 3) = 0.;

    // std::cout << "pers=" << pers_proj << std::endl;
}

int main(void)
{
    GLFWwindow* window;

    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

    // On apple we have to load a core profile with forward compatibility
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(800, 800, "Hello World", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    #ifndef __APPLE__
      glewExperimental = true;
      GLenum err = glewInit();
      if(GLEW_OK != err)
      {
        /* Problem: glewInit failed, something is seriously wrong. */
       fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      }
      glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
      fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
    #endif

    int major, minor, rev;
    major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    Shape::init();

    //create VBO
    // Cube cube;
    std::string bumpy_cube = "../data/bumpy_cube.off";
    std::string bunny = "../data/bunny.off";
    std::string cube = "../data/cube.off";
    std::string sphere = "../data/sphere.off";

    // Shape *pbunny = new Mesh(bunny, 32.0);
    // shape_vectors.push_back(pbunny);

    OglObject *pOglObj = new OglObject(bunny, 20.0, OBJECT_TOTAL_NUM);
    oglObj_vectors.push_back(pOglObj);

    pSphere = new OglObject(obj_names[3], 1.0, OBJECT_SPHERE_BIG);
    Sphere *pSphereCenter = new Sphere(obj_names[3], 0.5, OBJECT_SPHERE_CENTER);
    psmallSphere = new Sphere(obj_names[3], 0.5, OBJECT_SPHERE_SMALL);
    // psmallSphere->move_to(-0.32, -0.32, 0.);

    // Shape *pcube2 = new Mesh(cube, 1.0);
    // shape_vectors.push_back(pcube2);

    float pers_near = 0.1, pers_far = 1000.0;
    float ortho_near = -10.0, ortho_far = 10.0;
    float left = -10.0;
    float right = 10.0;
    float bottom = -10.0;
    float top = 10.0;
    float scale_factor = 1.0;
    float view_angle_degree = 45.0;

    projection = Eigen::Matrix4f::Identity();
    camview = Eigen::Matrix4f::Identity();
    common_model = Eigen::Matrix4f::Identity();
    Eigen::Vector3f origin;
    // Initialize the VBO with the vertices data
    // A VBO is a data container that lives in the GPU memory
    is_need_scale = true;

    eye_position << 0., 0.0, 6.0; 
    origin << 0., 0., 0.;

    Eigen::Vector3f up;
    up << 0., 1.0, 0.;
    build_camera_view(eye_position, origin, up, camview);

    int width, height;
    glfwGetWindowSize(window, &width, &height);
    float aspect = (float)width / (float)height;
    std::cout << "res=" << projection * camview << std::endl;

    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    long long int nbFrames = 0;
    long long int newFrames = 0;

    double lastTime = glfwGetTime();
    double newLastTime = glfwGetTime();

    const double maxFPS = 60.0;
    const double maxPeriod = 1.0 / maxFPS;
    int counter = 0;

    glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window))
    {
        Shape::VAO.bind();
        Shape::program.bind();
        
        // Set the uniform value depending on the time difference
        auto t_now = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();
        float degree = abs(150 * sin(time / 600.0f) + 0.5f) + 0.5f;

        // Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
        // float cosine = cos(degree);
        // float sine = sin(degree);

        double newCurrentTime = glfwGetTime();

        double deltaTime = newCurrentTime - newLastTime;
        if (deltaTime >= maxPeriod) {
            newLastTime = newCurrentTime;
            double currentTime = glfwGetTime();
            nbFrames++;
            if ( currentTime - lastTime >= 1.0 ){
                // printf and reset timer
                ++counter;
                if (counter % 8 == 0) {
                    counter = 0;
                    printf("%f ms/frame, %lld frames/s\n", 1000.0/double(nbFrames), nbFrames);
                }
                nbFrames = 0;
                lastTime += 1.0;
            }

            if (is_recompute_proj) {
                if (cur_view_mode == PERSPECTIVE_VIEW) {
                    build_pers(view_angle_degree, aspect, pers_near, pers_far, projection);
                    std::cout << "va=" << view_angle_degree << ", asp=" << aspect
                                << ", persnear=" << pers_near << ", persfar=" << pers_far << std::endl;
                } else {
                    build_ortho(left, right, bottom, top, ortho_near, ortho_far, projection);
                    std::cout << "lft=" << left << ", right=" << right << ", bottom=" << bottom
                                << ", top=" << top << ", orthonear=" << ortho_near << ", orthfar=" 
                                << ortho_far << std::endl;
                }

                is_recompute_proj = false;
                std::cout << "projection = " << projection << std::endl;
            }

            glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            Eigen::Matrix4f vp = (projection * camview * common_model);
            glUniformMatrix4fv(Shape::program.uniform("view"), 1, GL_FALSE, vp.data());

            pSphere->render(sphere_vp);
            psmallSphere->render(sphere_vp);
            pSphereCenter->render(sphere_vp);
            for (std::vector<OglObject*>::iterator it = oglObj_vectors.begin();
                it != oglObj_vectors.end(); ++it) {
                (*it)->render(vp);
            }

            // Swap front and back buffers
            glfwSwapBuffers(window);
        }
        // Poll for and process events
        glfwPollEvents();
    }

    glfwDestroyWindow(window);

    // Deallocate glfw internals
    glfwTerminate();
    return 0;
}
