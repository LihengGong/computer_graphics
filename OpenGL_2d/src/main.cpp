// This example is heavily based on the tutorial at https://open.gl

// OpenGL Helpers to reduce the clutter
#include "Helpers.h"

// GLFW is necessary to handle the OpenGL context
#include <GLFW/glfw3.h>
#include <iostream>
// Linear Algebra Library
#include <Eigen/Core>

// Timer
#include <chrono>

#include <unistd.h>
#include <spawn.h>
#include <sys/wait.h>
// VertexBufferObject wrapper
VertexBufferObject VBO;

int TOTAL_TRIANGLE_NUM = 200;
int cur_triangle_num = 0;
// Contains the vertex positions
Eigen::MatrixXf V(2, 3 * TOTAL_TRIANGLE_NUM);
const double EPSILON = 0.00000001;
const double PI  =3.141592653589793238463;

void insert_mode_draw(GLFWwindow* window, bool is_clicked);

bool point_in_triangle(const Eigen::Ref<const Eigen::Vector2f> point,
                       const Eigen::Ref<const Eigen::Vector2f> t_a,
                       const Eigen::Ref<const Eigen::Vector2f> t_b,
                       const Eigen::Ref<const Eigen::Vector2f> t_c);

void translation_triangle(int triangle_index, const float& xdisplace, const float& ydisplace);

int mouse_click_times = 0;
int draw_mode = 0;
int vert_number = 0;
bool is_pressed = false;
Eigen::Vector2f Orig_Place = (Eigen::Vector2f() << -2.0, -2.0).finished();
Op_mode cur_mode = INIT_MODE;
TRANSLATION_SUBMODE cur_sub_mode = INIT_SUB_MODE;
int selected_triangle = -1;

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void screen_to_world(GLFWwindow* window, double& xworld, double& yworld) {
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    xworld = (xpos / double(width)) * 2 - 1.0;
    yworld = (height - 1 - ypos) / double(height) * 2 - 1.0;
}

void expand_vertices_matrix() {
    Eigen::MatrixXf V_NEW(2, 3 * TOTAL_TRIANGLE_NUM * 2);
    V_NEW << V, Eigen::MatrixXf::Zero(2, 3 * TOTAL_TRIANGLE_NUM);
    V.resize(2, 3 * TOTAL_TRIANGLE_NUM * 2);
    V << V_NEW;
    TOTAL_TRIANGLE_NUM *= 2;
}

void mouse_click_insertion_mode(const double xworld, const double yworld) {
    printf("Insertion mode\n");
    mouse_click_times++;
    printf("just increased mouse click times: %d\n", mouse_click_times);
    switch (mouse_click_times % 3) {
        case 1:
            printf("dot\n");
            V.col(cur_triangle_num * 3 + 0) << xworld, yworld;
            draw_mode = GL_POINTS;
            vert_number = 1;
            break;
        case 2:
            printf("clicked twice. line\n");
            V.col(cur_triangle_num * 3 + 1) << xworld, yworld;
            draw_mode = GL_LINES;
            vert_number = 2;
            break;
        case 0:
            printf("triangle\n");
            V.col(cur_triangle_num * 3 + 2) << xworld, yworld;
            draw_mode = GL_TRIANGLES;
            vert_number = 3;
            cur_triangle_num++;
            if (cur_triangle_num >= 0.6 * TOTAL_TRIANGLE_NUM) {
                printf("expanding matrix\n");
                expand_vertices_matrix();
                printf("expanding done\n");
            }
            printf("Total triangle number increases to %d\n", cur_triangle_num);
            break;
    }
}

// Find the first vertext index for the selected triangle. If no triangle is
// selected, return -1.
int find_selected_triangle(double xworld, double yworld) {
    Eigen::Vector2f cur_point;
    cur_point << xworld, yworld;
    for (int i = 0; i < cur_triangle_num * 3; i += 3) {
        if (point_in_triangle(cur_point, V.col(i), V.col(i+1), V.col(i+2))) {
            return i;
        }
    }
    return -1;
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    double xworld, yworld;
    screen_to_world(window, xworld, yworld);

    printf("enter function: old click times: %d\n", mouse_click_times);

    // if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            switch(cur_mode) {
            case INSERTION_MODE:
                mouse_click_insertion_mode(xworld, yworld);
                break;
            case TRANSLATION_MODE:
            case DELETE_MODE:
                is_pressed = true;
                if (selected_triangle == -1) {
                    double x_cur_world, y_cur_world;
                    screen_to_world(window, x_cur_world, y_cur_world);
                    selected_triangle = find_selected_triangle(x_cur_world, y_cur_world);
                    Orig_Place << xworld, yworld;
                    printf("selected: %d\n", selected_triangle);
                }
                break;
            default:
                break;
            }
        } else if (action == GLFW_RELEASE) {
            is_pressed = false;
            selected_triangle = -1;
            Orig_Place << -2.0, -2.0;
        }
    }

    VBO.update(V);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Update the position of the first vertex if the keys 1,2, or 3 are pressed
    switch (key)
    {
        case GLFW_KEY_1:
            V.col(0) << -0.5,  0.5;
            break;
        case GLFW_KEY_2:
            V.col(0) << 0,  0.5;
            break;
        case GLFW_KEY_3:
            V.col(0) << 0.5,  0.5;
            break;
        case GLFW_KEY_I:
            cur_mode = INSERTION_MODE;
            printf("enter insertion mode\n");
            break;
        case GLFW_KEY_O:
            cur_mode = TRANSLATION_MODE;
            printf("enter translation mode\n");
            break;
        case GLFW_KEY_P:
            cur_mode = DELETE_MODE;
            printf("enter delete mode\n");
            break;
        case GLFW_KEY_H:
            if (cur_mode == TRANSLATION_MODE) {
                printf("enter rotate clickwise sub-mode\n");
                cur_sub_mode = ROTATE_CLOCKWISE_MODE;
            }
            break;
        case GLFW_KEY_J:
            if (cur_mode == TRANSLATION_MODE) {
                printf("enter rotate counter-clickwise sub-mode\n");
                cur_sub_mode = ROTATE_COUNTERCLOCKWISE_MODE;
            }
            break;
        case GLFW_KEY_K:
            if (cur_mode == TRANSLATION_MODE) {
                printf("enter scale up sub-mode\n");
                cur_sub_mode = SCALE_UP_MODE;
            }
            break;
        case GLFW_KEY_L:
            if (cur_mode == TRANSLATION_MODE) {
                printf("enter scale down sub-mode\n");
                cur_sub_mode = SCALE_DOWN_MODE;
            }
            break;
        default:
            break;
    }

    // Upload the change to the GPU
    VBO.update(V);
}

void mouse_button_callback2(GLFWwindow* window, int button, int action, int mods) {
    double xworld, yworld;
    screen_to_world(window, xworld, yworld);

    printf("enter function callback2: old click times: %d\n", mouse_click_times);

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            switch(cur_mode) {
                case INSERTION_MODE:
                    mouse_click_insertion_mode(xworld, yworld);
                    break;
                case TRANSLATION_MODE:
                    double x_cur_world, y_cur_world;
                    screen_to_world(window, x_cur_world, y_cur_world);
                    selected_triangle = find_selected_triangle(x_cur_world, y_cur_world);
                    // Orig_Place << xworld, yworld;
                    printf("selected: %d\n", selected_triangle);
                    break;
                default:
                    break;
            }
        }
    }

    VBO.update(V);
}

double distance_square(const double x1, const double y1, const double x2, const double y2) {
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

void mouse_move_insertion_mode(GLFWwindow* window) {
    static double old_xworld = std::numeric_limits<double>::infinity();
    static double old_yworld = std::numeric_limits<double>::infinity();
    double xworld, yworld;

    screen_to_world(window, xworld, yworld);
    switch (mouse_click_times % 3) {
    case 1:
        V.col(cur_triangle_num * 3 + 1) << xworld, yworld;
        draw_mode = GL_LINES;
        vert_number = 2;
        old_xworld = xworld;
        old_yworld = yworld;
        break;
    case 2:
        if (distance_square(xworld, yworld, old_xworld, old_yworld) >
                            EPSILON * EPSILON) {
            V.col(cur_triangle_num * 3 + 2) << xworld, yworld;
            draw_mode = GL_TRIANGLES;
            vert_number = 3;
        } else {
            draw_mode = GL_LINES;
            vert_number = 2;
            old_xworld = xworld;
            old_yworld = yworld;
        }
        break;
    }

    VBO.update(V);
}

bool point_in_triangle(const Eigen::Ref<const Eigen::Vector2f> point,
                       const Eigen::Ref<const Eigen::Vector2f> t_a,
                       const Eigen::Ref<const Eigen::Vector2f> t_b,
                       const Eigen::Ref<const Eigen::Vector2f> t_c) {
    std::cout << "t_a=" << t_a.transpose() << ", "
        << "t_b=" << t_b.transpose() << "," << "t_c=" << t_c.transpose() << std::endl;

    Eigen::Vector2f vec_0 = t_b - t_a;
    Eigen::Vector2f vec_1 = t_c - t_a;
    Eigen::Vector2f vec_2 = point - t_a;
    double d00 = vec_0.dot(vec_0);
    double d01 = vec_0.dot(vec_1);
    double d11 = vec_1.dot(vec_1);
    double d20 = vec_2.dot(vec_0);
    double d21 = vec_2.dot(vec_1);
    double denom = d00 * d11 - d01 * d01;
    if (denom < EPSILON && denom > -EPSILON) {
        return false;
    }
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    printf("v=%f,u=%f,w=%f\n", v, u, w);
    return (u > 0) && (v > 0) && (w > 0);
}

void drag_selected_triangle(GLFWwindow* window, const Program& program) {
    // highlight selected triangle as blue
    glUniform3f(program.uniform("triangleColor"), 0.0f, 0.0f, 1.0f);
    double xworld, yworld;
    screen_to_world(window, xworld, yworld);
    double x_displace = xworld - Orig_Place(0);
    double y_displace = yworld - Orig_Place(1);
    Orig_Place << xworld, yworld;
    // for (int i = 0; i < 3; i++) {
    //     V.col(selected_triangle + i)(0) += x_displace;
    //     V.col(selected_triangle + i)(1) += y_displace;
    // }
    translation_triangle(selected_triangle, -x_displace, -y_displace);
    VBO.update(V);
    glDrawArrays(GL_TRIANGLES, selected_triangle, 3);
}

void drag_and_draw(GLFWwindow* window, const Program& program) {
    printf("selected triangle is %d\n", selected_triangle);
    glDrawArrays(GL_TRIANGLES, 0, selected_triangle);
    drag_selected_triangle(window, program);
    // reset to default uniform(color red)
    glUniform3f(program.uniform("triangleColor"), 1.0f, 0.0f, 0.0f);
    glDrawArrays(GL_TRIANGLES, selected_triangle + 3, 3 * cur_triangle_num - 3 - selected_triangle);
}

void handle_mouse_click_and_draw(GLFWwindow* window, const Program& program) {
    if (mouse_click_times > 0) {
        switch (cur_mode) {
            case INSERTION_MODE:
                mouse_move_insertion_mode(window);
                if (cur_triangle_num > 0) {
                    glDrawArrays(GL_TRIANGLES, 0, 3 * cur_triangle_num);
                }
                glDrawArrays(draw_mode, 3 * cur_triangle_num, vert_number);
                break;
            case TRANSLATION_MODE:
                if (is_pressed && selected_triangle != -1) {
                    drag_and_draw(window, program);
                } else if (cur_triangle_num > 0) {
                    glDrawArrays(GL_TRIANGLES, 0, 3 * cur_triangle_num);
                }
                break;
            case DELETE_MODE:
                if (is_pressed && selected_triangle != -1) {
                    for (int i = 0; i < 3; i++) {
                        V.col(selected_triangle + i) << -2.0, -2.0;
                    }
                    VBO.update(V);
                }
                if (cur_triangle_num > 0) {
                    glDrawArrays(GL_TRIANGLES, 0, 3 * cur_triangle_num);
                }
                break;
            default:
                break;
        }
    }
}

int setup(GLFWwindow* &window, VertexArrayObject& VAO, Program& program) {
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
    window = glfwCreateWindow(800, 600, "Hello World", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

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

    // Initialize the VAO
    // A Vertex Array Object (or VAO) is an object that describes how the vertex
    // attributes are stored in a Vertex Buffer Object (or VBO). This means that
    // the VAO is not the actual object storing the vertex data,
    // but the descriptor of the vertex data.
    
    VAO.init();
    VAO.bind();

    // Initialize the VBO with the vertices data
    // A VBO is a data container that lives in the GPU memory
    VBO.init();

    V.resize(2, 3 * TOTAL_TRIANGLE_NUM);
    // V << 0,  0.5, -0.5, 0.5, -0.5, -0.5;
    // V << 0., 0., 0., 0., 0., 0.;
    V << Eigen::MatrixXf::Zero(2, 3 * TOTAL_TRIANGLE_NUM);
    VBO.update(V);

    // Initialize the OpenGL Program
    // A program controls the OpenGL pipeline and it must contains
    // at least a vertex shader and a fragment shader to be valid
    
    const GLchar* vertex_shader =
            "#version 150 core\n"
                    "in vec2 position;"
                    "void main()"
                    "{"
                    "    gl_Position = vec4(position, 0.0, 1.0);"
                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "out vec4 outColor;"
                    "uniform vec3 triangleColor;"
                    "void main()"
                    "{"
                    "    outColor = vec4(triangleColor, 1.0);"
                    "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor
    // is the one that we want in the fragment buffer (and thus on screen)
    program.init(vertex_shader,fragment_shader,"outColor");
    program.bind();

    // The vertex shader wants the position of the vertices as an input.
    // The following line connects the VBO we defined above with the position "slot"
    // in the vertex shader
    program.bindVertexAttribArray("position",VBO);

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    // Update viewport
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    return 0;
}

void teardown(VertexArrayObject& VAO, Program& program) {
    // Deallocate opengl memory
    program.free();
    VAO.free();
    VBO.free();

    // Deallocate glfw internals
    glfwTerminate();
}

int task_1(void)
{
    GLFWwindow* window;
    VertexArrayObject VAO;
    Program program;
    printf("task 1\n");
    if (setup(window, VAO, program)) {
        exit(-1);
    }
    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();
    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window))
    {
        // Bind your VAO (not necessary if you have only one)
        VAO.bind();

        // Bind your program
        program.bind();

        // Set the uniform value depending on the time difference
        auto t_now = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();
        // glUniform3f(program.uniform("triangleColor"), (float)(sin(time * 4.0f) + 1.0f) / 2.0f, 0.0f, 0.0f);
        glUniform3f(program.uniform("triangleColor"), 1.0f, 0.0f, 0.0f);

        // Clear the framebuffer
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        handle_mouse_click_and_draw(window, program);

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    teardown(VAO, program);
    exit(0);
    return 0;
}

void get_baricenter(int triangle_index, float& x_center, float& y_center) {
    // float x_center = 0., y_center = 0.;
    for (int ind = 0; ind < 3; ++ind) {
        x_center += V.col(triangle_index + ind)(0);
        y_center += V.col(triangle_index + ind)(1);
    }
    x_center /= 3.0;
    y_center /= 3.0;
}

void translation_triangle(int triangle_index, const float& xdisplace, const float& ydisplace) {
    for (int ind = 0; ind < 3; ++ind) {
        V.col(triangle_index + ind)(0) -= xdisplace;
        V.col(triangle_index + ind)(1) -= ydisplace;
    }
}

void bari_rotate_triangle(int triangle_index, double degree) {
    float x_center, y_center;
    get_baricenter(triangle_index, x_center, y_center);
    // translate to origin
    translation_triangle(triangle_index, x_center, y_center);
    double theta = degree / 360 * 2 * PI;
    // rotate
    for (int ind = 0; ind < 3; ++ind) {
        double x = V.col(triangle_index + ind)(0);
        double y = V.col(triangle_index + ind)(1);
        V.col(triangle_index + ind)(0) = cos(theta) * x - sin(theta) * y;
        V.col(triangle_index + ind)(1) = sin(theta) * x + cos(theta) * y;
    }
    // translate back
    // for (int ind = 0; ind < 3; ++ind) {
    //     V.col(triangle_index + ind)(0) += x_center;
    //     V.col(triangle_index + ind)(1) += y_center;
    // }
    translation_triangle(triangle_index, -x_center, -y_center);
    VBO.update(V);
}

void bari_scale_triangle(int triangle_index, double scale) {
    if (scale < EPSILON) {
        printf("Wrong parameter. Make sure the scaling factor is positive\n");
        return;
    }

    float x_center, y_center;
    get_baricenter(triangle_index, x_center, y_center);
    // translate to origin
    translation_triangle(triangle_index, x_center, y_center);
    // scale
    for (int ind = 0; ind < 3; ++ind) {
        V.col(triangle_index + ind)(0) *= scale;
        V.col(triangle_index + ind)(1) *= scale;
    }
    translation_triangle(triangle_index, -x_center, -y_center);
    VBO.update(V);
}

void rotate_and_scale(GLFWwindow* window, const Program& program) {
    // printf("selected triangle is %d\n", selected_triangle);
    glDrawArrays(GL_TRIANGLES, 0, selected_triangle);
    glUniform3f(program.uniform("triangleColor"), 0.0f, 0.0f, 1.0f);
    // handle rotate and scale
    switch (cur_sub_mode) {
        case ROTATE_CLOCKWISE_MODE:
            bari_rotate_triangle(selected_triangle, -10.0);
            cur_sub_mode = INIT_SUB_MODE;
        break;
        case ROTATE_COUNTERCLOCKWISE_MODE:
            bari_rotate_triangle(selected_triangle, 10.0);
            cur_sub_mode = INIT_SUB_MODE;
        break;
        case SCALE_UP_MODE:
            bari_scale_triangle(selected_triangle, 1.25);
            cur_sub_mode = INIT_SUB_MODE;
        break;
        case SCALE_DOWN_MODE:
            bari_scale_triangle(selected_triangle, 0.75);
            cur_sub_mode = INIT_SUB_MODE;
        break;
        default:
        break;
    }
    glDrawArrays(GL_TRIANGLES, selected_triangle, 3);
    // reset to default uniform(color red)
    glUniform3f(program.uniform("triangleColor"), 1.0f, 0.0f, 0.0f);
    glDrawArrays(GL_TRIANGLES, selected_triangle + 3, 3 * cur_triangle_num - 3 - selected_triangle);
}

void handle_rotate_and_scale(GLFWwindow* window, const Program& program) {
    if (mouse_click_times > 0) {
        switch (cur_mode) {
            case INSERTION_MODE:
                mouse_move_insertion_mode(window);
                if (cur_triangle_num > 0) {
                    glDrawArrays(GL_TRIANGLES, 0, 3 * cur_triangle_num);
                }
                glDrawArrays(draw_mode, 3 * cur_triangle_num, vert_number);
                break;
            case TRANSLATION_MODE:
                if (selected_triangle != -1) {
                    rotate_and_scale(window, program);
                } else if (cur_triangle_num > 0) {
                    glDrawArrays(GL_TRIANGLES, 0, 3 * cur_triangle_num);
                }
                break;
            case DELETE_MODE:
                if (is_pressed && selected_triangle != -1) {
                    for (int i = 0; i < 3; i++) {
                        V.col(selected_triangle + i) << -2.0, -2.0;
                    }
                    VBO.update(V);
                }
                if (cur_triangle_num > 0) {
                    glDrawArrays(GL_TRIANGLES, 0, 3 * cur_triangle_num);
                }
                break;
            default:
                break;
        }
    }
}

int task_2(void) {
    GLFWwindow* window;
    VertexArrayObject VAO;
    Program program;
    printf("task 2\n");
    if (setup(window, VAO, program)) {
        exit(-1);
    }

    // Register the mouse callback
    glfwSetMouseButtonCallback(window, mouse_button_callback2);
    // Save the current time --- it will be used to dynamically change the triangle color
    auto t_start = std::chrono::high_resolution_clock::now();
    while (!glfwWindowShouldClose(window))
    {
        // Bind your VAO (not necessary if you have only one)
        VAO.bind();

        // Bind your program
        program.bind();

        // Set the uniform value depending on the time difference
        auto t_now = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration_cast<std::chrono::duration<float>>(t_now - t_start).count();
        // glUniform3f(program.uniform("triangleColor"), (float)(sin(time * 4.0f) + 1.0f) / 2.0f, 0.0f, 0.0f);
        glUniform3f(program.uniform("triangleColor"), 1.0f, 0.0f, 0.0f);

        // Clear the framebuffer
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        handle_rotate_and_scale(window, program);

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    teardown(VAO, program);
    exit(0);
    return 0;
}

int main(void) {
    pid_t pid = -1;
    int status;

    int task_number;
    do {
        printf("Select task number: enter 1, 2, 3, 4, or 5; enter 0 to exit.\n");
        scanf("%d", &task_number);
        fflush(NULL);
        switch (task_number) {
            case 1:
                pid = fork();
                if (pid == 0) {
                    task_1();
                }
            break;
            case 2:
                pid = fork();
                if (pid == 0) {
                    task_2();
                }
            break;
            case 3:
            break;
        }
    } while (task_number != 0);

    if (pid > 0) {
        fflush(NULL);
        if (waitpid(pid, &status, 0) != -1) {
            printf("Child process exited with status %d\n", status);
        }
    }

    return 0;
}