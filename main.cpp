#include <GL/glut.h>
#include <iostream>
#include <vector>
#include <map>
#include <Eigen/Dense>

#define GLUT_WHEEL_UP 3 //定义滚轮操作
#define GLUT_WHEEL_DOWN 4
bool max(double a, double b) {
    return a > b;
}
bool min(double a, double b) {
    return a < b;
}
Eigen::Vector3d GetRay(
    const Eigen::Vector3d &camera_position,
    const Eigen::Matrix4d &transformationMatrix,
    const Eigen::Matrix4d &projectionMatrix);

// simple Ray struct
struct Ray {
    Eigen::Vector3d m_origin, m_direction;
    float t;

    void SetDirection(Eigen::Vector3d direction) {
        m_direction = direction;
    }
    void SetOrigin(Eigen::Vector3d origin) {
        m_origin = origin;
    }

    Ray() {
        t = (float)INT_MAX;
        m_origin = Eigen::Vector3d::Zero();
        m_direction = Eigen::Vector3d::Zero();
    }
};
struct Triangle {
    Eigen::Vector3d m_v1;
    Eigen::Vector3d m_v2;
    Eigen::Vector3d m_v3;
    Triangle() {
        m_v1 = Eigen::Vector3d::Zero();
        m_v3 = Eigen::Vector3d::Zero();
        m_v2 = Eigen::Vector3d::Zero();
    }

    void SetVertex(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3) {
        m_v1 = v1;
        m_v3 = v3;
        m_v2 = v2;
    }
};

struct Object {
    Eigen::Vector3d min;
    Eigen::Vector3d max;
    float pos_y_offset;
    float pos_x_offset;
    float pos_z_offset;
    float m_vertex_list[8][3];
    GLint m_index_list[12][2];
    std::map<int, std::vector<int>> triangle_plane;

    Triangle t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
    std::vector<Triangle> triangles;
    void SetVertexList(float vertex_list[8][3]) {
        for (size_t i = 0; i < 8; i++) {
            for (size_t j = 0; j < 3; j++) {
                m_vertex_list[i][j] = vertex_list[i][j];
            }
        }
    }

    void SetIndexList(GLint index_list[12][2]) {
        for (size_t i = 0; i < 12; i++) {
            for (size_t j = 0; j < 2; j++) {
                m_index_list[i][j] = index_list[i][j];
            }
        }
    }

    void GetTwelveTriangle() {
        // front
        t1.SetVertex(
            Eigen::Vector3d(min(0), max(1), max(2)),
            Eigen::Vector3d(min(0), min(1), max(2)),
            Eigen::Vector3d(max(0), min(1), max(2)));
        t2.SetVertex(
            Eigen::Vector3d(min(0), max(1), max(2)),
            Eigen::Vector3d(max(0), max(1), max(2)),
            Eigen::Vector3d(max(0), min(1), max(2)));
        triangle_plane[1] = {2,3,7,6};
        triangle_plane[2] = {2,3,7,6};

        // back
        t3.SetVertex(
            Eigen::Vector3d(min(0), max(1), min(2)),
            Eigen::Vector3d(min(0), min(1), min(2)),
            Eigen::Vector3d(max(0), min(1), min(2)));
        t4.SetVertex(
            Eigen::Vector3d(min(0), max(1), min(2)),
            Eigen::Vector3d(max(0), max(1), min(2)),
            Eigen::Vector3d(max(0), min(1), min(2)));
        triangle_plane[3] = {4,5,1,0};
        triangle_plane[4] = {4,5,1,0};

        // left
        t5.SetVertex(
            Eigen::Vector3d(min(0), max(1), max(2)),
            Eigen::Vector3d(min(0), max(1), min(2)),
            Eigen::Vector3d(min(0), min(1), min(2)));
        t6.SetVertex(
            Eigen::Vector3d(min(0), max(1), max(2)),
            Eigen::Vector3d(min(0), min(1), max(2)),
            Eigen::Vector3d(min(0), min(1), min(2)));
        triangle_plane[5] = {4,6,2,0};
        triangle_plane[6] = {4,6,2,0};

        // right
        t7.SetVertex(
            Eigen::Vector3d(max(0), max(1), max(2)),
            Eigen::Vector3d(max(0), min(1), max(2)),
            Eigen::Vector3d(max(0), max(1), min(2)));
        t8.SetVertex(
            Eigen::Vector3d(max(0), min(1), max(2)),
            Eigen::Vector3d(max(0), max(1), min(2)),
            Eigen::Vector3d(max(0), min(1), min(2)));
        triangle_plane[7] = {5,7,3,1};
        triangle_plane[8] = {5,7,3,1};
        // up
        t9.SetVertex(
            Eigen::Vector3d(min(0), max(1), min(2)),
            Eigen::Vector3d(min(0), max(1), max(2)),
            Eigen::Vector3d(max(0), max(1), max(2)));
        t10.SetVertex(
            Eigen::Vector3d(max(0), max(1), max(2)),
            Eigen::Vector3d(min(0), max(1), min(2)),
            Eigen::Vector3d(max(0), max(1), min(2)));
        triangle_plane[9] = {4,5,7,6};
        triangle_plane[10] = {4,5,7,6};
        // down
        t11.SetVertex(
            Eigen::Vector3d(min(0), min(1), max(2)),
            Eigen::Vector3d(min(0), min(1), min(2)),
            Eigen::Vector3d(max(0), min(1), max(2)));
        t12.SetVertex(
            Eigen::Vector3d(min(0), min(1), min(2)),
            Eigen::Vector3d(max(0), min(1), max(2)),
            Eigen::Vector3d(max(0), min(1), min(2)));
        triangle_plane[11] = {0,1,3,2};
        triangle_plane[12] = {0,1,3,2};

        triangles.emplace_back(t1);
        triangles.emplace_back(t2);
        triangles.emplace_back(t3);
        triangles.emplace_back(t4);
        triangles.emplace_back(t5);
        triangles.emplace_back(t6);
        triangles.emplace_back(t7);
        triangles.emplace_back(t8);
        triangles.emplace_back(t9);
        triangles.emplace_back(t10);
        triangles.emplace_back(t11);
        triangles.emplace_back(t12);
    }

    void SetMinPoint(float x, float y, float z) {
        min = Eigen::Vector3d(x, y, z);
    }
    void SetMaxPoint(float x, float y, float z) {
        max = Eigen::Vector3d(x, y, z);
    }
    Object() {
        min = Eigen::Vector3d::Zero();
        max = Eigen::Vector3d::Zero();
        pos_x_offset = 0.0;
        pos_y_offset = 0.0;
        pos_z_offset = 0.0;
    }
};

struct Camera {
    float cameraPos_x;
    float cameraPos_y;
    float cameraPos_z;
    float cameraCenter_x;
    float cameraCenter_y;
    float cameraCenter_z;
    float cameraUp_x;
    float cameraUp_y;
    float cameraUp_z;

    void SetCameraParams(
        float pos_x, float pos_y, float pos_z, float center_x, float center_y,
        float center_z, float up_x, float up_y, float up_z) {
        cameraPos_x = pos_x;
        cameraPos_y = pos_y;
        cameraPos_z = pos_z;

        cameraCenter_x = center_x;
        cameraCenter_y = center_y;
        cameraCenter_z = center_z;

        cameraUp_x = up_x;
        cameraUp_y = up_y;
        cameraUp_z = up_z;
    }
};

struct MouseState {
    bool mouseLeftDown;
    bool mouseRightDown;
    bool mouseWheelUp;
    bool mouseWheelDown;
    float mousepos_x;
    float mousepos_y;
    float mouseangle_x;
    float mouseangle_y;
    float mouse_wheel_value;
    MouseState() {
        mouseLeftDown = false;
        mouseRightDown = false;
        mouseWheelUp = false;
        mouseWheelDown = false;
        mousepos_x = 0.0;
        mousepos_y = 0.0;
        mouse_wheel_value = 1.0;
        mouseangle_x = 0.0;
        mouseangle_y = 0.0;
    }
};

Object boundingbox;
Camera camera;
MouseState mouseState;

void mouseCB(int button, int state, int x, int y) {
    mouseState.mousepos_x = x;
    mouseState.mousepos_y = y;

    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            mouseState.mouseLeftDown = true;
        } else if (state == GLUT_UP) {
            mouseState.mouseLeftDown = false;
        }
    }

    else if (button == GLUT_RIGHT_BUTTON) {
        if (state == GLUT_DOWN) {
            mouseState.mouseRightDown = true;
        } else if (state == GLUT_UP)
            mouseState.mouseRightDown = false;
    }

    else if (state == GLUT_UP && button == GLUT_WHEEL_UP) {
        mouseState.mouseWheelUp = true;
        mouseState.mouseWheelDown = false;
    }

    else if (state == GLUT_UP && button == GLUT_WHEEL_DOWN) {
        mouseState.mouseWheelDown = true;
        mouseState.mouseWheelUp = false;
    }
}

void mouseMotionCB(int x, int y) {
//    if (mouseState.mouseLeftDown) {
//        mouseState.mouseangle_x += (x - mouseState.mousepos_x);
//        mouseState.mouseangle_y += (y - mouseState.mousepos_y);
//        mouseState.mousepos_x = x;
//        mouseState.mousepos_y = y;
//    }
//    if (mouseState.mouseRightDown) {
//        boundingbox.pos_y_offset += (y - mouseState.mousepos_y) * 0.03f;
//        boundingbox.pos_x_offset += (x - mouseState.mousepos_x) * 0.03f;
//        mouseState.mousepos_y = y;
//        mouseState.mousepos_x = x;
//    }
//    if (mouseState.mouseWheelUp) {
//        mouseState.mouse_wheel_value += 0.008f;
//    }
//    if (mouseState.mouseWheelDown) {
//        mouseState.mouse_wheel_value -= 0.008f;
//    }
    glutPostRedisplay();
}

void KeyBoards(unsigned char key, int x, int y) {
    switch (key) {
    case 'w':
        boundingbox.pos_y_offset -= 0.03;
        glutPostRedisplay();
        break;
    case 'a':
        boundingbox.pos_x_offset -= 0.03;
        glutPostRedisplay();
        break;
    case 's':
        boundingbox.pos_y_offset += 0.03;
        glutPostRedisplay();
        break;
    case 'd':
        boundingbox.pos_x_offset += 0.03;
        glutPostRedisplay();
        break;
    case 'f':
        boundingbox.pos_z_offset += 0.03;
        glutPostRedisplay();
        break;
    case 'b':
        boundingbox.pos_z_offset -= 0.03;
        glutPostRedisplay();
        break;
    default:
        break;
    case 27:
        exit(0);
        break;
    }
}

void DrawCube(void) {
    int i, j;

    glBegin(GL_LINES);
    for (i = 0; i < 12; ++i)

    {
        for (j = 0; j < 2; ++j)

        {
            glVertex3fv(
                boundingbox.m_vertex_list[boundingbox.m_index_list[i][j]]);
        }
    }
    glEnd();


}

// ray Box intersection code
bool IsIntersectWithCube(const Ray &ray, const Object &cube) {
    Eigen::Vector3d inv_dir = Eigen::Vector3d(
        1.0f / ray.m_direction(0), 1.0f / ray.m_direction(1),
        1.0f / ray.m_direction(2));
    Eigen::Vector3d tMin = Eigen::Vector3d(
        (cube.min - ray.m_origin)(0) * inv_dir(0),
        (cube.min - ray.m_origin)(1) * inv_dir(1),
        (cube.min - ray.m_origin)(2) * inv_dir(2));
    Eigen::Vector3d tMax = Eigen::Vector3d(
        (cube.max - ray.m_origin)(0) * inv_dir(0),
        (cube.max - ray.m_origin)(1) * inv_dir(1),
        (cube.max - ray.m_origin)(2) * inv_dir(2));
    Eigen::Vector3d t1 = Eigen::Vector3d(
        tMin(0) < tMax(0) ? tMin(0) : tMax(0),
        tMin(1) < tMax(1) ? tMin(1) : tMax(1),
        tMin(2) < tMax(2) ? tMin(2) : tMax(2));
    Eigen::Vector3d t2 = Eigen::Vector3d(
        tMin(0) > tMax(0) ? tMin(0) : tMax(0),
        tMin(1) > tMax(1) ? tMin(1) : tMax(1),
        tMin(2) > tMax(2) ? tMin(2) : tMax(2));
    float tNear = max(max(t1(0), t1(1)), t1(2));
    float tFar = min(min(t2(0), t2(1)), t2(2));

    return tNear <= tFar;
}

// Determine whether a ray intersect with a triangle
// Parameters
// orig: origin of the ray
// dir: direction of the ray
// v0, v1, v2: vertices of triangle
// t(out): weight of the intersection for the ray
// u(out), v(out): barycentric coordinate of intersection
bool IsIntersectWithTriangle(
    const Eigen::Vector3d &orig, const Eigen::Vector3d &dir,
    Eigen::Vector3d &v0, Eigen::Vector3d &v1, Eigen::Vector3d &v2, float &t,
    float &u, float &v) {
    // E1
    Eigen::Vector3d E1 = v1 - v0;

    // E2
    Eigen::Vector3d E2 = v2 - v0;

    // P
    Eigen::Vector3d P = dir.cross(E2);

    // determinant
    float det = E1.dot(P);

    // keep det > 0, modify T accordingly
    Eigen::Vector3d T;
    if (det > 0) {
        T = orig - v0;
    } else {
        T = v0 - orig;
        det = -det;
    }

    // If determinant is near zero, ray lies in plane of triangle
    if (det < 0.0001f)
        return false;

    // Calculate u and make sure u <= 1
    u = T.dot(P);
    if (u < 0.0f || u > det)
        return false;

    // Q
    Eigen::Vector3d Q = T.cross(E1);

    // Calculate v and make sure u + v <= 1
    v = dir.dot(Q);
    if (v < 0.0f || u + v > det)
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    t = E2.dot(Q);

    float fInvDet = 1.0f / det;
    t *= fInvDet;
    u *= fInvDet;
    v *= fInvDet;
    return true;
}

void renderScene(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//    glOrtho(-5, 5, -5, 5, 0.1, 100); // projection matrix
     gluPerspective(45, 1.333, 0.01, 100);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPushMatrix();

    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();

    camera.SetCameraParams(0.0, 0.0, 5, 0, 0, 0, 0, 1, 0);
    gluLookAt(
        camera.cameraPos_x, camera.cameraPos_y, camera.cameraPos_z,
        camera.cameraCenter_x, camera.cameraCenter_y, camera.cameraCenter_z,
        camera.cameraUp_x, camera.cameraUp_y, camera.cameraUp_z); // view matrix

    // model matrix
    // glScalef(
    // mouseState.mouse_wheel_value, mouseState.mouse_wheel_value,
    // mouseState.mouse_wheel_value); //缩放
//    glTranslatef(
//        boundingbox.pos_x_offset, -boundingbox.pos_y_offset,
//        boundingbox.pos_z_offset); //平移
        //glRotatef(mouseState.mouseangle_x, 1, 0, 0); //旋转
        //glRotatef(mouseState.mouseangle_y, 0, 1, 0);

    Eigen::Matrix4d projectionMatrix = Eigen::Matrix4d::Identity();

    if (mouseState.mouseLeftDown) {
        GLfloat view_model[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, view_model);
        GLdouble projection[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        transformationMatrix << view_model[0], view_model[1], view_model[2],
            view_model[3], view_model[4], view_model[5], view_model[6],
            view_model[7], view_model[8], view_model[9], view_model[10],
            view_model[11], view_model[12], view_model[13], view_model[14],
            view_model[15];

        projectionMatrix << projection[0], projection[1], projection[2],
            projection[3], projection[4], projection[5], projection[6],
            projection[7], projection[8], projection[9], projection[10],
            projection[11], projection[12], projection[13], projection[14],
            projection[15];

        Eigen::Vector3d ray = GetRay(
            Eigen::Vector3d(
                camera.cameraPos_x, camera.cameraPos_y, camera.cameraPos_z),
            transformationMatrix.transpose(), projectionMatrix.transpose());

        // draw line
        glBegin(GL_LINES);
        glLineWidth(3.0);
        glColor3f(1.0, 1.0, 0.0);
//        glVertex3f(camera.cameraPos_x, camera.cameraPos_y, camera.cameraPos_z-1.f);
        glVertex3f(camera.cameraPos_x, camera.cameraPos_y, camera.cameraPos_z);
        glVertex3f(ray(0)+camera.cameraPos_x, ray(1)+camera.cameraPos_y, ray(2)+camera.cameraPos_z);
//        glVertex3f(ray(0), ray(1), ray(2));
        glEnd();

        glPointSize(3.0);
        glBegin(GL_POINTS);
        glColor3f(0, 1.0, 0);
//        glVertex3f(camera.cameraPos_x, camera.cameraPos_y, camera.cameraPos_z-1.f);
        glVertex3f(camera.cameraPos_x, camera.cameraPos_y, camera.cameraPos_z);
//        glVertex3f(camera.cameraPos_x, camera.cameraPos_y, camera.cameraPos_z);
        glColor3f(1.0, 0, 0);
        glVertex3f(ray(0)+camera.cameraPos_x, ray(1)+camera.cameraPos_y, ray(2)+camera.cameraPos_z);
//        glVertex3f(ray(0), ray(1), ray(2));
        glEnd();

        Ray ray_forIntersection;
        ray_forIntersection.SetOrigin(Eigen::Vector3d(
            camera.cameraPos_x, camera.cameraPos_y, camera.cameraPos_z));
        ray_forIntersection.SetDirection(
            Eigen::Vector3d(ray(0), ray(1), ray(2)));

        float minDistance = INT_MAX;
        int minTriangleIndex = -1;
        for (size_t i = 0; i < boundingbox.triangles.size(); i++) {
            Triangle thisTriangle = boundingbox.triangles[i];
            float t;
            float u;
            float v;
            if (IsIntersectWithTriangle(
                    Eigen::Vector3d(
                        camera.cameraPos_x, camera.cameraPos_y,
                        camera.cameraPos_z),
                    Eigen::Vector3d(ray(0), ray(1), ray(2)), thisTriangle.m_v1,
                    thisTriangle.m_v2, thisTriangle.m_v3, t, u, v)) {

                std::cout << "insert with triangle\n";
                Eigen::Vector3d intersectionPoint =
                    (1 - u - v) * thisTriangle.m_v1 + thisTriangle.m_v2 * u +
                    thisTriangle.m_v3 * v;
                float distance =
                    pow((intersectionPoint(0) - camera.cameraPos_x), 2) +
                    pow((intersectionPoint(1) - camera.cameraPos_y), 2) +
                    pow((intersectionPoint(2) - camera.cameraPos_z), 2);
                if (distance < minDistance) {
                    std::cout << "minDistance " << distance << std::endl;
                    minDistance = distance;
                    minTriangleIndex = i;
                }
            }
        }

        // draw this intersection plane
        if(minTriangleIndex != -1) {
            glBegin(GL_QUADS);  //简单颜色绘两个面，一个单一色，一个渐变色
            glColor3f(0.0f,1.0f,0.0f);  //绿色
            glVertex3fv(boundingbox.m_vertex_list[boundingbox.triangle_plane[minTriangleIndex+1][0]]); //glVertex3fv(pnt[0]); glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3fv(boundingbox.m_vertex_list[boundingbox.triangle_plane[minTriangleIndex+1][1]]); //glVertex3fv(pnt[2]); glVertex3f(0.0f, 1.0f, 0.0f);
            glVertex3fv(boundingbox.m_vertex_list[boundingbox.triangle_plane[minTriangleIndex+1][2]]); //glVertex3fv(pnt[1]); glVertex3f(1.0f, 0.0f, 0.0f);
            glVertex3fv(boundingbox.m_vertex_list[boundingbox.triangle_plane[minTriangleIndex+1][3]]);
            glEnd();
        }
    }

    glColor3f(0, 0, 1);
    DrawCube();
    glPopMatrix();
    glutSwapBuffers();
}

Eigen::Vector3d GetRay(
    const Eigen::Vector3d &camera_position,
    const Eigen::Matrix4d &transformationMatrix,
    const Eigen::Matrix4d &projectionMatrix) {
    float x = (2.0f * mouseState.mousepos_x) / 800 - 1.0f;
    float y = 1.0f - (2.0f * mouseState.mousepos_y) / 800;
    float z = 1.0f;
    Eigen::Vector3d ray_nds = Eigen::Vector3d(x, y, z);
    Eigen::Vector4d ray_clip =
        Eigen::Vector4d(ray_nds(0), ray_nds(1), ray_nds(2), 1.0);
    Eigen::Vector4d ray_eye = projectionMatrix.inverse() * ray_clip;
    Eigen::Vector4d ray_world = transformationMatrix.inverse() * ray_eye;

    if (ray_world(3) != 0.0) {
        ray_world(0) /= ray_world(3);
        ray_world(1) /= ray_world(3);
        ray_world(2) /= ray_world(3);
    }

    return Eigen::Vector3d(ray_world(0), ray_world(1), ray_world(2));
}

int main(int argc, char **argv) {
    float vertex_list[8][3] = {
        -0.5f, -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, -0.5f, 0.5f,
        -0.5f, 0.5f,  0.5f,  -0.5f, -0.5f, -0.5f, 0.5f,  0.5f,
        -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,  0.5f,  0.5f,
    };

    GLint index_list[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                               {4, 6}, {5, 7}, {0, 4}, {1, 5}, {7, 3}, {2, 6}};

    boundingbox.SetVertexList(vertex_list);
    boundingbox.SetIndexList(index_list);
    boundingbox.SetMinPoint(-0.5, -0.5, -0.5);
    boundingbox.SetMaxPoint(0.5, 0.5, 0.5);
    boundingbox.GetTwelveTriangle();


    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(800, 800);

    glutCreateWindow("GLDemo");
    glutDisplayFunc(renderScene);
    glutIdleFunc(renderScene);
    glutMouseFunc(mouseCB);
    glutKeyboardFunc(&KeyBoards); //注册键盘事件
    glutMotionFunc(mouseMotionCB);
    glutMainLoop();
    return 0;
}