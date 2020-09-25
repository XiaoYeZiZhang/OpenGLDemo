#include <GL/glut.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include "Struct.h"

#define GLUT_WHEEL_UP 3
#define GLUT_WHEEL_DOWN 4
Eigen::Vector3d GetRay(
    const Eigen::Vector3d &camera_position,
    const Eigen::Matrix4d &transformationMatrix,
    const Eigen::Matrix4d &projectionMatrix);
Object boundingbox;
Camera camera;
MouseState mouseState;

void mouseCB(int button, int state, int x, int y) {
    mouseState.SetMousePoseX(x);
    mouseState.SetMousePoseY(y);

    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            mouseState.SetMouseLeftDown(true);
        } else if (state == GLUT_UP) {
            mouseState.SetMouseLeftDown(false);
        }
    }

    else if (button == GLUT_RIGHT_BUTTON) {
        if (state == GLUT_DOWN) {
            mouseState.SetMouseRightDown(true);
        } else if (state == GLUT_UP)
            mouseState.SetMouseRightDown(false);
    }

    else if (state == GLUT_UP && button == GLUT_WHEEL_UP) {
        mouseState.SetMouseWheelUp(true);
        mouseState.SetMouseWheelDown(false);
    }

    else if (state == GLUT_UP && button == GLUT_WHEEL_DOWN) {
        mouseState.SetMouseWheelDown(true);
        mouseState.SetMouseWheelUp(false);
    }
}

void mouseMotionCB(int x, int y) {
    if (mouseState.IsMouseLeftDown()) {
        mouseState.SetMouseAngleX(
            mouseState.GetMouseAngleX() + (x - mouseState.GetMousePoseX()));
        mouseState.SetMouseAngleY(
            mouseState.GetMouseAngleY() + (y - mouseState.GetMousePoseY()));
        mouseState.SetMousePoseX(x);
        mouseState.SetMousePoseY(y);
    }
    if (mouseState.IsMouseLeftDown()) {
        mouseState.SetMousePoseY(y);
        mouseState.SetMousePoseX(x);
    }
    if (mouseState.IsMouseWheelUp()) {
        mouseState.SetMouseWheelValue(mouseState.GetMouseWheelValue() + 0.008f);
    }
    if (mouseState.IsMouseWheelDown()) {
        mouseState.SetMouseWheelValue(mouseState.GetMouseWheelValue() - 0.008f);
    }
    glutPostRedisplay();
}

void KeyBoards(unsigned char key, int x, int y) {
    switch (key) {
    case 'w':
        boundingbox.MoveObject(0.03, 1);
        glutPostRedisplay();
        break;
    case 'a':
        boundingbox.MoveObject(-0.03, 0);
        glutPostRedisplay();
        break;
    case 's':
        boundingbox.MoveObject(-0.03, 1);
        glutPostRedisplay();
        break;
    case 'd':
        boundingbox.MoveObject(0.03, 0);
        glutPostRedisplay();
        break;
    case 'f':
        boundingbox.MoveObject(0.03, 2);
        glutPostRedisplay();
        break;
    case 'b':
        boundingbox.MoveObject(-0.03, 2);
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
        camera.GetCamPos()[0], camera.GetCamPos()[1], camera.GetCamPos()[2],
        camera.GetCamCenter()[0], camera.GetCamCenter()[1],
        camera.GetCamCenter()[2], camera.GetCamUp()[0], camera.GetCamUp()[1],
        camera.GetCamUp()[2]); // view matrix

    Eigen::Matrix4d projectionMatrix = Eigen::Matrix4d::Identity();

    if (mouseState.IsMouseLeftDown()) {
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
                camera.GetCamPos()[0], camera.GetCamPos()[1],
                camera.GetCamPos()[2]),
            transformationMatrix.transpose(), projectionMatrix.transpose());

        // draw line
        glBegin(GL_LINES);
        glLineWidth(3.0);
        glColor3f(1.0, 1.0, 0.0);
        glVertex3f(
            camera.GetCamPos()[0], camera.GetCamPos()[1],
            camera.GetCamPos()[2]);
        glVertex3f(ray(0), ray(1), ray(2));
        glEnd();

        glPointSize(10.0);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0);
        glVertex3f(ray(0), ray(1), ray(2));
        glEnd();

        // intersection
        float minDistance = INT_MAX;
        int minTriangleIndex = -1;
        for (size_t i = 0; i < boundingbox.GetAllTriangles().size(); i++) {
            Triangle thisTriangle = boundingbox.GetAllTriangles()[i];
            float t;
            float u;
            float v;
            if (IsIntersectWithTriangle(
                    Eigen::Vector3d(
                        camera.GetCamPos()[0], camera.GetCamPos()[1],
                        camera.GetCamPos()[2]),
                    Eigen::Vector3d(ray(0), ray(1), ray(2)),
                    thisTriangle.GetVertex()[0], thisTriangle.GetVertex()[1],
                    thisTriangle.GetVertex()[2], t, u, v)) {

                Eigen::Vector3d intersectionPoint =
                    (1 - u - v) * thisTriangle.GetVertex()[0] +
                    thisTriangle.GetVertex()[1] * u +
                    thisTriangle.GetVertex()[2] * v;

                float distance =
                    pow((intersectionPoint(0) - camera.GetCamPos()[0]), 2) +
                    pow((intersectionPoint(1) - camera.GetCamPos()[1]), 2) +
                    pow((intersectionPoint(2) - camera.GetCamPos()[2]), 2);
                std::cout << "current triangle" << i << std::endl;
                if (distance < minDistance) {
                    minDistance = distance;
                    minTriangleIndex = i;
                }
            }
        }

        // draw this intersection plane
        if (minTriangleIndex != -1) {
            std::cout << "finally triangle id:" << minTriangleIndex
                      << std::endl;
            glBegin(GL_QUADS);
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3fv(
                boundingbox.m_vertex_list
                    [boundingbox.triangle_plane[minTriangleIndex + 1][0]]);
            glVertex3fv(
                boundingbox.m_vertex_list
                    [boundingbox.triangle_plane[minTriangleIndex + 1][1]]);
            glVertex3fv(
                boundingbox.m_vertex_list
                    [boundingbox.triangle_plane[minTriangleIndex + 1][2]]);
            glVertex3fv(
                boundingbox.m_vertex_list
                    [boundingbox.triangle_plane[minTriangleIndex + 1][3]]);
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
    float x = (2.0f * mouseState.GetMousePoseX()) / 800 - 1.0f;
    float y = 1.0f - (2.0f * mouseState.GetMousePoseY()) / 800;
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
    // set bounding box position
    float vertex_list[8][3] = {
        -0.5f, -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, -0.5f, 0.5f,
        -0.5f, 0.5f,  0.5f,  -0.5f, -0.5f, -0.5f, 0.5f,  0.5f,
        -0.5f, 0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,  0.5f,  0.5f,
    };

    GLint index_list[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                               {4, 6}, {5, 7}, {0, 4}, {1, 5}, {7, 3}, {2, 6}};

    boundingbox.SetVertexList(vertex_list);
    boundingbox.SetIndexList(index_list);

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