//
// Created by root on 2020/9/25.
//
#ifndef TESTOPENGL_STRUCT_H
#define TESTOPENGL_STRUCT_H
#include <Eigen/Dense>
#include <vector>
#include <map>

class Triangle {
public:
    Triangle() {
        m_v1 = Eigen::Vector3d::Zero();
        m_v3 = Eigen::Vector3d::Zero();
        m_v2 = Eigen::Vector3d::Zero();
    }
    void SetVertex(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3);
    std::vector<Eigen::Vector3d> GetVertex();

private:
    Eigen::Vector3d m_v1;
    Eigen::Vector3d m_v2;
    Eigen::Vector3d m_v3;
};

class Scene {
public:
    int GetSceneWidth();
    int GetSceneHeight();
    bool isChangingPlane;
    Scene(int width, int height) {
        m_width = width;
        m_height = height;
        isChangingPlane = false;
    }

private:
    int m_width;
    int m_height;
};

class Object {
public:
    Object() {
        minCornerPoint = Eigen::Vector3d::Zero();
        maxCornerPoint = Eigen::Vector3d::Zero();
        changePlaneOffset = 0.0;
    }
    void MoveObject(float offset, int axies);
    void SetVertexList(float vertex_list[8][3]);
    void SetIndexList(GLint index_list[12][2]);
    void SetAllTriangles();
    void SetChangePlaneOffset(float offset_x);
    float GetChangePlaneOffset();
    void ChangePlane(size_t planeNumber, float offset);
    std::vector<Triangle> GetAllTriangles();
    float m_vertex_list[8][3];
    GLint m_index_list[12][2];
    std::map<int, std::vector<int>> triangle_plane;
    int minTriangleIndex = -1;

private:
    Eigen::Vector3d minCornerPoint;
    Eigen::Vector3d maxCornerPoint;
    float changePlaneOffset;

    Triangle t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
    std::vector<Triangle> triangles;
};

class Camera {
public:
    void SetCameraParams(
        float pos_x, float pos_y, float pos_z, float front_x, float front_y,
        float front_z, float up_x, float up_y, float up_z);
    Eigen::Vector3d GetCamPos();
    void SetCamPos(float x, float y, float z);
    Eigen::Vector3d GetCamFront();
    void SetCamFront(float x, float y, float z);
    Eigen::Vector3d GetCamUp();

    Camera(
        float pos_x, float pos_y, float pos_z, float front_x, float front_y,
        float front_z, float up_x, float up_y, float up_z) {
        cameraPos_x = pos_x;
        cameraPos_y = pos_y;
        cameraPos_z = pos_z;
        cameraFront_x = front_x;
        cameraFront_y = front_y;
        cameraFront_z = front_z;
        cameraUp_x = up_x;
        cameraUp_y = up_y;
        cameraUp_z = up_z;
        yaw = -90.0f;
        pitch = 0.0f;
    }

    float yaw;
    float pitch;

private:
    float cameraPos_x;
    float cameraPos_y;
    float cameraPos_z;
    float cameraFront_x;
    float cameraFront_y;
    float cameraFront_z;
    float cameraUp_x;
    float cameraUp_y;
    float cameraUp_z;
};

class MouseState {
public:
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

    void SetMouseAngleX(float x);
    float GetMouseAngleX();
    void SetMouseAngleY(float y);
    float GetMouseAngleY();
    void SetMousePoseX(int x);
    float GetMousePoseX();
    void SetMousePoseY(int y);
    float GetMousePoseY();
    void SetMouseLeftDown(bool state);
    bool IsMouseLeftDown();
    void SetMouseRightDown(bool state);
    bool IsMouseRightDown();
    void SetMouseWheelUp(bool state);
    bool IsMouseWheelUp();
    void SetMouseWheelDown(bool state);
    bool IsMouseWheelDown();
    float GetMouseWheelValue();
    void SetMouseWheelValue(float value);

private:
    bool mouseLeftDown;
    bool mouseRightDown;
    bool mouseWheelUp;
    bool mouseWheelDown;
    float mousepos_x;
    float mousepos_y;
    float mouseangle_x;
    float mouseangle_y;
    float mouse_wheel_value;
};
#endif // TESTOPENGL_STRUCT_H
