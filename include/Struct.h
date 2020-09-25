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

class Object {
public:
    Object() {
        minCornerPoint = Eigen::Vector3d::Zero();
        maxCornerPoint = Eigen::Vector3d::Zero();
    }
    void MoveObject(float offset, int axies);
    void SetVertexList(float vertex_list[8][3]);
    void SetIndexList(GLint index_list[12][2]);
    void SetAllTriangles();
    std::vector<Triangle> GetAllTriangles();
    float m_vertex_list[8][3];
    GLint m_index_list[12][2];
    std::map<int, std::vector<int>> triangle_plane;

private:
    Eigen::Vector3d minCornerPoint;
    Eigen::Vector3d maxCornerPoint;

    Triangle t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12;
    std::vector<Triangle> triangles;
};

class Camera {
public:
    void SetCameraParams(
        float pos_x, float pos_y, float pos_z, float center_x, float center_y,
        float center_z, float up_x, float up_y, float up_z);
    Eigen::Vector3d GetCamPos();
    Eigen::Vector3d GetCamCenter();
    Eigen::Vector3d GetCamUp();

private:
    float cameraPos_x;
    float cameraPos_y;
    float cameraPos_z;
    float cameraCenter_x;
    float cameraCenter_y;
    float cameraCenter_z;
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
    void SetMousePoseX(float x);
    float GetMousePoseX();
    void SetMousePoseY(float y);
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
