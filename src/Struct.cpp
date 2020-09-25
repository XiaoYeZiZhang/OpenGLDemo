//
// Created by root on 2020/9/25.
//
#include <GL/gl.h>
#include "Struct.h"
void Triangle::SetVertex(
    Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3) {
    m_v1 = v1;
    m_v3 = v3;
    m_v2 = v2;
}

void Object::MoveObject(float offset, int axies) {
    for (size_t i = 0; i < 8; i++) {
        m_vertex_list[i][axies] += offset;
    }
    SetAllTriangles();
}

std::vector<Triangle> Object::GetAllTriangles() {
    return triangles;
}

void Object::SetVertexList(float vertex_list[8][3]) {
    for (size_t i = 0; i < 8; i++) {
        for (size_t j = 0; j < 3; j++) {
            m_vertex_list[i][j] = vertex_list[i][j];
        }
    }
    minCornerPoint = Eigen::Vector3d(
        m_vertex_list[0][0], m_vertex_list[0][1], m_vertex_list[0][2]);
    maxCornerPoint = Eigen::Vector3d(
        m_vertex_list[7][0], m_vertex_list[7][1], m_vertex_list[7][2]);
    SetAllTriangles();
}

void Object::SetIndexList(GLint index_list[12][2]) {
    for (size_t i = 0; i < 12; i++) {
        for (size_t j = 0; j < 2; j++) {
            m_index_list[i][j] = index_list[i][j];
        }
    }
}

void Object::SetAllTriangles() {
    minCornerPoint = Eigen::Vector3d(
        m_vertex_list[0][0], m_vertex_list[0][1], m_vertex_list[0][2]);
    maxCornerPoint = Eigen::Vector3d(
        m_vertex_list[7][0], m_vertex_list[7][1], m_vertex_list[7][2]);
    // front
    t1.SetVertex(
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            minCornerPoint(0), minCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), maxCornerPoint(2)));
    t2.SetVertex(
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), maxCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), maxCornerPoint(2)));
    triangle_plane[1] = {6, 7, 5, 4};
    triangle_plane[2] = {6, 7, 5, 4};

    // back
    t3.SetVertex(
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            minCornerPoint(0), minCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), minCornerPoint(2)));
    t4.SetVertex(
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), maxCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), minCornerPoint(2)));
    triangle_plane[3] = {2, 3, 1, 0};
    triangle_plane[4] = {2, 3, 1, 0};

    // left
    t5.SetVertex(
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            minCornerPoint(0), minCornerPoint(1), minCornerPoint(2)));
    t6.SetVertex(
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            minCornerPoint(0), minCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            minCornerPoint(0), minCornerPoint(1), minCornerPoint(2)));
    triangle_plane[5] = {2, 0, 4, 6};
    triangle_plane[6] = {2, 0, 4, 6};

    // right
    t7.SetVertex(
        Eigen::Vector3d(
            maxCornerPoint(0), maxCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), maxCornerPoint(1), minCornerPoint(2)));
    t8.SetVertex(
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), maxCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), minCornerPoint(2)));
    triangle_plane[7] = {3, 7, 5, 1};
    triangle_plane[8] = {3, 7, 5, 1};
    // up
    t9.SetVertex(
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), maxCornerPoint(1), maxCornerPoint(2)));
    t10.SetVertex(
        Eigen::Vector3d(
            maxCornerPoint(0), maxCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            minCornerPoint(0), maxCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), maxCornerPoint(1), minCornerPoint(2)));
    triangle_plane[9] = {2, 3, 7, 6};
    triangle_plane[10] = {2, 3, 7, 6};
    // down
    t11.SetVertex(
        Eigen::Vector3d(
            minCornerPoint(0), minCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            minCornerPoint(0), minCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), maxCornerPoint(2)));
    t12.SetVertex(
        Eigen::Vector3d(
            minCornerPoint(0), minCornerPoint(1), minCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), maxCornerPoint(2)),
        Eigen::Vector3d(
            maxCornerPoint(0), minCornerPoint(1), minCornerPoint(2)));
    triangle_plane[11] = {0, 1, 5, 4};
    triangle_plane[12] = {0, 1, 5, 4};

    triangles.clear();
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

std::vector<Eigen::Vector3d> Triangle::GetVertex() {
    return {m_v1, m_v2, m_v3};
}

void Camera::SetCameraParams(
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

Eigen::Vector3d Camera::GetCamPos() {
    return Eigen::Vector3d(cameraPos_x, cameraPos_y, cameraPos_z);
}

Eigen::Vector3d Camera::GetCamCenter() {
    return Eigen::Vector3d(cameraCenter_x, cameraCenter_y, cameraCenter_z);
}

Eigen::Vector3d Camera::GetCamUp() {
    return Eigen::Vector3d(cameraUp_x, cameraUp_y, cameraUp_z);
}

void MouseState::SetMousePoseX(float x) {
    mousepos_x = x;
}

float MouseState::GetMousePoseX() {
    return mousepos_x;
}

void MouseState::SetMousePoseY(float y) {
    mousepos_y = y;
}

float MouseState::GetMousePoseY() {
    return mousepos_y;
}

void MouseState::SetMouseLeftDown(bool state) {
    mouseLeftDown = state;
}

bool MouseState::IsMouseLeftDown() {
    return mouseLeftDown;
}

void MouseState::SetMouseRightDown(bool state) {
    mouseRightDown = state;
}

bool MouseState::IsMouseRightDown() {
    return mouseRightDown;
}

void MouseState::SetMouseWheelUp(bool state) {
    mouseWheelUp = state;
}

bool MouseState::IsMouseWheelUp() {
    return mouseWheelUp;
}

void MouseState::SetMouseWheelDown(bool state) {
    mouseWheelDown = state;
}

bool MouseState::IsMouseWheelDown() {
    return mouseWheelDown;
}

void MouseState::SetMouseAngleX(float x) {
    mouseangle_x = x;
}

float MouseState::GetMouseAngleX() {
    return mouseangle_x;
}

void MouseState::SetMouseAngleY(float y) {
    mouseangle_y = y;
}

float MouseState::GetMouseAngleY() {
    return mouseangle_y;
}

float MouseState::GetMouseWheelValue() {
    return mouse_wheel_value;
}

void MouseState::SetMouseWheelValue(float value) {
    mouse_wheel_value = value;
}
