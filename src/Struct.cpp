//
// Created by root on 2020/9/25.
//
#include <GL/gl.h>
#include <iostream>
#include "Struct.h"
void Triangle::SetVertex(
    Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3) {
    m_v1 = v1;
    m_v3 = v3;
    m_v2 = v2;
}

int Scene::GetSceneHeight() {
    return m_height;
}

int Scene::GetSceneWidth() {
    return m_width;
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

void Object::SetChangePlaneOffset(float offset) {
    changePlaneOffset = offset;
}

float Object::GetChangePlaneOffset() {
    return changePlaneOffset;
}

void Object::ChangePlane(size_t planeNumber, float offset) {
    int axies = -1;
    if (planeNumber == 0 || planeNumber == 1) {
        axies = 2;
        if (planeNumber == 0) {
            m_vertex_list[4][axies] += offset;
            m_vertex_list[5][axies] += offset;
            m_vertex_list[6][axies] += offset;
            m_vertex_list[7][axies] += offset;
        } else {
            m_vertex_list[0][axies] -= offset;
            m_vertex_list[1][axies] -= offset;
            m_vertex_list[2][axies] -= offset;
            m_vertex_list[3][axies] -= offset;
        }
    } else if (planeNumber == 2 || planeNumber == 3) {
        axies = 0;
        if (planeNumber == 2) {
            m_vertex_list[2][axies] -= offset;
            m_vertex_list[6][axies] -= offset;
            m_vertex_list[4][axies] -= offset;
            m_vertex_list[0][axies] -= offset;
        } else {
            m_vertex_list[3][axies] += offset;
            m_vertex_list[7][axies] += offset;
            m_vertex_list[5][axies] += offset;
            m_vertex_list[1][axies] += offset;
        }
    } else {
        axies = 1;
        if (planeNumber == 4) {
            m_vertex_list[2][axies] += offset;
            m_vertex_list[3][axies] += offset;
            m_vertex_list[6][axies] += offset;
            m_vertex_list[7][axies] += offset;
        } else {
            m_vertex_list[0][axies] -= offset;
            m_vertex_list[1][axies] -= offset;
            m_vertex_list[4][axies] -= offset;
            m_vertex_list[5][axies] -= offset;
        }
    }
    SetAllTriangles();
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

void Camera::SetCamPos(float x, float y, float z) {
    cameraPos_x = x;
    cameraPos_y = y;
    cameraPos_z = z;
}

void Camera::SetCamFront(float x, float y, float z) {
    cameraFront_x = x;
    cameraFront_y = y;
    cameraFront_z = z;
}

void Camera::SetCameraParams(
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
}

Eigen::Vector3d Camera::GetCamPos() {
    return Eigen::Vector3d(cameraPos_x, cameraPos_y, cameraPos_z);
}

Eigen::Vector3d Camera::GetCamFront() {
    return Eigen::Vector3d(cameraFront_x, cameraFront_y, cameraFront_z);
}

Eigen::Vector3d Camera::GetCamUp() {
    return Eigen::Vector3d(cameraUp_x, cameraUp_y, cameraUp_z);
}

void MouseState::SetMousePoseX(int x) {
    mousepos_x = x;
}

float MouseState::GetMousePoseX() {
    return mousepos_x;
}

void MouseState::SetMousePoseY(int y) {
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
