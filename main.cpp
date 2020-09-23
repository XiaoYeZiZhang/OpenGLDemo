#include<GL/glut.h>
#include <iostream>
#include <Eigen/Dense>

#define  GLUT_WHEEL_UP 3           //定义滚轮操作
#define  GLUT_WHEEL_DOWN 4
bool max(double a, double b) {
    return a > b;
}
bool min(double a, double b) {
    return a < b;
}
Eigen::Vector3d getRay(const Eigen::Matrix4d& transformationMatrix, const Eigen::Matrix4d& projectionMatrix);
// 绘制立方体
// 将立方体的八个顶点保存到一个数组里面
static const float vertex_list[][3] =
        {
                -0.5f, -0.5f, -0.5f,
                0.5f, -0.5f, -0.5f,
                -0.5f, 0.5f, -0.5f,
                0.5f, 0.5f, -0.5f,
                -0.5f, -0.5f, 0.5f,
                0.5f, -0.5f, 0.5f,
                -0.5f, 0.5f, 0.5f,
                0.5f, 0.5f, 0.5f,
        };

// 将要使用的顶点的序号保存到一个数组里面
static const GLint index_list[][2] =
        {
                {0, 1},
                {2, 3},
                {4, 5},
                {6, 7},
                {0, 2},
                {1, 3},
                {4, 6},
                {5, 7},
                {0, 4},
                {1, 5},
                {7, 3},
                {2, 6}
        };


bool mouseLeftDown;
bool mouseRightDown;
float mouseX, mouseY;
float cameraDistanceY;
float cameraDistanceX;
float cameraAngleX;
float cameraAngleY;
float times=1;

void mouseCB(int button, int state, int x, int y)
{
    mouseX = x;
    mouseY = y;

    if(button == GLUT_LEFT_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseLeftDown = true;
        }
        else if(state == GLUT_UP) {
            mouseLeftDown = false;
        }

    }

    else if(button == GLUT_RIGHT_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseRightDown = true;
        }
        else if(state == GLUT_UP)
            mouseRightDown = false;
    }

    else if (state == GLUT_UP && button == GLUT_WHEEL_UP)
    {
        times += 0.008f;
        glutPostRedisplay();
    }

    else if (state == GLUT_UP && button == GLUT_WHEEL_DOWN)
    {
        times -= 0.008f;
        glutPostRedisplay();
    }

}



void mouseMotionCB(int x, int y)
{
    if(mouseLeftDown)
    {
        cameraAngleY += (x - mouseX);
        cameraAngleX += (y - mouseY);
        mouseX = x;
        mouseY = y;
    }
    if(mouseRightDown)
    {
        cameraDistanceY += (y - mouseY) * 0.03f;
        cameraDistanceX += (x - mouseX) * 0.03f;
        mouseY = y;
        mouseX = x;
    }

    glutPostRedisplay();
}

// 绘制立方体
void DrawCube(void)
{
    int i,j;

    glBegin(GL_LINES);
    for(i=0; i<12; ++i) // 12 条线段

    {
        for(j=0; j<2; ++j) // 每条线段 2个顶点

        {
            glVertex3fv(vertex_list[index_list[i][j]]);
        }
    }
    glEnd();
}



void renderScene(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 模型变换
    glMatrixMode(GL_MODELVIEW);
    // 当前矩阵设置为单位矩阵
    glLoadIdentity();
    glPushMatrix();

    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();

    gluLookAt(0,0,1,0,0,-1,0,1,0);
    glOrtho(-1.5, 1.5, -1.5, 1.5, -10, 10);

    transformationMatrix(0,0) = times;
    transformationMatrix(1,1) = times;
    transformationMatrix(2,2) = times;
    glScalef(times, times, times);//缩放


    transformationMatrix(0,3) = cameraDistanceX;
    transformationMatrix(1,3) = -cameraDistanceY;
    glTranslatef(cameraDistanceX, -cameraDistanceY, 0); //平移


    transformationMatrix(1,1) = cos(cameraAngleX);
    transformationMatrix(1,2) = -sin(cameraAngleX);
    transformationMatrix(2,1) = sin(cameraAngleX);
    transformationMatrix(2,2) = cos(cameraAngleX);
    glRotatef(cameraAngleX, 1, 0, 0); //旋转

    transformationMatrix(0,0) = cos(cameraAngleY);
    transformationMatrix(0,3) = sin(cameraAngleY);
    transformationMatrix(2,0) = -sin(cameraAngleY);
    transformationMatrix(2,3) = cos(cameraAngleY);
    glRotatef(cameraAngleY, 0, 1, 0);

    Eigen::Matrix4d projectionMatrix = Eigen::Matrix4d::Identity();
    /*projectionMatrix(0, 0) = 1.0 / 3.0;
    projectionMatrix(1, 1) = 1.0 / 3.0;
    projectionMatrix(2, 2) = -(2.0 / 20);
    projectionMatrix(2, 3) = 0;*/

    if(mouseLeftDown) {
        Eigen::Vector3d ray = getRay(transformationMatrix, projectionMatrix);
        // draw line
        glBegin(GL_LINES);
        glLineWidth(3.0);
        glColor3f(1.0,1.0,0.0);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(ray(0),ray(1),ray(2));
        glEnd();
   }

    glPointSize(3.0);
    glBegin(GL_POINTS);
    glColor3f(0,1.0,0);

    glVertex3f(0.0,0.0,0.0);
    glEnd();

    glColor3f(0, 0, 1);
    DrawCube();
    glPopMatrix();
    glutSwapBuffers();
}


//box struct
struct Box
{
    Eigen::Vector3d min;
    Eigen::Vector3d max;
}boxes[3];

//simple Ray struct
struct Ray
{
    Eigen::Vector3d origin, direction;
    float t;

    Ray()
    {
        t = (float)INT_MAX;
        origin = Eigen::Vector3d::Zero();
        direction = Eigen::Vector3d::Zero();
    }
}eyeRay;

//ray Box intersection code
Eigen::Vector2d intersectBox(const Ray& ray, const Box& cube)
{
    Eigen::Vector3d inv_dir = Eigen::Vector3d(1.0f / ray.direction(0), 1.0f/ray.direction(1), 1.0f/ray.direction(2));
    Eigen::Vector3d tMin = Eigen::Vector3d((cube.min - ray.origin)(0) * inv_dir(0), (cube.min - ray.origin)(1) * inv_dir(1), (cube.min - ray.origin)(2) * inv_dir(2));
    Eigen::Vector3d tMax = Eigen::Vector3d((cube.max - ray.origin)(0) * inv_dir(0), (cube.max - ray.origin)(1) * inv_dir(1), (cube.max - ray.origin)(2) * inv_dir(2));
    Eigen::Vector3d t1 = Eigen::Vector3d(tMin(0) < tMax(0) ? tMin(0) : tMax(0), tMin(1) < tMax(1) ? tMin(1) : tMax(1), tMin(2) < tMax(2) ? tMin(2) : tMax(2));
    Eigen::Vector3d t2 = Eigen::Vector3d(tMin(0) > tMax(0) ? tMin(0) : tMax(0), tMin(1) > tMax(1) ? tMin(1) : tMax(1), tMin(2) > tMax(2) ? tMin(2) : tMax(2));
    float tNear = max(max(t1(0), t1(1)), t1(2));
    float tFar = min(min(t2(0), t2(1)), t2(2));

    return Eigen::Vector2d(tNear, tFar);
}

Eigen::Vector3d getRay(const Eigen::Matrix4d& transformationMatrix, const Eigen::Matrix4d& projectionMatrix) {
    Eigen::Vector3d camera_position = Eigen::Vector3d(0,0,0);

    float x = (2.0f * mouseX) / 800 - 1.0f;
    float y = 1.0f - (2.0f * mouseY) / 800;
    float z = 1.0f;
    Eigen::Vector3d ray_nds = Eigen::Vector3d(x, y, z);
    Eigen::Vector4d ray_clip = Eigen::Vector4d (ray_nds(0), ray_nds(1), ray_nds(2), 1.0);
    Eigen::Vector4d ray_eye = projectionMatrix.inverse() * ray_clip;
    Eigen::Vector4d ray_world = transformationMatrix.inverse() * ray_eye;

    if (ray_world(3) != 0.0)
    {
        ray_world(0) /= ray_world(3);
        ray_world(1) /= ray_world(3);
        ray_world(2) /= ray_world(3);
    }


    Eigen::Vector3d ray_dir = Eigen::Vector3d(ray_world(0) - camera_position(0),
                                              ray_world(1) - camera_position(1),
                                              ray_world(2) - camera_position(2));
    //ray_dir.normalize();

    return ray_dir;
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100,100);
    glutInitWindowSize(800, 800);


    glutCreateWindow("GLDemo");
    glutDisplayFunc(renderScene);
    glutIdleFunc(renderScene);
    glutMouseFunc(mouseCB);
    glutMotionFunc(mouseMotionCB);
    glutMainLoop();
    return 0;
}