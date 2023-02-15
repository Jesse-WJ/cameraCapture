/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-14 13:14:15
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 15:16:58
 */
#include "mygl.h"

MyGLWidget::MyGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setGeometry( 0, 0, 640, 480 ); //设置窗口的位置，即左上角为(0,0)点，大小为640*480
    //设置窗口的标题为“ goose's OpenGL Framework”
    setWindowTitle( "A goose's OpenGL Framework" ); 
}

void MyGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0,0,0,1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

}


void MyGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBegin(GL_TRIANGLES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(-5, -5, 0);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f( 5, -5, 0);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f( 0.0,  5, 0);
    glEnd();
}

void MyGLWidget::resizeGL(int width, int height)
{
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-width/2,width/2,-height/2,height/2,-1,1);
    glMatrixMode(GL_MODELVIEW);
}

