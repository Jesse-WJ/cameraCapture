/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-14 13:14:22
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 14:32:50
 */
#ifndef MYGL_H_
#define MYGL_H_

#include <GL/gl.h>
#include <GL/glu.h>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
 
class MyGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
 
public:
    MyGLWidget(QWidget *parent = nullptr);
    // ~MyGLWidget();
    
    void initializeGL();
    void paintGL();
    void resizeGL( int width, int height );
 
};

#endif