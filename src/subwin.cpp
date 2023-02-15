/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-06 21:38:31
 * @LastEditors: Jesse
 * @LastEditTime: 2023-01-10 16:08:26
 */
#include "subwin.h"
#include <opencv2/opencv.hpp>
#include <QImage>
#include <QMessageBox>
#include "myEvent.h"
#include <iostream>


subwin::subwin(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui_subwin)
{
    ui->setupUi(this);
    this->setContextMenuPolicy(Qt::CustomContextMenu);
    reflush();
}

subwin::~subwin()
{
    delete ui; 
}

void subwin::resizeEvent(QResizeEvent * e)
{
    reflush();
}

void subwin::reflush()
{
    cv::Mat img;
    if(screen.empty())
    {
        img = cv::Mat(this->height(),this->width(),CV_8UC3,cv::Scalar(0,0,0));
    }
    else
    {
        img=screen.clone();
    }
    
    cvtColor(img,img,cv::COLOR_BGR2RGB);
    QImage image = QImage((const unsigned char*)(img.data),
            img.cols,img.rows,img.cols*img.channels(),
            QImage::Format_RGB888);
    palette.setBrush(QPalette::Window,QBrush(QPixmap::fromImage(image)));         
    this->setPalette(palette);
}

void subwin::on_subwin_customContextMenuRequested(const QPoint &pos)
{
    QString tmp;
    QMenu *pMenu = new QMenu(this);
    QAction *maxScreen = new QAction(tr("全屏"), this);
    QAction *track = new QAction(tr("追踪"), this);
    QAction *quit = new QAction(tr("退出"), this);

    /* 添加菜单项 */
    pMenu->addAction(maxScreen);
    pMenu->addAction(track);
    pMenu->addAction(quit);

    /* 连接槽函数 */
    connect(maxScreen, SIGNAL(triggered()), this, SLOT(maxScreen()));
    connect(track, SIGNAL(triggered()), this, SLOT(track()));
    connect(quit, SIGNAL(triggered()), this, SLOT(close()));  //直接触发窗口的close函数

    /* 在鼠标右键处显示菜单 */
    pMenu->exec(cursor().pos());

    /* 释放内存 */
    QList<QAction*> list = pMenu->actions();
    foreach (QAction* pAction, list) delete pAction;
    delete pMenu;

}

void subwin::maxScreen()
{
   this->showFullScreen();
}

void subwin::track()
{
    MyEvent eyeTrackEvent(MyEvent::eyeTrackEventId,this);
    QApplication::sendEvent(this,&eyeTrackEvent);
    m.clear();
    myTimer = startTimer(600);
}

void subwin::timerEvent(QTimerEvent *event)
{
    moveDot = m.getmoveDot(2);
    if(m.isFinish())
    {
        killTimer(myTimer);
    }
}

void subwin::showPOR(cv::Point POR)
{
    screen.release();
    screen = cv::Mat(this->height(),this->width(),CV_8UC3,cv::Scalar(0,0,0));
    // cout<<moveDot.x<<','<<moveDot.y<<endl;
    if(!m.isFinish() && moveDot.x!=0)
    {
        cv::circle(screen,moveDot,20,cv::Scalar(0,255,0),-1);
        // cout<<moveDot.x<<','<<moveDot.y<<endl;
    }
    emit sendcalibPointID(m.getcalibPointID());
    circle(screen,POR,20,cv::Scalar(0,0,255),-1);
    reflush();
}