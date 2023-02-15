#ifndef MYEVENT_H_
#define MYEVENT_H_

#include <QEvent>
#include <QMainWindow>

//继承QEvent创建自定义事件类，注意初始化向量表时需要初始化事件ID。
class MyEvent :public QEvent
{
public:
    MyEvent(int myShowEvent,QObject*parent,int val=0);
    ~MyEvent();
    int m_val;
    QObject* m_parent;
    enum eventId{userId=QEvent::User,showEventId,processEventId,eyeTrackEventId};
};

#endif