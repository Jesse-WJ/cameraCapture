#include "myEvent.h"

MyEvent::MyEvent(int EventID,QObject*parent,int val):QEvent(Type(EventID)),m_parent(parent),m_val(val) //2、指定事件类型
{
}

MyEvent::~MyEvent()
{
    // delete m_parent;
}
