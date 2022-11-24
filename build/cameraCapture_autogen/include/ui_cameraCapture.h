/********************************************************************************
** Form generated from reading UI file 'cameraCapture.ui'
**
** Created by: Qt User Interface Compiler version 6.4.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAMERACAPTURE_H
#define UI_CAMERACAPTURE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cameraCapture
{
public:
    QWidget *centralwidget;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *cameraCapture)
    {
        if (cameraCapture->objectName().isEmpty())
            cameraCapture->setObjectName("cameraCapture");
        cameraCapture->resize(800, 600);
        centralwidget = new QWidget(cameraCapture);
        centralwidget->setObjectName("centralwidget");
        cameraCapture->setCentralWidget(centralwidget);
        menubar = new QMenuBar(cameraCapture);
        menubar->setObjectName("menubar");
        cameraCapture->setMenuBar(menubar);
        statusbar = new QStatusBar(cameraCapture);
        statusbar->setObjectName("statusbar");
        cameraCapture->setStatusBar(statusbar);

        retranslateUi(cameraCapture);

        QMetaObject::connectSlotsByName(cameraCapture);
    } // setupUi

    void retranslateUi(QMainWindow *cameraCapture)
    {
        cameraCapture->setWindowTitle(QCoreApplication::translate("cameraCapture", "cameraCapture", nullptr));
    } // retranslateUi

};

namespace Ui {
    class cameraCapture: public Ui_cameraCapture {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAMERACAPTURE_H
