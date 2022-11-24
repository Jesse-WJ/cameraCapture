#pragma once
#include "ui_cameraCapture.h"
#include <QMainWindow>

class cameraCapture : public QMainWindow {
    Q_OBJECT
    
public:
    cameraCapture(QWidget* parent = nullptr);
    ~cameraCapture();

private:
    Ui_cameraCapture* ui;
};