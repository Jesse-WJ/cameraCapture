#include "cameraCapture.h"

cameraCapture::cameraCapture(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui_cameraCapture)
{
    ui->setupUi(this);
}

cameraCapture::~cameraCapture()
{
    delete ui; 
}