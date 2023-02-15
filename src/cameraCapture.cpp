/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-11-24 15:09:27
 * @LastEditors: Jesse
 * @LastEditTime: 2023-02-03 13:19:26
 */
#include "cameraCapture.h"
#include "glass.h"
#include "myEvent.h"
#include "frame.h"
#include <QMessageBox>
#include <qdatetime.h>
#include <iostream>
#include <io.h>
#include <direct.h>
#include <optimization.h>
#include "utils.h"

cameraCapture::cameraCapture(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui_cameraCapture)
{
    ui->setupUi(this);
    qApp->installEventFilter(this);
    ui->CloseCamera->setEnabled(false);
    connect(ui->OpenCamera, SIGNAL(clicked()), this, SLOT(ClickOpenCameraButton()));
    connect(ui->CloseCamera, SIGNAL(clicked()), this, SLOT(ClickCloseCameraButton()));
    connect(ui->Capture, SIGNAL(clicked()), this, SLOT(ClickCaptureButton()));
    connect(ui->testgl, SIGNAL(clicked()), this, SLOT(ClicktestglButton()));
    connect(&myTracker,&tracker::sendPOR,&capture,&subwin::showPOR);
    connect(&capture,&subwin::sendcalibPointID,this,cameraCapture::saveImage);
    Enalbeshow=true;
    listFiles("../output");
}

cameraCapture::~cameraCapture()
{
    delete ui; 
}

//------------------------------------button slot-----------------------------------
void cameraCapture::ClickOpenCameraButton()
{
    glass * g = glass::Instance();

    if(g->Start())
    {
        ui->OpenCamera->setEnabled(false);
        ui->CloseCamera->setEnabled(true);
    }

}

void cameraCapture::ClickCloseCameraButton()
{
    glass * g = glass::Instance();

    if(g->Stop())
    {
        ui->OpenCamera->setEnabled(true);
        ui->CloseCamera->setEnabled(false);
    }
}

void cameraCapture::ClickCaptureButton()
{
    capture.show();
}

void cameraCapture::ClicktestglButton()
{
    // cv::Point3d pup = {-2.37568,1.99954,39.3611};
    // cv::Point3d cc = {-2.46078,3.08303,43.2106};
    // optimization myopt(8,4,1.65,1,1.34,{320.168430300339, 245.003736613060});
    // auto result = myopt.process_oc({0.115912,0.0146294},{0.0849074,-0.172349},{-2.46078,3.08303,43.2106});
    // std::cout<<result.x<<','<<result.y<<','<<result.z<<std::endl;
    // std::cout<<Get_angle(cc-pup,result);
    mygl.show();
}
// -----------------------------------eventFilter-----------------------------------
bool cameraCapture::eventFilter(QObject *obj, QEvent *event)
{
    int curId = event->type();
	if (curId == MyEvent::showEventId)
    {
        MyEvent* e = dynamic_cast<MyEvent*>(event);
        if (e)
        {
            switch (e->m_val)
			{
			case SCENE:
				showImage(sceneImage);
				break;
			case LEFT:
				showImage(leftImage);
				break;
			case RIGHT:
				showImage(rightImage);
				break;
			default:
				break;
			}
        }
    }
	else if(curId == MyEvent::processEventId)
	{
		// subEdgeDection();
	}
	else if(curId == MyEvent::eyeTrackEventId)
	{
        Enalbeshow=false;
        myTimer = startTimer(1);
		// eyeTracking();
        // QMessageBox::information(NULL,"test","this is subwin!",QMessageBox::Ok, QMessageBox::Ok);
	}
    else if(curId == QEvent::Close)
    {
        if(obj == this)
        {
            glass * g = glass::Instance();
            g->Release();
        }
        else if(obj == &capture)
        {
            killTimer(myTimer);
            Enalbeshow=true;
            myTracker.file.close();
            capture.showNormal();

            // QMessageBox::information(NULL,"test","this is subwin!",QMessageBox::Ok, QMessageBox::Ok);
        }
        
    }

    return QObject::eventFilter(obj, event);
}

void cameraCapture::timerEvent(QTimerEvent *event)
{
    
    mylock.lock();
    cv::Mat img = leftImage.imgData.clone();
    mylock.unlock();
    
    myTracker.getPOR(img);
}
// --------------------------------------------------------------Interface function----------------------------------------------------------------
void cameraCapture::showImage(frame input)
{
    static QDateTime lastTime = QDateTime::currentDateTime();
    QDateTime curTime = QDateTime::currentDateTime();
    qint64 intervalTimeMS = lastTime.msecsTo(curTime);
    lastTime = curTime;
    if(intervalTimeMS>=1000)
    {
        std::cout<<intervalTimeMS<<"ms"<<std::endl;
    }
    
	if(input.imgData.empty())
	{
		return;
	}

	cv::Mat img= input.imgData.clone();
    int frameId = input.cameraID;
    long long timestamp = input.timeStamp;
    QImage image;

	if(img.channels()==1)
	{
		cv::cvtColor(img,img,cv::COLOR_GRAY2RGB);
	}
	else if(img.channels()==3)
	{
		cv::cvtColor(img,img,cv::COLOR_BGR2RGB);
	}

	

	switch(frameId)
	{
		case SCENE:
            cv::resize(img,img,cv::Size(ui->SceneCamera->width(),ui->SceneCamera->height()));
            image = QImage((const unsigned char*)(img.data),
            img.cols,img.rows,img.cols*img.channels(),
            QImage::Format_RGB888);
			ui->SceneCamera->setPixmap(QPixmap::fromImage(image));
			break;
		case LEFT:
            cv::resize(img,img,cv::Size(ui->LeftCamera->width(),ui->LeftCamera->height()));
            image = QImage((const unsigned char*)(img.data),
            img.cols,img.rows,img.cols*img.channels(),
            QImage::Format_RGB888);
			ui->LeftCamera->setPixmap(QPixmap::fromImage(image));
			break;
		case RIGHT:
            cv::resize(img,img,cv::Size(ui->RightCamera->width(),ui->RightCamera->height()));
            image = QImage((const unsigned char*)(img.data),
            img.cols,img.rows,img.cols*img.channels(),
            QImage::Format_RGB888);
			ui->RightCamera->setPixmap(QPixmap::fromImage(image));
			break;
		default:
			break;
	}
}

// -----------------------------------API-----------------------------------
void cameraCapture::showImageAPI(frame img)
{
    switch(img.cameraID)
    {
        case SCENE:
            sceneImage=img;
            break;
        case LEFT:
            mylock.lock();
            leftImage=img; 
            mylock.unlock();
            // emit sendPOR(myTracker.getPOR(leftImage.imgData));
            break;
        case RIGHT:
            rightImage=img;
            break;
        default:
            break;
    }
    if(Enalbeshow)
    {
        MyEvent showEvent(MyEvent::showEventId,this,img.cameraID);
        QApplication::sendEvent(this,&showEvent);
    }
}

void cameraCapture::listFiles(std::string dir) {
	//在目录后面加上"\\*.*"进行第一次搜索
	std::string newDir = dir + "\\*.*";
	//用于查找的句柄
	intptr_t handle;
	struct _finddata_t fileinfo;
	//第一次查找
	handle = _findfirst(newDir.c_str(), &fileinfo);
 
	if (handle == -1) {
		std::cout << "无文件" << std::endl;
		system("pause");
		return;
	}
 
	do
	{
		if (fileinfo.attrib & _A_SUBDIR) {//如果为文件夹，加上文件夹路径，再次遍历
			if (strcmp(fileinfo.name, ".") == 0 || strcmp(fileinfo.name, "..") == 0)
				continue;
 
			// 在目录后面加上"\\"和搜索到的目录名进行下一次搜索
			newDir = dir + "\\" + fileinfo.name;
			listFiles(newDir.c_str());//先遍历删除文件夹下的文件，再删除空的文件夹
			// cout << newDir.c_str() << endl;
			if (_rmdir(newDir.c_str()) == 0) {//删除空文件夹
				// cout << "delete empty dir success" << endl;
			}
			else {
				std::cout << "delete empty dir error" << std::endl;
			}
		}
		else{
			std::string file_path = dir + "\\" + fileinfo.name;
			// cout << file_path.c_str() << endl;
			if (remove(file_path.c_str()) == 0) {//删除文件
				// cout << "delete file success" << endl;
			}else{
				std::cout << "delete file error" << std::endl;
			}
		}
	} while (!_findnext(handle, &fileinfo));
 
	_findclose(handle);
	return;
}

void cameraCapture::saveImage(int id)
{
    static int i=0;
    static int last_id = 0;
    if(id!=last_id)
    {
        i=0;
        last_id = id;
    }
    myTracker.index = id;
    std::string path = "../output/"+std::to_string(id)+"_"+std::to_string(i++)+".bmp";
    mylock.lock();
    cv::Mat img = leftImage.imgData.clone();
    mylock.unlock();
    imwrite(path,img);
}