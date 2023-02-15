#include "glass.h"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <cstdlib>

glass* glass::p=nullptr;

glass::CGarbo glass::Garbo;

glass* glass::Instance()
{
	if(!p)
	{
		p=new glass();
	}
    return p;
}


// 前置场景图像回调
void _7INVENSUN_CALL glass::SceneImageCallback(uint8_t* image, int32_t size, 
	int32_t width, int32_t height, int64_t timestamp)
{
	static cv::Mat img(height, width, CV_8UC3);
	memcpy_s(img.data, size, image, size);
	// imshow("",img);
	// waitKey(1);
	// Mat img = imread("D:/Jesse/Desktop/subPixelEdgeDetection/image/1_0.bmp");
	frame sceneImage(img,cameraCapture::SCENE,timestamp);
	glass * g = glass::Instance();
	g->w.showImageAPI(sceneImage);
}

// 眼图回调
void _7INVENSUN_CALL glass::EyeImageCallback(int32_t flag, uint8_t* image, int32_t size, 
	int32_t width, int32_t height, int64_t timestamp)
{
	// 不要回调里，做耗时的处理，尽量快速返回!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	if (1 == flag) // left eye
	{		
		static cv::Mat left_img(height, width, CV_8UC1);
		memcpy_s(left_img.data, size, image, size);
		// Mat left_img = imread("D:/Jesse/Desktop/subPixelEdgeDetection/image/1_0.bmp");
		frame leftImage(left_img,cameraCapture::LEFT,timestamp);
		glass * g = glass::Instance();
		g->w.showImageAPI(leftImage);
	}
	else if (2 == flag) // right eye
	{
		static cv::Mat right_img(height, width, CV_8UC1);
		memcpy_s(right_img.data, size, image, size);
		// Mat right_img = imread("D:/Jesse/Desktop/subPixelEdgeDetection/image/1_0.bmp");
		frame rightImage(right_img,cameraCapture::RIGHT,timestamp);
		glass * g = glass::Instance();
		g->w.showImageAPI(rightImage);
	}
}

bool glass::Init()
{
	w.show();

	unsigned seed = time(0);
    srand(seed);

    set_sence_image_callback(SceneImageCallback);

	set_eye_image_callback(EyeImageCallback);	

	int ret = API_ASEEGLASSES_7INVENSUN::init("./");
	if (0 != ret) {
		Isinited = false;
		printf("error init: %d\n", ret);
		return false;
	}
	Isinited = true;
    printf("Glass_init: successed!\n");
	return true;
}

bool glass::Start()
{
	if(!Isinited)
	{
		int ret = API_ASEEGLASSES_7INVENSUN::init("./");
		if (0 != ret) {
			Isinited = false;
			printf("error init: %d\n", ret);
			return false;
		}
		Isinited = true;
		printf("Glass_init: successed!\n");
	}
    int ret = API_ASEEGLASSES_7INVENSUN::start();
	if (0 != ret) {
		printf("error start: %d\n", ret);
		API_ASEEGLASSES_7INVENSUN::release();
		return false;
	}
	printf("camera started.\n");
	return true;
}

bool glass::Stop()
{
    int ret = API_ASEEGLASSES_7INVENSUN::stop();
    if (0 != ret) {
		printf("error stop: %d\n", ret);
        API_ASEEGLASSES_7INVENSUN::release();
		return false;
	}
	printf("camera stoped.\n");
	return true;
}

void glass::Release()
{
    int ret = API_ASEEGLASSES_7INVENSUN::release();
	printf("glass released.\n");
}
