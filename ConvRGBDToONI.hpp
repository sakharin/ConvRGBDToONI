#ifndef __CONV_RGBD_TO_ONI_H__
#define __CONV_RGBD_TO_ONI_H__

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// C++
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include <dirent.h>
#include <sys/stat.h>

using namespace cv;
using namespace std;

void getFiles(string dir, vector< string >& files);
#endif
