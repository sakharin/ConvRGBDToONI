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

// OpenNI
#include <XnCppWrapper.h>
#include <XnLog.h>

using namespace cv;
using namespace std;
using namespace xn;

#define CHECK_RC(rc, what) \
  if (rc != XN_STATUS_OK) \
  { \
    printf("%s failed: %s\n", what, xnGetStatusString(rc)); \
    return rc; \
  }

class ConvRGBDToONI {
 private:
  Context context_;
  XnStatus ret_val_;

  DepthGenerator depth_generator_;
  ImageGenerator image_generator_;

  DepthMetaData depth_meta_data_;
  ImageMetaData image_meta_data_;

  MockDepthGenerator mock_depth_generator_;
  MockImageGenerator mock_image_generator_;

  Recorder* recorder_;
  Player player_;

 public:
  ConvRGBDToONI();
  ~ConvRGBDToONI();
  int init();
  int initXML(string xml_file);
  int openInputFile(string input_oni);
  int initGenerators();
  int readFrame(Mat& image, Mat& depth);
  int openOutputFile(string output_oni);
  int writeFrame(Mat image, Mat depth, int frame_no);
  int record();
  int stopRecord();
};
#endif
