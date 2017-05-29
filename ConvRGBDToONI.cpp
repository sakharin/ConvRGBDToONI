#include "ConvRGBDToONI.hpp"

void getFiles(string dir, vector< string >& files) {
  DIR *dp = opendir(dir.c_str());
  if (dp == NULL) {
    cout << "Error opening " << dir << "." << endl;
    return;
  }
  struct dirent *dirp;
  while ((dirp = readdir(dp))) {
    string filepath = dir + "/" + dirp->d_name;

    // If the file is a directory (or is in some way invalid) we'll skip it
    struct stat filestat;
    if (stat( filepath.c_str(), &filestat )) continue;
    if (S_ISDIR( filestat.st_mode ))         continue;

    files.push_back(dirp->d_name);
  }
  closedir(dp);
}

ConvRGBDToONI::ConvRGBDToONI() {
  ret_val_ = XN_STATUS_OK;
}

ConvRGBDToONI::~ConvRGBDToONI() {
  context_.Shutdown();

  depth_generator_.Release();
  image_generator_.Release();

  mock_depth_generator_.Release();
}

int ConvRGBDToONI::initXML(string xml_file) {
  EnumerationErrors errors;
  ret_val_ = context_.InitFromXmlFile(xml_file.c_str(), &errors);
  CHECK_RC(ret_val_, "Init from XML");
  return 1;
}

/*
XnChar ConvRGBDToONI::openInputFile(string input_oni) {
  // Open input file
  Player player;
  ret_val_ = context_.OpenFileRecording(input_oni.c_str(), player);
  CHECK_RC(ret_val_, "Open input file");
  // Play as fast as you can
  ret_val_ = player.SetPlaybackSpeed(XN_PLAYBACK_SPEED_FASTEST);
  CHECK_RC(ret_val_, "Setting playback speed");
  // Set loop
  ret_val_ = player.SetRepeat(FALSE);
  CHECK_RC(ret_val_, "Set loop");
}
*/

int ConvRGBDToONI::initGenerators() {
  NodeInfoList list;
  ret_val_ = context_.EnumerateExistingNodes(list);
  CHECK_RC(ret_val_, "Enumerate nodes");
  for (NodeInfoList::Iterator it = list.Begin(); it != list.End(); ++it) {
    switch ((*it).GetDescription().Type) {
    case XN_NODE_TYPE_IMAGE:
      (*it).GetInstance(image_generator_);
      break;
    case XN_NODE_TYPE_DEPTH:
      (*it).GetInstance(depth_generator_);
      break;
    }
  }

  if ( ( depth_generator_.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT) ) && ( image_generator_.IsValid() )) {
    depth_generator_.GetAlternativeViewPointCap().SetViewPoint(image_generator_);
  } else {
    depth_generator_.GetAlternativeViewPointCap().ResetViewPoint();
  }

  if (depth_generator_.IsCapabilitySupported(XN_CAPABILITY_USER_POSITION))
    cout << "depth capability user" << endl;

  Mat image;
  Mat depth;
  readFrame(image, depth);
  readFrame(image, depth);
  return 1;
}

int ConvRGBDToONI::readFrame(Mat& image, Mat& depth) {
  EnumerationErrors errors;
  ret_val_ = context_.WaitAnyUpdateAll();
  CHECK_RC(ret_val_, "Wait for update");

  if (image_generator_.IsValid()) {
   image_generator_.GetMetaData(image_meta_data_);
  }
  if (depth_generator_.IsValid()) {
    depth_generator_.GetMetaData(depth_meta_data_);
  }

  // Take current image
  XnRGB24Pixel* pImage = const_cast <XnRGB24Pixel*> (image_generator_.GetRGB24ImageMap());

  IplImage *imageAux = cvCreateImageHeader(cvSize(image_meta_data_.XRes(), image_meta_data_.YRes()), 8, 3);
  IplImage *iplImgFrame = cvCreateImage(cvSize(image_meta_data_.XRes(), image_meta_data_.YRes()), 8, 3);

  XnRGB24Pixel* pImageData = const_cast <XnRGB24Pixel*> (pImage);
  cvSetData(imageAux, pImageData, 640*3);
  //cvCvtColor(imageAux, iplImgFrame, CV_RGB2BGR);
  Mat tmp_image = cv::cvarrToMat(imageAux);
  cvtColor(tmp_image, image, CV_RGB2BGR);

  //cvShowImage("RGB Dentro", iplImgFrame);
  //cvShowImage("RGB Dentro Aux", imageAux);
  //waitKey(0);

  // Take current depth map
  XnDepthPixel* pDepthMap = const_cast <XnDepthPixel*> ( depth_generator_.GetDepthMap());

  IplImage *iplDepthFrame = cvCreateImageHeader(cvSize(depth_meta_data_.XRes(), depth_meta_data_.YRes()), 16, 1);

  XnDepthPixel* pDepthData = const_cast <XnDepthPixel*> (pDepthMap);
  cvSetData(iplDepthFrame, pDepthData, 640*2);
  depth = cv::cvarrToMat(iplDepthFrame);

  //cvShowImage("Depth Dentro Aux", iplDepthFrame);
  //waitKey(0);

  cvReleaseImageHeader(&imageAux);
  cvReleaseImage(&iplImgFrame);
  cvReleaseImageHeader(&iplDepthFrame);
  return 1;
}

int ConvRGBDToONI::openOutputFile(string output_oni) {
  NodeInfoList recorder_list;
  ret_val_ = context_.EnumerateProductionTrees(XN_NODE_TYPE_RECORDER, NULL, recorder_list);
  CHECK_RC(ret_val_, "Enumerate prodution tree");

  NodeInfo node_info = *recorder_list.Begin();
  ret_val_ = context_.CreateProductionTree(node_info);
  CHECK_RC(ret_val_, "Create prodution tree");

  recorder_ = new Recorder();
  ret_val_ = node_info.GetInstance(*(recorder_));
  CHECK_RC(ret_val_, "Get instance of node_info");

  ret_val_ = recorder_->SetDestination(XN_RECORD_MEDIUM_FILE, output_oni.c_str());
  CHECK_RC(ret_val_, "Set destination");

  ret_val_ = mock_image_generator_.CreateBasedOn(image_generator_);
  CHECK_RC(ret_val_, "Create mock image");
  if (mock_image_generator_.IsValid()) {
    ret_val_ = recorder_->AddNodeToRecording(mock_image_generator_, XN_CODEC_JPEG);
    CHECK_RC(ret_val_, "Add image node to recorder");
  }

  ret_val_ = mock_depth_generator_.CreateBasedOn(depth_generator_);
  CHECK_RC(ret_val_, "Create mock depth");
  if (mock_depth_generator_.IsValid()) {
    ret_val_ = recorder_->AddNodeToRecording(mock_depth_generator_, XN_CODEC_16Z_EMB_TABLES);
    CHECK_RC(ret_val_, "Add depth node to recorder");
  }
}

int ConvRGBDToONI::writeFrame(Mat image, Mat depth) {
  EnumerationErrors errors;
  ret_val_ = context_.WaitAnyUpdateAll();
  CHECK_RC(ret_val_, "Wait for update");

  if (image_generator_.IsValid()) {
    image_generator_.GetMetaData(image_meta_data_);
  }
  if (depth_generator_.IsValid()) {
    depth_generator_.GetMetaData(depth_meta_data_);
  }
  if (mock_image_generator_.IsValid()) {
    //cout << "Image valid" << endl;
  }
  if (mock_depth_generator_.IsValid()) {
    //cout << "Depth valid" << endl;
  }

  // Process image data
  ret_val_ = image_meta_data_.MakeDataWritable();
  CHECK_RC(ret_val_, "Set image writable");

  // Copy image
  RGB24Map& image_map = image_meta_data_.WritableRGB24Map();
  for (XnUInt32 v = 0; v < image_map.YRes(); v++) {
    for (XnUInt32 u = 0; u < image_map.XRes(); u++) {
      Vec3b pixel = image.at<Vec3b>(v, u);

      XnRGB24Pixel image_pixel;
      image_pixel.nRed   = pixel.val[2];
      image_pixel.nGreen = pixel.val[1];
      image_pixel.nBlue  = pixel.val[0];

      image_map(u, v) = image_pixel;
    }
  }
  ret_val_ = mock_image_generator_.SetData(image_meta_data_);
  CHECK_RC(ret_val_, "Set image data");

  // Process depth data
  ret_val_ = depth_meta_data_.MakeDataWritable();
  CHECK_RC(ret_val_, "Set depth writable");

  // Copy depth
  DepthMap& depth_map = depth_meta_data_.WritableDepthMap();
  for (XnUInt32 v = 0; v < depth_map.YRes(); v++) {
    for (XnUInt32 u = 0; u < depth_map.XRes(); u++) {
      depth_map(u, v) = depth.at<unsigned short>(v, u);
    }
  }
  ret_val_ = mock_depth_generator_.SetData(depth_meta_data_);
  CHECK_RC(ret_val_, "Set depth data");
}

int ConvRGBDToONI::record() {
  ret_val_ = recorder_->Record();
  CHECK_RC(ret_val_, "Record");
}

int ConvRGBDToONI::stopRecord() {
  recorder_->Release();
  delete recorder_;
}

int main( int argc, const char** argv ) {
  // Check parameters
  string xml_file;
  string dir;
  string dummy_input_oni;
  string output_oni;
  int number_of_frames;
  if (argc < 5 || argc > 6) {
    cout << "Usage: " << argv[0] << " XML_file image_foler dummy_input.oni output.oni [number_of_frames]" << endl;
    return -1;
  } else {
    xml_file = argv[1];
    dir = argv[2];
    dummy_input_oni = argv[3];
    output_oni = argv[4];
    if (argc == 6) {
      number_of_frames = atoi(argv[5]);
    } else {
      number_of_frames = 1;
    }
  }

  // Get files in directory
  vector< string > image_files;
  vector< string > depth_files;
  getFiles(dir + "/rgb", image_files);
  getFiles(dir + "/depth", depth_files);
  sort(image_files.begin(), image_files.end());
  sort(depth_files.begin(), depth_files.end());
  number_of_frames = min(number_of_frames, (int)image_files.size());
  number_of_frames = min(number_of_frames, (int)depth_files.size());

  // Init ONI
  ConvRGBDToONI conv;
  conv.initXML(xml_file);
  //conv.openInputFile(dummy_input_oni);
  conv.initGenerators();

  Mat image;
  Mat depth;
  conv.readFrame(image, depth);

  conv.openOutputFile(output_oni);
  int frames = 0;
  while (frames < number_of_frames) {
    cout << "Frame : " << frames << "/" << number_of_frames << "\r" << flush;

    image = imread(dir + "/rgb/" + image_files[frames],
                   CV_LOAD_IMAGE_UNCHANGED);
    depth = imread(dir + "/depth/" + depth_files[frames],
                   CV_LOAD_IMAGE_UNCHANGED);
    imshow("Image", image);
    imshow("Depth", depth);

    conv.writeFrame(image, depth);
    conv.record();

    int key = waitKey(1) & 255;
    if (key == 27)
      break;
    frames++;
  }
  conv.stopRecord();
  return 0;
}
