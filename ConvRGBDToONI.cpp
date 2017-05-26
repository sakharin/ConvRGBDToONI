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

int main( int argc, const char** argv ) {
  // Check parameters
  string xml_file;
  string dir;
  string output_oni;
  int number_of_frames;
  if (argc < 4 || argc > 5) {
    cout << "Usage: " << argv[0] << " XML_file image_foler output.oni [number_of_frames]" << endl;
    return -1;
  } else {
    xml_file = argv[1];
    dir = argv[2];
    output_oni = argv[3];
    if (argc == 5) {
      number_of_frames = atoi(argv[4]);
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

  for (int i = 0; i < number_of_frames; i++) {
    cout << image_files[i] << endl;
    cout << depth_files[i] << endl;
  }
  return 0;
}
