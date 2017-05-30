# ConvRGBDToONI

Convert RGBD images from [A Large Dataset of Object Scans](http://redwood-data.org/3dscan) to ONI file.

## Requirments
Please follow method of [OpenNI](https://github.com/OpenNI/OpenNI).

## Build
Build using CMake.

## Usage
Usage with a Kinect V1.
  ./ConvRGBDToONI image_foler XML_file output.oni [number_of_frames]

Usage without a Kinect V1.
  ./ConvRGBDToONI image_foler dummy_input.oni output.oni [number_of_frames]

## Author
* **Sakharin Buachan** - *Initial work* - [Sakharin](https://github.com/sakharin)

## License

This project is licensed under the GNU GPLv3 License - see the [LICENSE.md](LICENSE.md) file for details.
