# ConvRGBDToONI

Convert RGBD images from [A Large Dataset of Object Scans](http://redwood-data.org/3dscan) to ONI file.

The project is modified from a source code in a [project page](http://www.uco.es/dptos/aceyte/arquitectura/profesores/jcgamez/?RESEARCH_%28English%29) of Juan Carlos Gámez.

## Requirments
Please follow method of [OpenNI](https://github.com/OpenNI/OpenNI).

## Build
Build using CMake.
```bash
mkdir build
cd build
cmake ..
make
```

## Usage
Usage with a Kinect V1.

```bash
  ./ConvRGBDToONI image_foler XML_file output.oni [number_of_frames]
```

Usage without a Kinect V1.
```bash
  ./ConvRGBDToONI image_foler dummy_input.oni output.oni [number_of_frames]
```

## Author
* **Sakharin Buachan** - *Initial work* - [Sakharin](https://github.com/sakharin)

## License

This project is licensed under the GNU GPLv3 License - see the [LICENSE.md](LICENSE.md) file for details.
