cmake_minimum_required(VERSION 2.8)
project(ConvRGBDToONI)

find_package(OpenCV REQUIRED)
find_package(libfreenect REQUIRED)

include_directories (${LIBUSB_INCLUDE_DIRS})
link_directories (${LIBUSB_LIBRARY_DIRS})

FIND_PATH(OPEN_NI_INCLUDE "XnOpenNI.h" "OpenNIConfig.h" HINTS "$ENV{OPEN_NI_INCLUDE}" "/usr/include/ni" "/usr/include/openni" "/opt/ros/groovy/include/openni_camera")
FIND_LIBRARY(OPEN_NI_LIBRARY NAMES OpenNI libOpenNI HINTS $ENV{OPEN_NI_LIB} "/usr/lib")
LINK_DIRECTORIES($ENV{OPEN_NI_LIB})
INCLUDE_DIRECTORIES(${OPEN_NI_INCLUDE})
LINK_LIBRARIES(${OPEN_NI_LIBRARY})

include_directories("${PROJECT_SOURCE_DIR}")

add_executable(ConvRGBDToONI ConvRGBDToONI.cpp)
target_link_libraries(ConvRGBDToONI ${OpenCV_LIBS})
target_link_libraries(ConvRGBDToONI ${OPEN_NI_LIBRARY})
target_link_libraries(ConvRGBDToONI ${FREENECT_LIBRARIES})
