cmake_minimum_required(VERSION 2.8)
project( DeepSpaceVision )

set( BOOST_ROOT ~/boost_1_69_0 )
set( OpenCV_DIR ~/opencv-2.4.9 )
add_definitions( -std=c++17 )

set( OpenCV_FOUND 1 )

find_package( PkgConfig )
find_package( OpenCV 2.4.9 REQUIRED )
find_package( Boost COMPONENTS system REQUIRED )
pkg_check_modules( GST REQUIRED gstreamer-1.0>=1.4
                               gstreamer-sdp-1.0>=1.4
                               gstreamer-video-1.0>=1.4
                               gstreamer-app-1.0>=1.4 )

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} ${GST_INCLUDE_DIRS} )

add_executable( DeepSpaceVision DeepSpaceVision.cpp UDPHandler.cpp Contour.cpp opencv-code/cap_gstreamer.cpp utility/cam_helper.cpp )
target_link_libraries( DeepSpaceVision ${OpenCV_LIBS} ${Boost_LIBRARIES} ${GST_LIBRARIES}  gstapp-1.0 gstriff-1.0 gstbase-1.0 gstvideo-1.0 gstpbutils-1.0 X11 pthread )

