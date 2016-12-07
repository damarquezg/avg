cmake_minimum_required(VERSION 2.8)
project( mono_vo )
find_package( OpenCV 3.0 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB mono_vo
    "src/*.h"
    "src/*.cc"
)
add_executable( monovo ${mono_vo} )
target_link_libraries( monovo ${OpenCV_LIBS} )
