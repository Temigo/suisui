CMAKE_MINIMUM_REQUIRED(VERSION 2.6)


set(OpenCV_DIR "/usr/local/share/OpenCV")
message("OpenCV_DIR : " ${OpenCV_DIR})

FIND_PACKAGE(OpenCV REQUIRED)



if(OpenCV_FOUND)
   message("Found OpenCV")
   message("Includes: " ${OpenCV_INCLUDE_DIRS})
   message("Libs : " ${OpenCV_LIBS})
   message("Version: " ${OpenCV_VERSION})
endif(OpenCV_FOUND)

PROJECT(suisui)

ADD_EXECUTABLE(suisui main.cpp)
TARGET_LINK_LIBRARIES(suisui ${OpenCV_LIBS})
