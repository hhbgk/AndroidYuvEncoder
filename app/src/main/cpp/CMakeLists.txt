# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.
cmake_minimum_required(VERSION 3.4.1)

#ffmpeg so文件路径
set(lib_src_DIR ${CMAKE_SOURCE_DIR}/../jniLibs/${ANDROID_ABI})

#配置加载native依赖
include_directories(${CMAKE_SOURCE_DIR}/include)

#动态方式加载ffmepg的so文件 第三方库加载方式
add_library(ffmpeg SHARED IMPORTED)

#引入libffmpeg.so文件
set_target_properties(ffmpeg PROPERTIES IMPORTED_LOCATION ${lib_src_DIR}/libijkffmpeg.so)

include_directories(${CMAKE_SOURCE_DIR}/inc)

#add_library(x264 SHARED IMPORTED)
#set_target_properties(x264 PROPERTIES IMPORTED_LOCATION ${lib_src_DIR}/libx264.so)

add_library(h264 SHARED IMPORTED)
set_target_properties(h264 PROPERTIES IMPORTED_LOCATION ${lib_src_DIR}/libopenh264.so)

file(GLOB_RECURSE JNI_SRC "*.c" "*.h")

add_library(codec SHARED ${JNI_SRC})

target_link_libraries(codec ffmpeg h264 android log)