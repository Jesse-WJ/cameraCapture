cmake_minimum_required(VERSION 3.5) # CMake install : https://cmake.org/download/
project(cameraCapture LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH "D:/software/Qt/6.4.0/mingw_64") # Qt Kit Dir
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


find_package(Qt6 REQUIRED COMPONENTS Core Gui OpenGL OpenGLWidgets Widgets) # Qt COMPONENTS
aux_source_directory(./src srcs)
file( GLOB headers  include/*.ui include/*.h)
file( GLOB mylibs  bin/*.dll bin/*.lib)
include_directories(include)

set(OpenCV_DIR  "D:/software/OpenCV/build/install")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(CMinpack REQUIRED)
include_directories(${CMinpack_INCLUDE_DIRS})

# Specify MSVC UTF-8 encoding   
# add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
# add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

qt_add_executable(${PROJECT_NAME}
    # WIN32 # If you need a terminal for debug, please comment this statement 
    ${srcs}
    ${headers}  
) 
target_link_libraries(${PROJECT_NAME} PUBLIC
    Qt::Core
    Qt::Gui
    Qt::OpenGL
    Qt::OpenGLWidgets
    Qt::Widgets
    ${OpenCV_LIBS} 
    cminpack::cminpack 
    ${mylibs}
    ${CMAKE_PREFIX_PATH}/bin/opengl32sw.dll
) # Qt6 Shared Library