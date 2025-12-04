# 这是 external/dancer-geometry/CMakeLists.txt 的新内容

cmake_minimum_required(VERSION 3.16)
project(dancer_geometry)

# 设置 C++ 标准 (与你的主项目保持一致)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    src/angle.cpp
    src/line_segment.cpp
    src/point.cpp
)
# 我们故意把 src/utils.cpp 排除在外了

# 将这些源文件编译为一个名为 "dancer_geometry" 的库
add_library(dancer_geometry ${SOURCES})

# 设为 PUBLIC，以便主项目可以 #include "dancer_geometry/point.hpp"
target_include_directories(dancer_geometry PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS} # <--- 添加了这一行
)

target_link_libraries(dancer_geometry PUBLIC
    ${OpenCV_LIBS}         # <--- 添加了这一行
)