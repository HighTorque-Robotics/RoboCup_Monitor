# 1. 设置最低 CMake 版本和项目名称
cmake_minimum_required(VERSION 3.16)
project(dmonitor)

# 2. 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 3. 设置编译选项
add_compile_options(-W -Wall -O2)
# add_compile_options(-g) # 用于 Debug

# ----------------------------------------------------
# 4. 查找所有必需的外部依赖包 (来自 apt)
# ----------------------------------------------------

# SFML (用于窗口和图形)
find_package(SFML 2.5 REQUIRED COMPONENTS system window graphics network)

# OpenCV (用于视频 I/O)
find_package(OpenCV REQUIRED)
message(STATUS "Using OpenCV: ${OpenCV_INCLUDE_DIRS}")

# jsoncpp (用于日志/回放)
find_package(jsoncpp REQUIRED)

# fmt (用于日志输出)
find_package(fmt REQUIRED)

# Boost (用于 monitor.cpp 中的 filesystem)
find_package(Boost 1.70 REQUIRED COMPONENTS filesystem)

# Threads (用于 std::thread)
find_package(Threads REQUIRED)

# ----- [!!!] 修复 TCLAP [!!!] -----
# libtclap-dev 在 Ubuntu 上不提供 CMake 配置文件, 所以我们手动查找头文件
find_path(TCLAP_INCLUDE_DIR NAMES tclap/CmdLine.h
          PATHS /usr/include /usr/local/include)

if(NOT TCLAP_INCLUDE_DIR)
    message(FATAL_ERROR "TCLAP header not found. Please run: sudo apt-get install libtclap-dev")
else()
    message(STATUS "Found TCLAP headers at: ${TCLAP_INCLUDE_DIR}")
endif()
# -------------------------------------------

# ----------------------------------------------------
# 5. 添加内部依赖库 (dancer_geometry, dtransmit)
# ----------------------------------------------------
add_subdirectory(external/dancer_geometry)
add_subdirectory(external/dtransmit)

# ----------------------------------------------------
# 6. 编译本项目 (dmonitor)
# ----------------------------------------------------
include_directories(include)

add_library(${PROJECT_NAME}
    src/monitor.cpp
    src/util.cpp
    src/rich_text.cpp
    src/custom_shape.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${SFML_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${JsonCpp_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    # 外部依赖
    sfml-system
    sfml-window
    sfml-graphics
    sfml-network
    ${OpenCV_LIBS}
    jsoncpp_lib
    fmt::fmt
    Boost::filesystem
    Threads::Threads

    # 内部依赖
    dancer_geometry
    dtransmit
)

# 声明可执行文件 (dmonitor_node)
add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
    ${TCLAP_INCLUDE_DIR}
)

# 链接可执行文件
target_link_libraries(${PROJECT_NAME}_node PRIVATE
    ${PROJECT_NAME}
)

# ----------------------------------------------------
# 6.5. 添加 GDAL、NetCDF 和 CURL（修复 undefined reference 问题）
# ----------------------------------------------------
find_package(GDAL)
find_package(NetCDF)
find_package(CURL)

if(GDAL_FOUND)
    message(STATUS "Found GDAL: ${GDAL_LIBRARIES}")
    target_link_libraries(${PROJECT_NAME}_node PRIVATE ${GDAL_LIBRARIES})
else()
    message(WARNING "GDAL not found, linking manually to gdal")
    target_link_libraries(${PROJECT_NAME}_node PRIVATE gdal)
endif()

if(NetCDF_FOUND)
    message(STATUS "Found NetCDF: ${NetCDF_LIBRARIES}")
    target_link_libraries(${PROJECT_NAME}_node PRIVATE ${NetCDF_LIBRARIES})
else()
    message(WARNING "NetCDF not found, linking manually to netcdf")
    target_link_libraries(${PROJECT_NAME}_node PRIVATE netcdf)
endif()

if(CURL_FOUND)
    message(STATUS "Found CURL: ${CURL_LIBRARIES}")
    target_link_libraries(${PROJECT_NAME}_node PRIVATE ${CURL_LIBRARIES})
else()
    message(WARNING "CURL not found, linking manually to curl")
    target_link_libraries(${PROJECT_NAME}_node PRIVATE curl)
endif()

# ----------------------------------------------------
# 7. 复制运行时所需的文件
# ----------------------------------------------------
set(RUNTIME_FILES
    ${PROJECT_SOURCE_DIR}/misc/font.ttf
    # ${PROJECT_SOURCE_DIR}/misc/logo.png
)

add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${RUNTIME_FILES}
    $<TARGET_FILE_DIR:${PROJECT_NAME}_node>
    COMMENT "Copying runtime files to executable directory"
)
