# 这是 external/dtransmit/CMakeLists.txt 的新内容

cmake_minimum_required(VERSION 3.16)
project(dtransmit)

# 设置 C++ 标准 (与你的主项目保持一致)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 dtransmit 所需的依赖
# dtransmit 使用 boost::asio, 它依赖 boost::system
find_package(Boost 1.70 REQUIRED COMPONENTS system)
# dtransmit 使用 std::thread, 它需要 Threads
find_package(Threads REQUIRED)

# 查找所有要编译的 .cpp 源文件
# (这比只列出 dtransmit.cpp 更健壮)
file(GLOB SOURCES "src/*.cpp")

# 将这些源文件编译为一个名为 "dtransmit" 的库
add_library(dtransmit ${SOURCES})

# 设置 PUBLIC 包含目录，以便主项目也能找到它们
target_include_directories(dtransmit PUBLIC
    include # 允许 #include "dtransmit/dtransmit.hpp"
    ${Boost_INCLUDE_DIRS}
)

# 链接 dtransmit 所需的库
target_link_libraries(dtransmit PUBLIC
    Boost::system     # 链接 Boost
    Threads::Threads  # 链接 pthread
)