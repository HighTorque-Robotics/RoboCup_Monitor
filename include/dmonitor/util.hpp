#pragma once

#include <string>

namespace dmonitor {
    namespace util {

        /**
         * @brief 获取路径的目录名 (C++ 实现)
         * @param path 完整路径
         * @return 目录路径
         */
        std::string dirname(const std::string& path);

        /**
         * @brief 检查文件是否存在 (C++ 实现)
         * @param path 文件路径
         * @return 如果存在则为 true
         */
        bool file_exists(const std::string& path);

    } // namespace util
} // namespace dmonitor