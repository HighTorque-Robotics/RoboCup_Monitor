#include "dmonitor/util.hpp" // 包含我们刚刚修复的头文件
#include <sys/stat.h>     // 用于 file_exists

namespace dmonitor {
    namespace util {

        /**
         * @brief 获取路径的目录名 (C++ 实现)
         */
        std::string dirname(const std::string& path) {
            size_t last_slash = path.find_last_of('/');
            if (last_slash == std::string::npos) {
                return ".";
            }
            return path.substr(0, last_slash);
        }

        /**
         * @brief 检查文件是否存在 (C++ 实现)
         */
        bool file_exists(const std::string& path) {
            struct stat buffer;
            return (stat(path.c_str(), &buffer) == 0);
        }

    } // namespace util
} // namespace dmonitor