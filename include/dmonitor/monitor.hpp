/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * \file monitor.hpp
 * \author Yusu Pan
 * \version 2018
 * \date 2018-06-07
 */

#pragma once

#include <json/json.h>
// #include <ros/ros.h> // 修改：移除 ROS 核心头文件
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <chrono> // 修改：添加 <chrono> 以支持 dmsgs.hpp 中的时间类型

#include "dancer_geometry/point.hpp"
#include "dmonitor/rich_text.hpp"
// #include "dmsgs/TeamInfo.h" // 修改：移除 ROS 消息头文件
#include "dmonitor/dmsgs.hpp" // 修改：添加自定义的、非 ROS 的消息定义文件
#include "dtransmit/dtransmit.hpp"
#include <opencv2/opencv.hpp>

namespace dmonitor {
class Monitor {
 public:
  explicit Monitor(const std::string &binary_path, const int &camera_device,
                   const std::string &log_path, bool quiet_mode);
  ~Monitor();

  void Start();

 private:
  /**
   * @brief Convert global position to screen coordinate
   * @param in
   */
  void GlobalToScreen(sf::Vector2f &in);

  /**
   * @brief Draw a line between two gievn points
   * @param window - SFML window instance
   * @param p1 - point 1
   * @param p2 - point 2
   */
  void DrawFieldLine(const sf::Vector2f &p1, const sf::Vector2f &p2);

  /**
   * @brief Draw the whole field for RoboCup Humanoid Kid-size
   * @param window - SFML window instance
   */
  void DrawField();

  /**
   * @brief Get color by robot id
   * @param id - robot id
   * @return robot color
   */
  sf::Color GetColor(const int &id);

  /**
   * @brief Draw text at given position with color by given robot id
   * @param window - SFML window instance
   * @param text - text to display
   * @param pos - text position
   * @param id - robot that text belongs to
   */
  void DrawText(sfe::RichText &text, const sf::Vector2f &pos, const int &id);

  /**
   * @brief Draw text at given position with color by given robot id
   * @param window - SFML window instance
   * @param str - text to display
   * @param pos - text position
   * @param id - robot that text belongs to
   */
  void DrawText(const std::string &str, const sf::Vector2f &pos, const int &id);

  /**
   * @brief Draw a ball at given position for given robot
   * @param window - SFML window instance
   * @param pos - ball position
   * @param id - ball that text belongs to
   */
  void DrawBall(const sf::Vector2f &pos, const int &id);

  /**
   * @brief Draw a ball with given pose
   * @param window - SFML window instance
   * @param pos - ball position
   * @param id - robot id
   */
  void DrawCircle(const sf::Vector2f &pos, const int &id);

  /**
   * @brief Draw a goal with given pose
   * @param window - SFML window instance
   * @param pos - goal position
   * @param id - robot id
   */
  void DrawGoal(const sf::Vector2f &pos, const int &id);

  /**
   * @brief Draw a circle with given pose
   * @param window - SFML window instance
   * @param pos - circiel position
   * @param id - robot id
   */
  void DrawPlayer(const sf::Vector2f &pos, double yaw, const int &id);

  /**
   * @brief Draw a line between two points with given colro and thickness
   * @param window - SFML window instance
   * @param from - beginning point
   * @param to - ending point
   * @param id - robot id
   * @param thickness - line thickness
   */
  void DrawLine(const sf::Vector2f &from, const sf::Vector2f &to, const int &id,
                double thickness = 2);
  /**
   * @brief Draw a dashed line between two given points
   * @param window - SFML window instance
   * @param pt1 - point 1
   * @param pt2 - point 2
   * @param id - robot id
   */
  void DrawDashedLine(dancer_geometry::Point pt1, dancer_geometry::Point pt2,
                      const int &id);

  /**
   * @brief Draw a local and final destination of robot walking scheduler
   * @param window - SFML window instance
   * @param pos - robot position
   * @param dest - local destination
   * @param final_dest - final destination
   * @param id - robot id
   */
  void DrawTarget(const sf::Vector2f &pos, const sf::Vector2f &dest,
                  const sf::Vector2f &final_dest, const int &id);

  /**
   * @brief Draw the attacking target of robot
   * @param window - SFML window instance
   * @param pos - robot position
   * @param attack_target - attack target
   * @param id - robot id
   */
  void DrawAttackTarget(const sf::Vector2f &pos,
                        const sf::Vector2f &attack_target, const int &id);

  /**
   * Read and load from given opened file
   * log and fill given data structure.
   * Return false on file end.
   */
  bool LoadReplayLine(std::ifstream &replay,
                      std::map<size_t, dmsgs::TeamInfo> &allInfo, // 修改：这里的 dmsgs::TeamInfo 现在指向 dmsgs.hpp 中的 struct
                      double *replayTimePtr = nullptr,
                      size_t *framePtr = nullptr);

  Json::Value TeamInfoToJson(const dmsgs::TeamInfo &info); // 修改：这里的 dmsgs::TeamInfo 现在指向 dmsgs.hpp 中的 struct
  void TeamInfofromJson(dmsgs::TeamInfo &info, const Json::Value &json_value); // 修改：这里的 dmsgs::TeamInfo 现在指向 dmsgs.hpp 中的 struct

  // Display window
  unsigned int window_width_ = 1280;
  unsigned int window_height_;
  double window_ratio = 16.0 / 10.0;
  sf::RenderWindow *window_;

  uint8_t global_alpha_ = 255;
  sf::Font font_;

  // Flags
  bool is_inverted_ = false; // [修改后] 改为 bool 类型，默认为 false (不反转)
  bool is_update_ = false;
  bool is_replay_ = false;
  bool is_stopped_ = false;

  // Camera
  int camera_device_ = 0;
  size_t idx_frame_curr_ = 0;
  size_t idx_frame_last_ = 0;
  bool has_new_frame_ = false;
  std::mutex frame_mutex_;
  std::thread *thread_capture_ = nullptr;
  std::thread *thread_show_ = nullptr;

  // Logging
  std::string log_path_;
  std::ofstream log_;
  bool is_quiet_mode_;
  int vedio_log=5;
  cv::VideoWriter outputVideo;
  std::string outputVideoPath;
  cv::VideoCapture video;
  cv::Mat frame_to_show;

  // Replay
  std::vector<std::map<size_t, dmsgs::TeamInfo>> replay_container_info_; // 修改：这里的 dmsgs::TeamInfo 现在指向 dmsgs.hpp 中的 struct
  std::vector<size_t> replay_container_frame_;
  std::vector<double> replay_container_time_;
  double replay_frame_rate = 0;
  double time_replay_ = 0;
  double time_replay_target_ = 0;
  double time_replay_start_ = 0;
  double time_replay_end_ = 0;

  // Replay user control
  size_t idx_replay_ = 0;
  bool is_replay_paused_ = false;
  bool is_replay_fast_ = false;
  bool is_replay_super_fast_ = false;
  bool is_replay_backward_ = false;

  /**
   * @brief Transmitter for receiving TeamInfo through UDP
   */
  dtransmit::DTransmit *transmitter_;
  std::map<size_t, dmsgs::TeamInfo> robot_info_; // 修改：这里的 dmsgs::TeamInfo 现在指向 dmsgs.hpp 中的 struct
};
}  // namespace dmonitor