#include "dmonitor/monitor.hpp"
#include "dmonitor/util.hpp"
#include "dmonitor/dconstant.hpp"
// #include "dmsgs/BehaviorInfo.h" // 修改：移除ROS自动生成的消息头文件
// #include "dmsgs/GCInfo.h" // 修改：移除ROS自动生成的消息头文件
#include "dmonitor/dmsgs.hpp" // 修改：添加我们自定义的、与ROS无关的消息定义头文件

#include <fmt/core.h>
#include <boost/filesystem.hpp>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <chrono> // 修改：添加 <chrono> 用于替换 ros::Time
#include <thread> // 修改：添加 <thread> 用于替换 ros::Rate
// --- [新增] 1. 引入必要的头文件 ---
#include <unordered_set>
#include <cmath> //用于 fmod 等数学运算

// --- [新增] 2. 定义辅助函数 (放在 namespace dmonitor 之前) ---

// 辅助函数：翻转单个 Vector3 (x, y 取反，z 旋转 180度)
// 注意：这里假设你的 dmsgs 使用的是 geometry_msgs/Vector3 或具有类似 x,y,z 结构的类型
template <typename T>
void mirrorVector3(T& vec) {
    vec.x *= -1;
    vec.y *= -1;
    vec.z += 180.0;
    // 将角度归一化到 [-180, 180]
    while (vec.z > 180.0) vec.z -= 360.0;
    while (vec.z < -180.0) vec.z += 360.0;
}

// 辅助函数：翻转 TeamInfo 中所有的空间坐标
void mirrorAllVec3s(dmsgs::TeamInfo& info) {
    mirrorVector3(info.dest);          // 目标点
    mirrorVector3(info.final_dest);    // 最终目标点
    mirrorVector3(info.attack_target); // 攻击点
    mirrorVector3(info.robot_pos);     // 机器人自身定位
    mirrorVector3(info.ball_global);   // 球的全局坐标
    mirrorVector3(info.circle_global); // 中圈坐标
    mirrorVector3(info.goal_global);   // 球门坐标
}

// 辅助函数：判断机器人是否属于“左侧队伍”
// 你需要根据你的实际情况修改这些 ID
bool inLeftTeam(int robot_id) {
    static const std::unordered_set<int> left_team_ids = {1, 2, 3, 4, 5, 6};
    return left_team_ids.find(robot_id) != left_team_ids.end();
}

namespace dmonitor {

// 修改：添加辅助函数以替换 ros::Time <-> double 转换
/**
 * @brief 将 std::chrono::time_point 转换为 double (秒)
 */
double to_sec(const StdTimePoint& time_point) {
    return std::chrono::duration<double>(time_point.time_since_epoch()).count();
}

/**
 * @brief 将 double (秒) 转换为 std::chrono::time_point
 */
StdTimePoint from_sec(double seconds) {
    auto duration_in_sec = std::chrono::duration<double>(seconds);
    return StdTimePoint(std::chrono::duration_cast<StdTimePoint::duration>(duration_in_sec));
}


Monitor::Monitor(const std::string &binary_path, const int &camera_device,
                 const std::string &log_path, bool quiet_mode)
    : camera_device_(camera_device),
      log_path_(log_path),
      is_quiet_mode_(quiet_mode) {
  // Load font and logo
  std::string binary_dir = dmonitor::util::dirname(binary_path);
  fmt::print(stdout, "Loading font from {}\n", binary_dir); // 修改：ROS_INFO("Loading font from %s", binary_dir.c_str());
  std::string font_path = binary_dir + "/font.ttf";
  if (!font_.loadFromFile(font_path)) {
    throw std::logic_error("failed to load font");
  }
  std::string img_path = binary_path + "logo.png";

  // Create window
  window_height_ = static_cast<unsigned int>(window_width_ / window_ratio);
  fmt::print(stdout, "Window size ({}, {})\n", window_width_, window_height_); // 修改：ROS_INFO("Window size (%d, %d)", window_width_, window_height_);
  sf::ContextSettings settings;
  settings.antialiasingLevel = 8;
  sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
  window_ = new sf::RenderWindow(
      sf::VideoMode(window_width_, window_height_, desktop.bitsPerPixel),
      "DMonitor", sf::Style::Default, settings);

  // UDP transmit
  transmitter_ = new dtransmit::DTransmit();
  transmitter_->addRawRecv(
      dconstant::network::TeamInfoBroadcastAddress,
      [this](void *buffer, std::size_t size) {
        // ========== 【新增】调试打印 ==========
        // 只要收到任何数据包，就打印出来
        std::cout << "[DEBUG] Received packet! Size: " << size << std::endl; 
        // ====================================

        if (!is_replay_ && size == sizeof(dmsgs::TeamInfo)) {
          dmsgs::TeamInfo team_info = *(dmsgs::TeamInfo *)buffer;
          team_info.recv_timestamp = std::chrono::system_clock::now(); 
          
          // ========== 【新增】如果大小匹配，确认解析成功 ==========
          std::cout << "[DEBUG] Valid TeamInfo received! ID: " 
                    << (int)team_info.player_number << std::endl;
          // ===================================================
        } else {
             // 如果大小不对，打印出来看看是多少
             if(size != sizeof(dmsgs::TeamInfo)) {
                 std::cout << "[DEBUG] Size mismatch! Expected: " << sizeof(dmsgs::TeamInfo) 
                           << " Got: " << size << std::endl;
             }
        }
      
        if (!is_replay_ && size == sizeof(dmsgs::TeamInfo)) {
          dmsgs::TeamInfo team_info = *(dmsgs::TeamInfo *)buffer;
          team_info.recv_timestamp = std::chrono::system_clock::now(); // 修改：team_info.recv_timestamp = ros::Time::now();
          robot_info_[team_info.player_number] = team_info;  // 或者 team_info.player_number - 1
            
          is_update_ = true;  // 确保这行也存在。
        }
      });
  transmitter_->startService();

  // Initialize log path
  if (quiet_mode) {
    fmt::print(stdout, "Running in QUIET mode\n"); // 修改：ROS_INFO("Running in QUIET mode");
  } else if (log_path_.empty()) {
    std::stringstream ss;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    ss << std::string(std::getenv("HOME")) << "/monitor-logs/"
       << std::put_time(&tm, "%Y%m%d%H%M%S");
    log_path_ = ss.str();
    
    // prepare output directory
    boost::filesystem::path dir(log_path_.c_str());
    if (boost::filesystem::create_directories(dir)) {
      fmt::print(stdout, "Directory created for logs: {}\n", log_path_); // 修改：ROS_INFO("Directory created for logs: %s", log_path_.c_str());
    }
  } else if (!boost::filesystem::exists(log_path_)) {
    fmt::print(stderr, "Log directory does not exist: {}\n", log_path_); // 修改：ROS_ERROR("Log directory does not exist: %s", log_path_.c_str());
    std::terminate();
  } else {
    is_replay_ = true;
    fmt::print(stdout, "Running in REPLAY mode\n"); // 修改：ROS_INFO("Running in REPLAY mode");
  }

  // Load replay
  std::string logFilename = log_path_ + "/monitoring.json";
  outputVideoPath = log_path_+"log_video.avi";
  if (is_replay_) {
    //replat_mode
    fmt::print(stdout, "Loading replay from {}\n", logFilename.c_str()); // 修改：ROS_INFO("Loading replay from %s", logFilename.c_str());
    std::ifstream replayFile;
    replayFile.open(logFilename);
    if (dmonitor::util::file_exists(outputVideoPath)) {
    video.open(outputVideoPath);
      if (!video.isOpened()) {
        // 视频文件存在，但无法打开
        std::cerr << "log video found but failed to open. Continuing without video." << std::endl;
    }
   }else {
    // 视频文件不存在
    std::cerr << "log video not found. Continuing without video." << std::endl;
   }
    
    while (true) {
      std::map<size_t, dmsgs::TeamInfo> tmpInfo;
      double tmpTime;
      size_t tmpFrame;
      bool isOk = LoadReplayLine(replayFile, tmpInfo, &tmpTime, &tmpFrame);
      // End of replay
      if (!isOk) {
        break;
      } else {
        replay_container_info_.push_back(tmpInfo);
        replay_container_time_.push_back(tmpTime);
        replay_container_frame_.push_back(tmpFrame);
      }
    }
    replayFile.close();
    fmt::print(stdout, "Log length: {}\n", replay_container_info_.size()); // 修改：ROS_INFO("Log length: %lu", replay_container_info_.size());
    // initialize
    time_replay_start_ = replay_container_time_[0];
    time_replay_end_ =
        replay_container_time_[replay_container_time_.size() - 1];
    time_replay_ = time_replay_target_ = time_replay_start_;
  } else if (!is_quiet_mode_) {
    // Open log file
    std::ifstream ifs(logFilename);
    if (ifs.good()) {
      std::cerr
          << "File '" << logFilename
          << "' already exists! Erase it before if you want to start a new log."
          << std::endl;
      exit(EXIT_FAILURE);
    }
    fmt::print(stdout, "Writing log to {}\n", logFilename.c_str()); // 修改：ROS_INFO("Writing log to %s", logFilename.c_str());
    log_.open(logFilename);
    //log the vediofile
    fmt::print(stdout, "Writing video to {}\n", logFilename.c_str()); // 修改：ROS_INFO("Writing video to %s", logFilename.c_str());
    
    //outputVideo.open(outputVideoPath,CV_FOURCC('D','I','V','X'),30.0,cv::Size(640,480));
    // // cv::Size S = cv::Size((int)capture0.get(CV_CAP_PROP_FRAME_WIDTH),
    // //                       (int)capture0.get(CV_CAP_PROP_FRAME_HEIGHT));

    //open the video writer to write the video
    outputVideo.open(outputVideoPath,cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),30.0,cv::Size(640,480));
    
    // Running the capture thread
    if (camera_device_ >= 0) {
      thread_capture_ = new std::thread([&]() {
        size_t cnt_frames = 0;
        if (camera_device_ >= 0) {
          fmt::print(stdout, "Capturing on camera #{}\n", camera_device_); // 修改：ROS_INFO("Capturing on camera #%d", camera_device_);
          cv::VideoCapture cap(camera_device_);
          //set the capture frame rate for the camera
          cap.set(cv::CAP_PROP_FPS, 30);
          cv::Mat frame;
  
          auto last = std::chrono::system_clock::now(); // 修改：auto last = ros::Time::now();
          while (!is_stopped_) {
            if (cap.isOpened()) {
              cnt_frames++;
              cap >> frame;
              outputVideo.write(frame);
              if (!frame.empty()) {
                auto frameTs = std::chrono::system_clock::now(); // 修改：auto frameTs = ros::Time::now();
                // save frames to log dir
                  // 修改：使用 std::chrono 计算时间差（纳秒）
                auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(frameTs - last).count();
                if (duration_ns > 33000000) { // 修改：if (frameTs.toNSec() - last.toNSec() > 33000000)
                  // update timestamp
                  last = frameTs;
                  idx_frame_last_ = cnt_frames;
        
                  // flag that a new frame exists
                  frame_mutex_.lock();
                  has_new_frame_ = true;
                  frame_to_show = frame;
                  frame_mutex_.unlock();
                }
              } else {
                // release the video capture and retry
                cap.release();
              }
            } else {
              std::this_thread::sleep_for(std::chrono::milliseconds(500));
              // re-open the camera
              if (camera_device_ >= 0) {
                cap.open(camera_device_);
              }
            }
          }
        }
      });
    }
  }

  // Running the show thread
  thread_show_ = new std::thread([&]() {
    size_t cnt_frame = 0;

    while (!is_stopped_) {
      if (idx_frame_curr_ && cnt_frame != idx_frame_curr_) {
        cnt_frame = idx_frame_curr_;
        if(!is_replay_){
          try {
            cv::namedWindow("Frames", cv::WINDOW_NORMAL);
            frame_mutex_.lock();
            cv::imshow("Frames", frame_to_show);
            frame_mutex_.unlock();
          } catch (cv::Exception &) {
            std::cerr<<"Some errors happen when show the current frame"<<std::endl;
            //std::cerr << "Can't read " << ss.str() << std::endl;
          }
        }
        else{
          if (video.isOpened()){
          //replay mode
          std::cout<<"reading from frame"<<cnt_frame<<std::endl;
          //set the start frame to current frame
          video.set( cv::CAP_PROP_POS_FRAMES,cnt_frame);
          video>>frame_to_show;
          cv::namedWindow("Frames", cv::WINDOW_NORMAL);
          cv::imshow("Frames", frame_to_show);
          }
        }
      }
      cv::waitKey(int(1000/30));
    }
  });
}

Monitor::~Monitor() {
  delete window_;
  delete transmitter_;
  delete thread_capture_;
  delete thread_show_;
}

void Monitor::Start() {
  // Set to default value
  is_update_ = false;

  // Main loop
  // ros::Rate r(30); // 修改：移除 ros::Rate
  while (window_->isOpen()) { // 修改：移除 ros::ok()
    if (!is_replay_) {
      frame_mutex_.lock();
      if (has_new_frame_) {
        idx_frame_curr_ = idx_frame_last_;
        has_new_frame_ = false;
        is_update_ = true;
      }
      frame_mutex_.unlock();
    } else {
      int speed = 50;
      if (!is_replay_paused_ && idx_replay_ < replay_container_info_.size()) {
        double sign = 1;
        if (is_replay_backward_) {
          sign = -1;
        }

        if (is_replay_super_fast_) {
          time_replay_target_ += sign * speed * 20;
        } else if (is_replay_fast_) {
          time_replay_target_ += sign * speed * 4;
        } else {
          time_replay_target_ += sign * speed;
        }
        if (time_replay_target_ < time_replay_start_)
          time_replay_target_ = time_replay_start_;
        if (time_replay_target_ > time_replay_end_)
          time_replay_target_ = time_replay_end_;
        while (time_replay_ < time_replay_target_ &&
               idx_replay_ < replay_container_time_.size() - 1) {
          robot_info_ = replay_container_info_[idx_replay_];
          time_replay_ = replay_container_time_[idx_replay_];
          idx_frame_curr_ = replay_container_frame_[idx_replay_];
          idx_replay_++;
        }
        while (time_replay_ > time_replay_target_ && idx_replay_ > 0) {
          robot_info_ = replay_container_info_[idx_replay_];
          time_replay_ = replay_container_time_[idx_replay_];
          idx_frame_curr_ = replay_container_frame_[idx_replay_];
          idx_replay_--;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(speed));
    }

    // Handle events
    sf::Event event;
    while (window_->pollEvent(event)) {
      // Quit events
      if (event.type == sf::Event::Closed) {
        window_->close();
      }
      if (event.type == sf::Event::KeyPressed &&
          event.key.code == sf::Keyboard::Escape) {
        outputVideo.release();
        //std::cout<<vedio_log<<std::endl;
        window_->close();
      }
      // Invert field event space
      if (event.type == sf::Event::KeyPressed &&
          event.key.code == sf::Keyboard::Tab) {
        is_inverted_ = !is_inverted_; // [修改后] 布尔取反
      }
    }

    // Replay user control
    if (is_replay_) {
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
        is_replay_paused_ = !is_replay_paused_;
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
      }
      is_replay_fast_ = sf::Keyboard::isKeyPressed(sf::Keyboard::F);
      is_replay_super_fast_ = sf::Keyboard::isKeyPressed(sf::Keyboard::S);
      is_replay_backward_ = sf::Keyboard::isKeyPressed(sf::Keyboard::B);
    }

    // Start rendering
    window_->clear(GetColor(-1));

    // //Draw logo
    // sf::Sprite sprite(logo);
    // sprite.setColor(sf::Color(255, 255, 255, 100));
    // sprite.setOrigin(sf::Vector2f(773 / 2.0, 960 / 2.0));
    // sprite.move(isInverted * -2.1, 0.2);
    // sprite.scale(0.0035, 0.0035);
    // window.draw(sprite, sf::RenderStates::Default);

    // Draw field
    DrawField();

    // Logging
    Json::Value json(Json::objectValue);
    if (!is_replay_ && is_update_) {
      // log timestamp in ms
      // 修改：使用 std::chrono 获取毫秒时间戳
      auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      json["ts"] = (unsigned int)now_ms; // 修改：json["ts"] = (unsigned int)(ros::Time::now().toNSec() / 1000000);
      json["frame"] = (unsigned int)idx_frame_curr_;
      json["info"] = Json::arrayValue;
    }

    // Draw players info
    size_t robot_index = 0;
    std::vector<size_t> robot_outdated;
    for (const auto &it : robot_info_) {
      auto info = it.second;
      auto id = info.player_number;
      // ============ 【新增】在这里添加镜像逻辑 ============
      // 这样无论是实时数据还是回放数据，画出来之前都会被处理
      bool need_mirror = is_inverted_ ^ inLeftTeam(id);
      if (need_mirror) {
          mirrorAllVec3s(info);
      }
      // ================================================
      
      // Log data
      if (!is_replay_ && is_update_) {
        json["info"].append(TeamInfoToJson(info));
      }

      // get robot info
      double yaw = info.robot_pos.z;
      sf::Vector2f robot_pos(info.robot_pos.x, info.robot_pos.y);
      sf::Vector2f ball_pos(info.ball_global.x, info.ball_global.y);

      // draw robot player
      DrawPlayer(sf::Vector2f(robot_pos.x, robot_pos.y), yaw, id);

      // draw ball
      if (info.see_ball) {
        if (info.state != dmsgs::TeamInfo::BALL_HANDLING) {
          global_alpha_ = 100;
        }
        DrawBall(ball_pos, id);
        std::stringstream ssBall;
        ssBall << std::fixed << std::setprecision(2) << id;
        DrawText(ssBall.str(), ball_pos - sf::Vector2f(0, 35), id);
        global_alpha_ = 255;
      }

      // Draw circle
      sf::Vector2f circle_pos(info.circle_global.x, info.circle_global.y);
      if (info.see_circle) {
        DrawCircle(circle_pos, id);
        std::stringstream ssCircie;
        ssCircie << std::fixed << std::setprecision(2) << id;
        DrawText(ssCircie.str(), circle_pos - sf::Vector2f(0, 35), id);
        global_alpha_ = 255;
      }


      // draw goal
      if (info.see_goal) {
        sf::Vector2f goal_pos;
        goal_pos = sf::Vector2f(info.goal_global.x, info.goal_global.y);
        DrawGoal(goal_pos, id);
        std::stringstream ssGoal;
        ssGoal << std::fixed << std::setprecision(2) << id;
        DrawText(ssGoal.str(), goal_pos - sf::Vector2f(0, 35), id);
        global_alpha_ = 255;
      }

      // draw placing target
      if (info.state == dmsgs::TeamInfo::PLACING || info.state == dmsgs::TeamInfo::ASSISTING) {
        DrawTarget(sf::Vector2f(robot_pos.x, robot_pos.y),
                   sf::Vector2f(info.dest.x, info.dest.y),
                   sf::Vector2f(info.final_dest.x, info.final_dest.y), id);

        //        if (info.state == BallHandling || info.state == Playing) {
        //          if (std::string(info.statePlaying) == "approach" ||
        //              std::string(info.statePlaying) == "walkBall" ||
        //              std::string(info.statePlaying) == "letPlay") {
        //            sf::Vector2f ballTarget(info.ballTargetX * isInverted,
        //            info.ballTargetY * isInverted);
        //            drawBallArrow(window, ballPos, ballTarget, id);
        //          }
        //        }
      } else if (info.state == dmsgs::TeamInfo::BALL_HANDLING || info.state == dmsgs::TeamInfo::PLACING) {
        DrawAttackTarget(
            sf::Vector2f(robot_pos.x, robot_pos.y),
            sf::Vector2f(info.attack_target.x, info.attack_target.y), id);
      } else if (!(abs(info.dest.x - 0) < 0.00001 && abs(info.dest.y - 0) < 0.00001)) {
        DrawTarget(sf::Vector2f(robot_pos.x, robot_pos.y),
                   sf::Vector2f(info.dest.x, info.dest.y),
                   sf::Vector2f(info.final_dest.x, info.final_dest.y), id);
      }

      // Print information
      sfe::RichText text(font_);
      text << GetColor(id);
      // robot id
      text << sf::Text::Bold;
      text << fmt::format("ID - {}\n", id);
      text << sf::Text::Regular;
      // role info
      text << fmt::format("Role: ");
      if (info.role == dmsgs::BehaviorInfo::ROLE_STRIKER) {
        text << fmt::format("Striker");
      } else if (info.role == dmsgs::BehaviorInfo::ROLE_DEFENDER) {
        text << fmt::format("Defender");
      } else if (info.role == dmsgs::BehaviorInfo::ROLE_SUPPORTER) {
        text << fmt::format("Supporter");
      } else if (info.role == dmsgs::BehaviorInfo::ROLE_GOALIE) {
        text << fmt::format("Goalie");
      } else if (info.role == dmsgs::BehaviorInfo::ROLE_OTHER) {
        text << fmt::format("Other");
      } else {
        text << fmt::format("Unknown");
      }
      text << "\n";
      // GC info
      text << fmt::format("GC: ");
      if (!info.gc_connected) {
        text << fmt::format("Not connected\n");
      } else {
        if (info.gc_state == dmsgs::GCInfo::STATE_INITIAL) {
          text << fmt::format("Initial");
        } else if (info.gc_state == dmsgs::GCInfo::STATE_READY) {
          text << fmt::format("Ready");
        } else if (info.gc_state == dmsgs::GCInfo::STATE_SET) {
          text << fmt::format("Set");
        } else if (info.gc_state == dmsgs::GCInfo::STATE_PLAYING) {
          text << fmt::format("Playing");
        } else if (info.gc_state == dmsgs::GCInfo::STATE_FINISHED) {
          text << fmt::format("Finished");
        } else {
          text << fmt::format("Unknown");
        }
        text << " - ";
        if (info.gc_state2 == dmsgs::GCInfo::STATE2_NORMAL) {
          text << fmt::format("Normal");
        } else if (info.gc_state2 == dmsgs::GCInfo::STATE2_DIRECT_FREEKICK ||
                   info.gc_state2 == dmsgs::GCInfo::STATE2_INDIRECT_FREEKICK) {
          text << fmt::format("Free Kick");
        } else if (info.gc_state2 == dmsgs::GCInfo::STATE2_CORNER_KICK) {
          text << fmt::format("Corner Kick");
        } else if (info.gc_state2 == dmsgs::GCInfo::STATE2_GOAL_KICK) {
          text << fmt::format("Goal Kick");
        } else if (info.gc_state2 == dmsgs::GCInfo::STATE2_THROW_IN) {
          text << fmt::format("Throw In");
        } else if (info.gc_state2 == dmsgs::GCInfo::STATE2_PENALTYKICK) {
          text << fmt::format("Penalty Kick");
        } else if (info.gc_state2 == dmsgs::GCInfo::STATE2_PENALTYSHOOT) {
          text << fmt::format("Penalty Shoot");
        } else if (info.gc_state2 == dmsgs::GCInfo::STATE2_OVERTIME) {
          text << fmt::format("Overtime");
        } else if (info.gc_state2 == dmsgs::GCInfo::STATE2_TIMEOUT) {
          text << fmt::format("Timeout");
        } else {
          text << fmt::format("Unknown");
        }
        text << "\n";
      }
      // robot state
      text << sf::Text::Regular;
      text << "State: ";
      if (info.state == dmsgs::TeamInfo::INACTIVE) {
        text << "Inactive";
      } else if (info.state == dmsgs::TeamInfo::PLAYING) {
        text << "Playing";
      } else if (info.state == dmsgs::TeamInfo::BALL_HANDLING) {
        text << "BallHandling";
      } else if (info.state == dmsgs::TeamInfo::PLACING) {
        text << "Placing";
      } else if (info.state == dmsgs::TeamInfo::SEARCHING) {
        text << "Searching";
      } else if (info.state == dmsgs::TeamInfo::INITIALIZING) {
        text << "Initializing";
      }
      text << "\n";

      // other info
      text << fmt::format("BallQ: {:.2f}\n", info.ball_quality);
      text << fmt::format("FieldQ: {:.2f}\n", info.field_quality);
      text << fmt::format("FieldC: {:.2f}\n", info.field_consistency);

      // timestamp
      double age;
      if (!is_replay_) {
        // 修改：使用 std::chrono 计算时间差
        age = std::chrono::duration<double>(std::chrono::system_clock::now() - info.recv_timestamp).count(); // 修改：age = (ros::Time::now() - info.recv_timestamp).toSec();
      } else {
        // 修改：使用辅助函数 to_sec 转换
        age = (time_replay_ / 1000.0 - to_sec(info.recv_timestamp)); // 修改：age = (time_replay_ / 1000.0 - info.recv_timestamp.toSec());
      }
      if (age > 5.0) {
        text << sf::Color(170, 63, 72, global_alpha_);
        text << fmt::format("Outdated: {:.2f} s\n", age);
        text << GetColor(id);
      }
      if (age > 20) {
        robot_outdated.push_back(it.first);
      }

      // incapacitated
      if (info.incapacitated) {
        text << sf::Color(170, 63, 72, global_alpha_);
        text << fmt::format("incapacitated", age);
        text << GetColor(id);
      }

      // draw text
      switch (robot_index) {
        case 0:
          DrawText(text, sf::Vector2f(-600, 300), id);
          break;
        case 1:
          DrawText(text, sf::Vector2f(-600, -170), id);
          break;
        case 2:
          DrawText(text, sf::Vector2f(490, 300), id);
          break;
        case 3:
          DrawText(text, sf::Vector2f(490, -170), id);
          break;
        default:
          // not to draw
          break;
      }

      robot_index++;
    }

    for (const auto &id : robot_outdated) {
      robot_info_.erase(id);
    }

    if (is_replay_) {
      sfe::RichText ssTime(font_);
      ssTime << GetColor(6);
      ssTime << fmt::format("Time: {:.2f}s",
                            (time_replay_ - time_replay_start_) / 1000.0);
      if (is_replay_fast_) {
        ssTime << "\nSpeed: Fast";
      } else if (is_replay_super_fast_) {
        ssTime << "\nSpeed: SuperFast";
      } else {
        ssTime << "\nSpeed: Normal";
      }
      if (is_replay_backward_) {
        ssTime << "\nDirection: Backward";
      } else {
        ssTime << "\nDirection: Forward";
      }
      if (is_replay_paused_) {
        ssTime << GetColor(1) << "\nPaused";
      }
      DrawText(ssTime, sf::Vector2f(-50, 370), 6);

      sfe::RichText ssHelp(font_);
      ssHelp << "<Space>: Pause";
      ssHelp << "\n<F>: Play in fast speed";
      ssHelp << "\n<S>: Play in super fast speed";
      ssHelp << "\n<B>: Play Backward";
      DrawText(ssHelp, sf::Vector2f(100, 370), 6);
    } else {
      sfe::RichText ssHelp(font_);

      ssHelp << "<Tab>: Invert Field View\n";

      if (is_inverted_) {

          ssHelp << "State: INVERTED (Attacking RIGHT)\n"; 

      } else {

          ssHelp << "State: NORMAL (Attacking LEFT)\n";

      } 
      ssHelp << "\n<Esc>: Exit";
      DrawText(ssHelp, sf::Vector2f(-50, 370), 6);
    }

    if (!is_replay_ && !is_quiet_mode_ && is_update_) {
      Json::FastWriter writer;
      log_ << writer.write(json);
    }
    log_.flush();

    // Display
    window_->display();

    // ros::spinOnce(); // 修改：移除 ros::spinOnce()
    std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 修改：r.sleep(); (近似30Hz)
  }

  is_stopped_ = true;
  if (!is_replay_ && !is_quiet_mode_) {
    log_.close();
  }
  
  if (thread_capture_ != nullptr) {
    thread_capture_->join();
  }
  if (thread_show_ != nullptr) {
    thread_show_->join();
  }
}

void Monitor::GlobalToScreen(sf::Vector2f &in) {
  in.x += window_width_ / 2.0;
  in.y += window_height_ / 2.0;
}

void Monitor::DrawFieldLine(const sf::Vector2f &p1, const sf::Vector2f &p2) {
  if (std::fabs(p1.x - p2.x) > std::fabs(p1.y - p2.y)) {
    // draw horizontal line
    double size_x = std::fabs(p1.x - p2.x);
    double size_y = 5;
    sf::RectangleShape shape(sf::Vector2f(size_x, size_y));
    shape.setFillColor(GetColor(0));
    shape.move(sf::Vector2f(-size_x / 2.0, -size_y / 2.0));
    shape.move(sf::Vector2f(0.5 * p1.x + 0.5 * p2.x, 0.5 * p1.y + 0.5 * p2.y));
    shape.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
    window_->draw(shape);
  } else {
    // draw vertical line
    double size_x = 5;
    double size_y = fabs(p1.y - p2.y);
    sf::RectangleShape shape(sf::Vector2f(size_x, size_y));
    shape.setFillColor(GetColor(0));
    shape.move(sf::Vector2f(-size_x / 2.0, -size_y / 2.0));
    shape.move(sf::Vector2f(0.5 * p1.x + 0.5 * p2.x, 0.5 * p1.y + 0.5 * p2.y));
    shape.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
    window_->draw(shape);
  }
}

void Monitor::DrawField() {
  double fieldWidth = dconstant::geometry::fieldLength;
  double fieldHeight = dconstant::geometry::fieldWidth;
  double goalWidth = dconstant::geometry::goalWidth;
  double goalDepth = dconstant::geometry::goalDepth;
  double penaltyAreaDepth = dconstant::geometry::penaltyAreaLength;
  double penaltyAreaWidth = dconstant::geometry::penaltyAreaWidth;
  double goalAreaDepth = dconstant::geometry::goalAreaLength;
  double goalAreaWidth = dconstant::geometry::goalAreaWidth;
  double penaltyMarkDistance = dconstant::geometry::penaltyMarkDistance;

  // Draw field lines
  // border and central line
  DrawFieldLine(sf::Vector2f(-fieldWidth / 2, -fieldHeight / 2),
                sf::Vector2f(fieldWidth / 2, -fieldHeight / 2));
  DrawFieldLine(sf::Vector2f(-fieldWidth / 2, fieldHeight / 2),
                sf::Vector2f(fieldWidth / 2, fieldHeight / 2));
  DrawFieldLine(sf::Vector2f(-fieldWidth / 2, fieldHeight / 2),
                sf::Vector2f(-fieldWidth / 2, -fieldHeight / 2));
  DrawFieldLine(sf::Vector2f(fieldWidth / 2, fieldHeight / 2),
                sf::Vector2f(fieldWidth / 2, -fieldHeight / 2));
  DrawFieldLine(sf::Vector2f(0.0, fieldHeight / 2),
                sf::Vector2f(0.0, -fieldHeight / 2));
// left penalty zone
  DrawFieldLine(
      sf::Vector2f(-fieldWidth / 2, -goalAreaWidth / 2),
      sf::Vector2f(-fieldWidth / 2 + goalAreaDepth, -goalAreaWidth / 2));
  DrawFieldLine(
      sf::Vector2f(-fieldWidth / 2, goalAreaWidth / 2),
      sf::Vector2f(-fieldWidth / 2 + goalAreaDepth, goalAreaWidth / 2));
  DrawFieldLine(
      sf::Vector2f(-fieldWidth / 2 + goalAreaDepth, -goalAreaWidth / 2),
      sf::Vector2f(-fieldWidth / 2 + goalAreaDepth, goalAreaWidth / 2));
  // right penalty zone
  DrawFieldLine(
      sf::Vector2f(fieldWidth / 2, -goalAreaWidth / 2),
      sf::Vector2f(fieldWidth / 2 - goalAreaDepth, -goalAreaWidth / 2));
  DrawFieldLine(
      sf::Vector2f(fieldWidth / 2, goalAreaWidth / 2),
      sf::Vector2f(fieldWidth / 2 - goalAreaDepth, goalAreaWidth / 2));
  DrawFieldLine(
      sf::Vector2f(fieldWidth / 2 - goalAreaDepth, -goalAreaWidth / 2),
      sf::Vector2f(fieldWidth / 2 - goalAreaDepth, goalAreaWidth / 2));

  // left bigger penalty zone
  DrawFieldLine(
      sf::Vector2f(-fieldWidth / 2, -penaltyAreaWidth / 2),
      sf::Vector2f(-fieldWidth / 2 + penaltyAreaDepth, -penaltyAreaWidth / 2));
  DrawFieldLine(
      sf::Vector2f(-fieldWidth / 2, penaltyAreaWidth / 2),
      sf::Vector2f(-fieldWidth / 2 + penaltyAreaDepth, penaltyAreaWidth / 2));
  DrawFieldLine(
      sf::Vector2f(-fieldWidth / 2 + penaltyAreaDepth, -penaltyAreaWidth / 2),
      sf::Vector2f(-fieldWidth / 2 + penaltyAreaDepth, penaltyAreaWidth / 2));
  // right bigger penalty zone
  DrawFieldLine(
      sf::Vector2f(fieldWidth / 2, -penaltyAreaWidth / 2),
      sf::Vector2f(fieldWidth / 2 - penaltyAreaDepth, -penaltyAreaWidth / 2));
  DrawFieldLine(
      sf::Vector2f(fieldWidth / 2, penaltyAreaWidth / 2),
      sf::Vector2f(fieldWidth / 2 - penaltyAreaDepth, penaltyAreaWidth / 2));
  DrawFieldLine(
      sf::Vector2f(fieldWidth / 2 - penaltyAreaDepth, -penaltyAreaWidth / 2),
      sf::Vector2f(fieldWidth / 2 - penaltyAreaDepth, penaltyAreaWidth / 2));

// left goal zone
  DrawFieldLine(sf::Vector2f(-fieldWidth / 2, -goalWidth / 2),
                sf::Vector2f(-fieldWidth / 2 - goalDepth, -goalWidth / 2));
  DrawFieldLine(sf::Vector2f(-fieldWidth / 2, goalWidth / 2),
                sf::Vector2f(-fieldWidth / 2 - goalDepth, goalWidth / 2));
  DrawFieldLine(sf::Vector2f(-fieldWidth / 2 - goalDepth, -goalWidth / 2),
                sf::Vector2f(-fieldWidth / 2 - goalDepth, goalWidth / 2));
  // right goal zone
  DrawFieldLine(sf::Vector2f(fieldWidth / 2, -goalWidth / 2),
                sf::Vector2f(fieldWidth / 2 + goalDepth, -goalWidth / 2));
  DrawFieldLine(sf::Vector2f(fieldWidth / 2, goalWidth / 2),
                sf::Vector2f(fieldWidth / 2 + goalDepth, goalWidth / 2));
  DrawFieldLine(sf::Vector2f(fieldWidth / 2 + goalDepth, -goalWidth / 2),
                sf::Vector2f(fieldWidth / 2 + goalDepth, goalWidth / 2));
  // left penalty marker
  DrawFieldLine(sf::Vector2f(-fieldWidth / 2 + penaltyMarkDistance - 10, 0),
                sf::Vector2f(-fieldWidth / 2 + penaltyMarkDistance + 10, 0));
  DrawFieldLine(sf::Vector2f(-fieldWidth / 2 + penaltyMarkDistance, -10),
                sf::Vector2f(-fieldWidth / 2 + penaltyMarkDistance, 10));
  // right penalty marker
  DrawFieldLine(sf::Vector2f(fieldWidth / 2 - penaltyMarkDistance - 10, 0),
                sf::Vector2f(fieldWidth / 2 - penaltyMarkDistance + 10, 0));
  DrawFieldLine(sf::Vector2f(fieldWidth / 2 - penaltyMarkDistance, -10),
                sf::Vector2f(fieldWidth / 2 - penaltyMarkDistance, 10));

  // Draw central circle
  double radius = 150 / 2.0;
  sf::CircleShape circle(radius);
  circle.move(-radius, -radius);
  circle.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
  circle.setOutlineColor(GetColor(0));
  circle.setFillColor(sf::Color::Transparent);
  circle.setOutlineThickness(5);
  window_->draw(circle);
}

sf::Color Monitor::Monitor::GetColor(const int &id) {
  sf::Color color(200, 200, 200, global_alpha_);
  if (id == 0) {
    // field line
    color = sf::Color(145, 130, 116, global_alpha_);
  } else if (id == 1) {
    // red
    color = sf::Color(251, 66, 56, global_alpha_);
  } else if (id == 2) {
    // yellow
    color = sf::Color(251, 186, 61, global_alpha_);
  } else if (id == 3) {
    // green
    color = sf::Color(120, 171, 50, global_alpha_);
  } else if (id == 4) {
    // blue
    color = sf::Color(97, 158, 150, global_alpha_);
  } else if (id == 5) {
    // pink
    color = sf::Color(210, 131, 154, global_alpha_);
  } else if (id == 6) {
    // white
    color = sf::Color(235, 217, 178, global_alpha_);
  } else {
    // background
    color = sf::Color(34, 37, 38, global_alpha_);
  }
  return color;
}

void Monitor::DrawText(sfe::RichText &text, const sf::Vector2f &pos,
                       const int &id) {
  (void)id;
  double size = 0.8;
  text.set_font(font_);
  text.set_font_size(20);
  text.move(pos.x, -pos.y);
  text.scale(size, size);
  text.move(-3.0, -size * 20);  // 3.0 is ball circle thickness
  text.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
  window_->draw(text);
}

void Monitor::DrawText(const std::string &str, const sf::Vector2f &pos,
                       const int &id) {
  sfe::RichText text(font_);
  text << GetColor(id) << str;
  DrawText(text, pos, id);
}

void Monitor::DrawBall(const sf::Vector2f &pos, const int &id) {
  double radius = 7.5;
  sf::CircleShape circle(radius);
  circle.setOrigin(radius, radius);
  circle.move(pos.x, -pos.y);
  circle.setFillColor(sf::Color::Transparent);
  circle.setOutlineColor(GetColor(id));
  circle.setOutlineThickness(3);
  circle.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
  window_->draw(circle);

  sf::CircleShape circle2(1.5 * radius);
  circle2.setOrigin(1.5 * radius, 1.5 * radius);
  circle2.move(pos.x, -pos.y);
  circle2.setFillColor(sf::Color::Transparent);
  circle2.setOutlineColor(GetColor(id));
  circle2.setOutlineThickness(3);
  circle2.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
  window_->draw(circle2);
}

void Monitor::DrawCircle(const sf::Vector2f &pos, const int &id) {
  int centerCircleDiameter = dconstant::geometry::centerCircleDiameter;
  double radius = centerCircleDiameter / 2;
  sf::CircleShape circle(radius);
  circle.setOrigin(radius, radius);
  circle.move(pos.x, -pos.y);
  circle.setFillColor(sf::Color::Transparent);
  circle.setOutlineColor(GetColor(id));
  circle.setOutlineThickness(2);
  circle.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
  window_->draw(circle);
}

void Monitor::DrawGoal(const sf::Vector2f &pos, const int &id) {
  double radius = 5;
  sf::CircleShape circle(radius);
  circle.setOrigin(radius, radius);
  circle.move(pos.x, -pos.y);
  circle.setFillColor(GetColor(id));
  circle.setOutlineColor(GetColor(id));
  circle.setOutlineThickness(2);
  circle.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
  window_->draw(circle);
}

void Monitor::DrawPlayer(const sf::Vector2f &pos, double yaw, const int &id) {
  double size_x = 15;
  double size_y = 30;
  sf::RectangleShape shape1(sf::Vector2f(size_x, size_y));
  shape1.setOrigin(size_x / 2.0, size_y / 2.0);
  shape1.rotate(-yaw);
  shape1.move(
      sf::Vector2f(window_width_ / 2.0 + pos.x, window_height_ / 2.0 - pos.y));
  shape1.setFillColor(GetColor(id));
  window_->draw(shape1);

  sf::RectangleShape shape2(sf::Vector2f(size_x, size_y / 4.0));
  shape2.setOrigin(-size_x / 2.0, size_y / 8.0);
  shape2.rotate(-yaw);
  shape2.move(
      sf::Vector2f(window_width_ / 2.0 + pos.x, window_height_ / 2.0 - pos.y));
  shape2.setFillColor(GetColor(id));
  window_->draw(shape2);

  //  sf::RectangleShape shape3(sf::Vector2f(size_x / 2.0, size_y / 10.0));
  //  shape3.setOrigin(-size_x * 1.5, size_y / 20.0);
  //  shape3.rotate(-yaw);
  //  shape3.move(sf::Vector2f(window_width_ / 2.0 + pos.x,
  //                           window_height_ / 2.0 - pos.y));
  //  shape3.setFillColor(GetColor(id));
  //  window_->draw(shape3);
}

void Monitor::DrawLine(const sf::Vector2f &from, const sf::Vector2f &to,
                       const int &id, double thickness) {
  auto diff = to - from;
  auto yaw = std::atan2(diff.y, diff.x);
  auto dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);

  sf::RectangleShape shape(sf::Vector2f(dist, thickness));
  shape.setOrigin(0, thickness / 2.0);
  shape.rotate(dancer_geometry::rad2deg(-yaw));
  shape.move(sf::Vector2f(window_width_ / 2.0 + from.x,
                          window_height_ / 2.0 - from.y));
  shape.setFillColor(GetColor(id));
  window_->draw(shape);
}

void Monitor::DrawDashedLine(dancer_geometry::Point pt1,
                             dancer_geometry::Point pt2, const int &id) {
  double delta = 5;
  while ((pt2 - pt1).length() > delta) {
    dancer_geometry::Point target = pt1 + (pt2 - pt1).normalize(delta / 2);
    DrawLine(sf::Vector2f(pt1.x, pt1.y), sf::Vector2f(target.x, target.y), id);
    pt1 = pt1 + (pt2 - pt1).normalize(delta);
  }
}

void Monitor::DrawTarget(const sf::Vector2f &pos, const sf::Vector2f &dest,
                         const sf::Vector2f &final_dest, const int &id) {
  double size_x = 10;
  double size_y = 2;

  dancer_geometry::Point pt(pos.x, pos.y);
  dancer_geometry::Point pt2(dest.x, dest.y);
  dancer_geometry::Point pt3(final_dest.x, final_dest.y);

  DrawDashedLine(pt, pt2, id);
  global_alpha_ = 100;
  DrawDashedLine(pt2, pt3, id);
  global_alpha_ = 255;

  for (int angle : {-45, 45}) {
    sf::RectangleShape shape1(sf::Vector2f(size_x, size_y));
    shape1.setOrigin(size_x / 2.0, size_y / 2.0);
    shape1.rotate(angle);
    shape1.move(sf::Vector2f(dest.x, -dest.y));
    shape1.setFillColor(GetColor(id));
    shape1.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
    window_->draw(shape1);

    sf::RectangleShape shape2(sf::Vector2f(size_x * 2, size_y * 2));
    shape2.setOrigin(size_x * 2 / 2.0, size_y * 2 / 2.0);
    shape2.rotate(angle);
    shape2.move(sf::Vector2f(final_dest.x, -final_dest.y));
    global_alpha_ = 100;
    shape2.setFillColor(GetColor(id));
    global_alpha_ = 255;
    shape2.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
    window_->draw(shape2);
  }
}

void Monitor::DrawAttackTarget(const sf::Vector2f &pos,
                               const sf::Vector2f &attack_target,
                               const int &id) {
  double size_x = 10;
  double size_y = 2;

  dancer_geometry::Point pt(pos.x, pos.y);
  dancer_geometry::Point pt2(attack_target.x, attack_target.y);

  DrawDashedLine(pt, pt2, id);
  global_alpha_ = 255;

  for (int angle : {-45, 45}) {
    sf::RectangleShape shape2(sf::Vector2f(size_x * 2, size_y * 2));
    shape2.setOrigin(size_x * 2 / 2.0, size_y * 2 / 2.0);
    shape2.rotate(angle);
    shape2.move(sf::Vector2f(attack_target.x, -attack_target.y));
    global_alpha_ = 100;
    shape2.setFillColor(GetColor(id));
    global_alpha_ = 255;
    shape2.move(sf::Vector2f(window_width_ / 2.0, window_height_ / 2.0));
    window_->draw(shape2);
  }
}

bool Monitor::LoadReplayLine(std::ifstream &replay,
                             std::map<size_t, dmsgs::TeamInfo> &allInfo,
                             double *replayTimePtr, size_t *framePtr) {
  // Check file end
  if (!replay.good() || replay.peek() == EOF) {
    return false;
  }
  // Peeking the next line
  std::string line;
  std::getline(replay, line);
  // Trying to parse
  Json::Reader reader;
  Json::Value json;
  if (reader.parse(line, json)) {
    if (json.isMember("ts") && json.isMember("frame")) {
      if (replayTimePtr != nullptr) {
        *replayTimePtr = json["ts"].asUInt();
      }
      if (framePtr != nullptr) {
        *framePtr = json["frame"].asUInt();
      }
      for (auto &infoJson : json["info"]) {
        dmsgs::TeamInfo info;
        TeamInfofromJson(info, infoJson);
        allInfo[info.player_number] = info;
      }
    }
  }

  return true;
}

void Monitor::TeamInfofromJson(dmsgs::TeamInfo &info,
                               const Json::Value &json_value) {
  if (json_value.size() >= 36) {
    int k = 0;
    // timestamp
    info.txp_timestamp = from_sec(json_value[k++].asDouble()); // 修改：info.txp_timestamp = ros::Time(json_value[k++].asDouble());
    info.recv_timestamp = from_sec(json_value[k++].asDouble()); // 修改：info.recv_timestamp = ros::Time(json_value[k++].asDouble());
    // robot id
    info.player_number = json_value[k++].asUInt();
    info.team_number = json_value[k++].asUInt();
    info.incapacitated = json_value[k++].asBool();
    // robot role and dest
    info.role = json_value[k++].asUInt();
    info.dest.x = json_value[k++].asDouble();
    info.dest.y = json_value[k++].asDouble();
    info.final_dest.x = json_value[k++].asDouble();
    info.final_dest.y = json_value[k++].asDouble();
    info.attack_target.x = json_value[k++].asDouble();
    info.attack_target.y = json_value[k++].asDouble();
    info.time_since_last_kick = json_value[k++].asFloat();
    // robot state
    info.state = json_value[k++].asUInt();
    info.priority = json_value[k++].asUInt();
    // localization and detections
    info.see_ball = json_value[k++].asBool();
    info.see_circle = json_value[k++].asBool();
    info.robot_pos.x = json_value[k++].asDouble();
    info.robot_pos.y = json_value[k++].asDouble();
    info.robot_pos.z = json_value[k++].asDouble();
    info.ball_field.x = json_value[k++].asDouble();
    info.ball_field.y = json_value[k++].asDouble();
    info.ball_global.x = json_value[k++].asDouble();
    info.ball_global.y = json_value[k++].asDouble();
    info.circle_field.x = json_value[k++].asDouble();
    info.circle_field.y = json_value[k++].asDouble();
    info.circle_global.x = json_value[k++].asDouble();
    info.circle_global.y = json_value[k++].asDouble();
    info.goal_global.x = json_value[k++].asDouble();
    info.goal_global.y = json_value[k++].asDouble();
    // quality
    info.ball_quality = json_value[k++].asFloat();
    info.field_quality = json_value[k++].asFloat();
    info.field_consistency = json_value[k++].asFloat();
    // game controller
    info.gc_connected = json_value[k++].asBool();
    info.gc_state = json_value[k++].asUInt();
    info.gc_state2 = json_value[k++].asUInt();
  } else {
    std::cerr << "TeamPlayInfo::fromJson bad array size!" << std::endl;
  }
}

Json::Value Monitor::TeamInfoToJson(const dmsgs::TeamInfo &info) {
  Json::Value json(Json::arrayValue);
  // timestamp
  json.append(to_sec(info.txp_timestamp)); // 修改：json.append(info.txp_timestamp.toSec());
  json.append(to_sec(info.recv_timestamp)); // 修改：json.append(info.recv_timestamp.toSec());
  // robot id
  json.append(info.player_number);
  json.append(info.team_number);
  json.append(info.incapacitated);
  // robot role and dest
  json.append(info.role);
  json.append(info.dest.x);
  json.append(info.dest.y);
  json.append(info.final_dest.x);
  json.append(info.final_dest.y);
  json.append(info.attack_target.x);
  json.append(info.attack_target.y);
  json.append(info.time_since_last_kick);
  // robot state
  json.append(info.state);
  json.append(info.priority);
  // localization and detections
  json.append(info.see_ball);
  json.append(info.see_circle);
  json.append(info.robot_pos.x);
  json.append(info.robot_pos.y);
  json.append(info.robot_pos.z);
  json.append(info.ball_field.x);
  json.append(info.ball_field.y);
  json.append(info.ball_global.x);
  json.append(info.ball_global.y);
  json.append(info.circle_field.x);
  json.append(info.circle_field.y);
  json.append(info.circle_global.x);
  json.append(info.circle_global.y);
  json.append(info.goal_global.x);
  json.append(info.goal_global.y);
  // quality
  json.append(info.ball_quality);
  json.append(info.field_quality);
  json.append(info.field_consistency);
  // game controller
  json.append(info.gc_connected);
  json.append(info.gc_state);
  json.append(info.gc_state2);

  return json;
}
}  // namespace dmonitor