/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file dtransmit.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-03-04
 */

#pragma once

// #include <ros/ros.h> // 修改：移除 ROS 核心头文件

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>

#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <map>
#include <iostream> // 修改：添加 iostream 用于日志

#include "dtransmit/recv_socket.hpp"

namespace dtransmit {

/**
 * @brief Transmitting information over UDP.
 */
class DTransmit {

 public:
  /**
   * @brief DTransmit constructor.
   *
   * @param address - udp broadcast address
   */
  explicit DTransmit(const std::string &address = "",
                     const bool &use_local_loop = false);
  /**
   * @brief DTransmit destructor
   */
  ~DTransmit();

  /**
   * @brief Add listener for receiving raw messages.
   *
   * @param port - listening port
   * @param callback - callback function when receiving messages
   */
  void addRawRecv(PORT port, std::function<void(void *, std::size_t)> callback);

  /**
   * @brief Add listener for receiving raw messages from specific remote
   * endpoint.
   *
   * @param port - listening port
   * @param remoteEndpoint - given remote endpoint address
   * @param callback - callback function when receiving messages
   */
  void addRawRecvFiltered(PORT port, std::string remoteEndpoint,
                          std::function<void(void *, std::size_t)> callback);

  /**
   * @brief Send raw messages.
   *
   * @param port - gievn port
   * @param buffer - buffer to send
   * @param size - size of buffer
   */
  void sendRaw(PORT port, const void *buffer, std::size_t size);

  /**
   * @brief Start service of asio.
   */
  void startService();

  // ----- 修改：完全移除 addRosRecv 和 sendRos -----
  //
  // template<typename ROSMSG>
  // void addRosRecv(PORT port, std::function<void(ROSMSG &)> callback);
  //
  // template<typename ROSMSG>
  // void sendRos(PORT port, ROSMSG &);
  //
  // ------------------------------------------------

 private:
  /**
   * @brief Start receiving.
   *
   * @tparam ReadHandler - type of handler
   * @param port - port for receiving messages
   * @param handler - handler for reading sockets
   */
  template<typename ReadHandler>
  void startRecv(PORT port, ReadHandler handler);
  /**
   * @brief Create socket for sending messages.
   *
   * @param addr - broadcast address for sending messages
   * @param port - port for sending messages
   */
  void createSendSocket(const std::string &addr, const PORT &port);
  /**
   * @brief Send buffer.
   *
   * @param boost::asio::ip::udp::socket - socket
   * @param buffer - buffer to send
   * @param size - size of buffer
   */
  void sendBuffer(boost::asio::ip::udp::socket *, const void *buffer,
                  std::size_t size);

  /**
   * @brief Retrieve for all interfaces the broadcast address
   */
  void retrieveBroadcastAddress();

  //! Broadcast addresses of all interfaces
  std::vector<std::string> broadcast_addresses_;
  //! IO service
  boost::asio::io_service service_;

  //! Thread instance
  std::thread thread_;
  //! Map of ports and corresponding Foo for receiving messages
  std::map<PORT, Foo> recv_foo_;
  //! Map of ports and sockets for sending messages
  std::map<std::pair<std::string, PORT>, boost::asio::ip::udp::socket *> send_sockets_;
};

template<typename ReadHandler>
void DTransmit::startRecv(PORT port, ReadHandler handler) {
  recv_foo_[port].socket->async_receive_from(
      boost::asio::buffer(
          boost::asio::mutable_buffer((void *) &recv_foo_[port].recvBuffer,
                                      sizeof(recv_foo_[port].recvBuffer))),
      recv_foo_[port].remoteEndpoint, handler);
}

// ----- 修改：完全移除 addRosRecv 和 sendRos 的实现 -----
//
// template<typename ROSMSG>
// void DTransmit::addRosRecv(PORT port, std::function<void(ROSMSG &)> callback) {
//   ...
// }
//
// template<typename ROSMSG>
// void DTransmit::sendRos(PORT port, ROSMSG &rosmsg) {
//   ...
// }
//
// --------------------------------------------------------

}  // namespace dtransmit