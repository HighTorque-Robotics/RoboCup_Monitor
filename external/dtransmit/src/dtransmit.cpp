/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file dtransmit.cpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-03-04
 */

#include <algorithm>
#include <ifaddrs.h>
#include "dtransmit/dtransmit.hpp"
#include <iostream>   // 修改：添加 iostream 用于日志
#include <cstring>    // 修改：添加 cstring 用于 strerror

using namespace std;
using namespace boost::asio;

namespace dtransmit {
DTransmit::DTransmit(const std::string &address,
                     const bool &use_local_loop)
    : service_() {
  // add broadcast address
  if (!address.empty()) {
    broadcast_addresses_.push_back(address);
  } else {
    retrieveBroadcastAddress();
  }
  // remove local loop 127.0.0.1 from broadcast address
  if (!use_local_loop) {
    broadcast_addresses_.erase(
        std::remove_if(broadcast_addresses_.begin(),
                       broadcast_addresses_.end(),
                       [](const std::string &addr) {
                         return addr == "127.0.0.1" || addr.find("172.17.") != std::string::npos;
                       }),
        broadcast_addresses_.end());
  }
  // pirnt info
  for (const auto &addr:broadcast_addresses_) {
    // ROS_INFO("New DTransmit on %s", addr.c_str());
    std::cout << "New DTransmit on " << addr << std::endl; // 修改：替换 ROS_INFO
  }
}

DTransmit::~DTransmit() {
  service_.stop();
  for (auto &p : send_sockets_) {
    ip::udp::socket *s = p.second;
    if (s) {
      if (s->is_open()) s->close();
      delete s;
    }
  }

  for (auto &p : recv_foo_) {
    ip::udp::socket *s = p.second.socket;
    if (s) {
      if (s->is_open()) s->close();

      delete s;
    }
  }

  service_.stop();
  if (thread_.joinable()) thread_.join();
}

void DTransmit::startService() {
  thread_ = std::thread([&]() {
    service_.reset();
    service_.run();
  });
}

void DTransmit::createSendSocket(const std::string &addr, const PORT &port) {
  using namespace boost::asio;

  ip::udp::endpoint broadcastEndpoint(
      ip::address::from_string(addr), port);
  send_sockets_[std::make_pair(addr, port)] = new ip::udp::socket(service_, ip::udp::v4());
  send_sockets_[std::make_pair(addr, port)]->set_option(socket_base::broadcast(true));

  boost::system::error_code ec;
  send_sockets_[std::make_pair(addr, port)]->connect(broadcastEndpoint, ec);
  if (ec) {
    // ROS_ERROR("DTransmit create sendRos socket error: %s", ec.message().c_str());
    std::cerr << "DTransmit create send socket error: " << ec.message() << std::endl; // 修改：替换 ROS_ERROR
  }
  // ROS_DEBUG("DTransmit create send socket on %s:%d", addr.c_str(), port);
  std::cout << "DTransmit create send socket on " << addr << ":" << port << std::endl; // 修改：替换 ROS_DEBUG
}

void DTransmit::sendBuffer(boost::asio::ip::udp::socket *socket,
                           const void *buffer, std::size_t size) {
  boost::system::error_code ec;
  socket->send(boost::asio::buffer(buffer, size), 0, ec);

  if (ec) {
    // ROS_WARN("DTransmit can't send Ros buffer to %s:%d : %s",
    //          socket->local_endpoint().address().to_string().c_str(),
    //          socket->local_endpoint().port(),
    //          ec.message().c_str());
    std::cerr << "DTransmit can't send buffer to " 
              << socket->local_endpoint().address().to_string() << ":"
              << socket->local_endpoint().port() << " : "
              << ec.message() << std::endl; // 修改：替换 ROS_WARN
  }
}

void DTransmit::addRawRecv(PORT port,
                           std::function<void(void *, std::size_t)> callback) {

  if (recv_foo_.count(port)) {
    // ROS_ERROR("Error in addRawRecv: port %d exist!", port);
    std::cerr << "Error in addRawRecv: port " << port << " exist!" << std::endl; // 修改：替换 ROS_ERROR
    return;
  }
  recv_foo_[port] = Foo(service_, port);

  recv_foo_[port].readHandler = [=](const boost::system::error_code &error,
                                    std::size_t bytesRecved) {
    if (error) {
      // ROS_ERROR("Error in RawRecv: %s", error.message().c_str());
      std::cerr << "Error in RawRecv: " << error.message() << std::endl; // 修改：替换 ROS_ERROR
    } else {
      callback(recv_foo_[port].recvBuffer, bytesRecved);
    }

    startRecv(port, recv_foo_[port].readHandler);
  };
  startRecv(port, recv_foo_[port].readHandler);
}

void DTransmit::addRawRecvFiltered(
    PORT port, std::string remoteEndpoint,
    std::function<void(void *, std::size_t)> callback) {

  if (recv_foo_.count(port)) {
    // ROS_ERROR("Error in addRawRecv: port %d exist!", port);
    std::cerr << "Error in addRawRecv: port " << port << " exist!" << std::endl; // 修改：替换 ROS_ERROR
    return;
  }
  recv_foo_[port] = Foo(service_, port);

  recv_foo_[port].readHandler = [=](const boost::system::error_code &error,
                                    std::size_t bytesRecved) {
    if (error) {
      // ROS_ERROR("Error in RosRecv: %s", error.message().c_str());
      std::cerr << "Error in RawRecvFiltered: " << error.message() << std::endl; // 修改：替换 ROS_ERROR
    } else if (recv_foo_[port].remoteEndpoint.address().to_string() !=
        remoteEndpoint) {
      // ROS_WARN("Someone else [%s] is broadcasting on this port [%d], packet
      // filtered.",
      // recv_foo_[port].remoteEndpoint.address().to_string().c_str(), port);
      // 修改：移除 ROS_WARN，保持安静
    } else {
      callback(recv_foo_[port].recvBuffer, bytesRecved);
    }

    startRecv(port, recv_foo_[port].readHandler);
  };
  startRecv(port, recv_foo_[port].readHandler);
}

void DTransmit::sendRaw(PORT port, const void *buffer, std::size_t size) {

  for (const auto &addr:broadcast_addresses_) {
    if (!send_sockets_.count(std::make_pair(addr, port))) {
      createSendSocket(addr, port);
    }
    sendBuffer(send_sockets_[std::make_pair(addr, port)], buffer, size);
    // ROS_DEBUG("DTransmit send raw data on %s:%d", addr.c_str(), port);
    // 修改：移除 ROS_DEBUG，保持安静
  }
}

void DTransmit::retrieveBroadcastAddress() {
  struct ifaddrs *ifap;
  if (getifaddrs(&ifap) == -1) {
    // ROS_ERROR("Failed to getting broadcast address: %s", strerror(errno));
    std::cerr << "Failed to getting broadcast address: " << strerror(errno) << std::endl; // 修改：替换 ROS_ERROR
    return;
  }

  while (ifap != nullptr) {
    struct sockaddr *addr = ifap->ifa_broadaddr;
    if (addr != nullptr && addr->sa_family == AF_INET) {
      char str[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(((struct sockaddr_in *) addr)->sin_addr), str, INET_ADDRSTRLEN);
      broadcast_addresses_.emplace_back(str);
    }
    ifap = ifap->ifa_next;
  }
  
  // 修改：getifaddrs 分配的内存需要释放
  freeifaddrs(ifap);
}
}