/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "orbbec_camera/mjpeg_streamer.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <sstream>

namespace orbbec_camera {

static constexpr const char* BOUNDARY = "mjpeg_frame_boundary";

MjpegStreamer::MjpegStreamer(int port, const std::string& bind_address)
    : port_(port), bind_address_(bind_address) {}

MjpegStreamer::~MjpegStreamer() { stop(); }

void MjpegStreamer::start() {
  if (running_.load()) {
    return;
  }

  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    return;
  }

  int opt = 1;
  ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port_));
  ::inet_pton(AF_INET, bind_address_.c_str(), &addr.sin_addr);

  if (::bind(server_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  if (::listen(server_fd_, 4) < 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  running_.store(true);
  accept_thread_ = std::thread(&MjpegStreamer::acceptLoop, this);
}

void MjpegStreamer::stop() {
  running_.store(false);

  if (server_fd_ >= 0) {
    ::shutdown(server_fd_, SHUT_RDWR);
    ::close(server_fd_);
    server_fd_ = -1;
  }

  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int fd : client_fds_) {
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
    }
    client_fds_.clear();
  }

  {
    std::lock_guard<std::mutex> lock(threads_mutex_);
    for (auto& t : client_threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
    client_threads_.clear();
  }
}

void MjpegStreamer::sendFrame(const uint8_t* data, size_t size) {
  if (!running_.load() || data == nullptr || size == 0) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    current_frame_.assign(data, data + size);
    frame_seq_++;
  }
}

size_t MjpegStreamer::clientCount() const {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  return client_fds_.size();
}

void MjpegStreamer::acceptLoop() {
  while (running_.load()) {
    struct sockaddr_in client_addr {};
    socklen_t client_len = sizeof(client_addr);
    int client_fd =
        ::accept(server_fd_, reinterpret_cast<struct sockaddr*>(&client_addr), &client_len);
    if (client_fd < 0) {
      if (!running_.load()) {
        break;
      }
      continue;
    }

    // Disable Nagle for lower latency
    int flag = 1;
    ::setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    // Send buffer sized for one 1080p MJPG frame (~100-200 KB)
    int sndbuf = 262144;  // 256 KB
    ::setsockopt(client_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      client_fds_.push_back(client_fd);
    }

    std::lock_guard<std::mutex> tlock(threads_mutex_);
    client_threads_.emplace_back(&MjpegStreamer::clientLoop, this, client_fd);
  }
}

void MjpegStreamer::clientLoop(int client_fd) {
  // Drain the HTTP request (we don't parse it)
  {
    char buf[2048];
    struct timeval tv {};
    tv.tv_sec = 1;
    ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::recv(client_fd, buf, sizeof(buf), 0);
  }

  // Send HTTP response header with no-cache directives
  std::ostringstream header;
  header << "HTTP/1.1 200 OK\r\n"
         << "Content-Type: multipart/x-mixed-replace;boundary=" << BOUNDARY << "\r\n"
         << "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
         << "Pragma: no-cache\r\n"
         << "Expires: 0\r\n"
         << "Connection: close\r\n"
         << "X-Content-Type-Options: nosniff\r\n"
         << "\r\n";
  std::string hdr = header.str();
  if (::send(client_fd, hdr.c_str(), hdr.size(), MSG_NOSIGNAL) <= 0) {
    removeClient(client_fd);
    return;
  }

  uint64_t last_seq = 0;

  while (running_.load()) {
    // Poll for a new frame — lock only briefly to check and grab
    std::vector<uint8_t> frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (frame_seq_ == last_seq || current_frame_.empty()) {
        // No new frame yet — release lock BEFORE sleeping
      } else {
        // Grab the latest frame
        frame = current_frame_;
        last_seq = frame_seq_;
      }
    }

    if (frame.empty()) {
      // Sleep outside the lock to avoid blocking sendFrame()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    // Build part header (stack buffer, no heap alloc)
    char ph_buf[128];
    int ph_len = snprintf(ph_buf, sizeof(ph_buf),
                          "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                          BOUNDARY, frame.size());

    // Send part header
    if (::send(client_fd, ph_buf, static_cast<size_t>(ph_len), MSG_NOSIGNAL) <= 0) {
      break;
    }

    // Send JPEG data
    size_t sent = 0;
    while (sent < frame.size()) {
      ssize_t n = ::send(client_fd, frame.data() + sent, frame.size() - sent, MSG_NOSIGNAL);
      if (n <= 0) {
        break;
      }
      sent += static_cast<size_t>(n);
    }
    if (sent < frame.size()) {
      break;
    }

    // Trailing CRLF
    if (::send(client_fd, "\r\n", 2, MSG_NOSIGNAL) <= 0) {
      break;
    }
  }

  removeClient(client_fd);
}

void MjpegStreamer::removeClient(int client_fd) {
  ::shutdown(client_fd, SHUT_RDWR);
  ::close(client_fd);
  std::lock_guard<std::mutex> lock(clients_mutex_);
  client_fds_.erase(std::remove(client_fds_.begin(), client_fds_.end(), client_fd),
                    client_fds_.end());
}

}  // namespace orbbec_camera
