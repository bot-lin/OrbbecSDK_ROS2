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

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace orbbec_camera {

/// Lightweight low-latency HTTP MJPEG streaming server.
///
/// Accepts TCP connections and serves an MJPEG stream using the standard
/// `multipart/x-mixed-replace` content type.
///
/// Compatible with VLC, FFplay, browsers, and any MJPEG-over-HTTP client:
///   vlc --network-caching=0 http://<host>:<port>/
///   ffplay -fflags nobuffer -flags low_delay http://<host>:<port>/
///
/// To relay to RTSP if needed:
///   ffmpeg -i http://<host>:<port>/ -c copy -f rtsp rtsp://...
class MjpegStreamer {
 public:
  explicit MjpegStreamer(int port = 8081, const std::string& bind_address = "0.0.0.0");
  ~MjpegStreamer();

  MjpegStreamer(const MjpegStreamer&) = delete;
  MjpegStreamer& operator=(const MjpegStreamer&) = delete;

  void start();
  void stop();

  /// Push a JPEG frame to all connected clients (non-blocking).
  void sendFrame(const uint8_t* data, size_t size);

  size_t clientCount() const;

 private:
  void acceptLoop();
  void clientLoop(int client_fd);
  void removeClient(int client_fd);

  int port_;
  std::string bind_address_;
  int server_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread accept_thread_;

  mutable std::mutex clients_mutex_;
  std::vector<int> client_fds_;

  // Latest frame shared with all clients
  mutable std::mutex frame_mutex_;
  std::vector<uint8_t> current_frame_;
  uint64_t frame_seq_ = 0;

  std::mutex threads_mutex_;
  std::vector<std::thread> client_threads_;
};

}  // namespace orbbec_camera
