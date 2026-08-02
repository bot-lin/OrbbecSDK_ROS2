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

#include "orbbec_camera/ob_camera_node.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>

#include "orbbec_camera/utils.h"
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace orbbec_camera {

void OBCameraNode::setupPointCloudTf() {
  point_cloud_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  point_cloud_tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*point_cloud_tf_buffer_);
}

bool OBCameraNode::lookupBaseFootprintTransform(const std::string &source_frame_id,
                                                tf2::Transform &transform) const {
  transform.setIdentity();
  if (source_frame_id.empty() || base_footprint_frame_id_.empty()) {
    RCLCPP_WARN_THROTTLE(
        logger_, *(node_->get_clock()), 5000,
        "Cannot transform point cloud to base_footprint: source or target frame is empty");
    return false;
  }
  if (source_frame_id == base_footprint_frame_id_) {
    return true;
  }
  if (!point_cloud_tf_buffer_) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_->get_clock()), 5000,
                         "Cannot transform point cloud to %s: TF listener is not initialized",
                         base_footprint_frame_id_.c_str());
    return false;
  }

  try {
    const auto transform_msg =
        point_cloud_tf_buffer_->lookupTransform(base_footprint_frame_id_, source_frame_id,
                                                tf2::TimePointZero);
    const auto &translation = transform_msg.transform.translation;
    const auto &rotation = transform_msg.transform.rotation;
    transform = tf2::Transform(
        tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        tf2::Vector3(translation.x, translation.y, translation.z));
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_->get_clock()), 5000,
                         "Failed to transform point cloud from %s to %s: %s",
                         source_frame_id.c_str(), base_footprint_frame_id_.c_str(), ex.what());
  }
  return false;
}

bool OBCameraNode::pointInBaseFootprintRoi(const tf2::Vector3 &point) const {
  PointCloudRoi roi;
  roi.min_x = point_cloud_roi_min_x_;
  roi.max_x = point_cloud_roi_max_x_;
  roi.min_y = point_cloud_roi_min_y_;
  roi.max_y = point_cloud_roi_max_y_;
  roi.min_z = point_cloud_roi_min_z_;
  roi.max_z = point_cloud_roi_max_z_;
  return pointInBaseFootprintRoi(point, roi);
}

bool OBCameraNode::pointInBaseFootprintRoi(const tf2::Vector3 &point,
                                           const PointCloudRoi &roi) const {
  return point.x() >= roi.min_x && point.x() <= roi.max_x && point.y() >= roi.min_y &&
         point.y() <= roi.max_y && point.z() >= roi.min_z && point.z() <= roi.max_z;
}

bool OBCameraNode::pointOnRemovedGroundPlane(
    const tf2::Vector3 &point, const PointCloudGroundPlane &ground_plane) const {
  const double normal_norm =
      std::sqrt(ground_plane.a * ground_plane.a + ground_plane.b * ground_plane.b +
                ground_plane.c * ground_plane.c);
  if (normal_norm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  const double signed_distance =
      (ground_plane.a * point.x() + ground_plane.b * point.y() +
       ground_plane.c * point.z() + ground_plane.d) /
      normal_norm;
  return std::abs(signed_distance) <= ground_plane.distance_threshold;
}

bool OBCameraNode::finalizePointCloud(sensor_msgs::msg::PointCloud2 &point_cloud_msg,
                                      const std::string &source_frame_id,
                                      const rclcpp::Time &timestamp,
                                      bool output_in_base_footprint, const PointCloudRoi *roi,
                                      bool keep_ordered_layout, size_t &valid_count,
                                      std::string *message,
                                      const PointCloudGroundPlane *ground_plane) {
  valid_count = 0;
  const size_t point_count =
      static_cast<size_t>(point_cloud_msg.width) * static_cast<size_t>(point_cloud_msg.height);
  const bool needs_base_footprint =
      output_in_base_footprint || roi != nullptr || ground_plane != nullptr;
  tf2::Transform base_footprint_transform;
  base_footprint_transform.setIdentity();
  if (needs_base_footprint && point_count > 0 &&
      !lookupBaseFootprintTransform(source_frame_id, base_footprint_transform)) {
    if (message) {
      *message = "Failed to transform point cloud from " + source_frame_id + " to " +
                 base_footprint_frame_id_;
    }
    return false;
  }

  const float nan = std::numeric_limits<float>::quiet_NaN();
  if (point_count > 0) {
    sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");
    uint8_t *data = point_cloud_msg.data.data();
    const size_t point_step = point_cloud_msg.point_step;
    size_t write_index = 0;

    for (size_t read_index = 0; read_index < point_count;
         ++read_index, ++iter_x, ++iter_y, ++iter_z) {
      bool keep_point =
          std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z);
      tf2::Vector3 point_in_base_footprint;
      if (keep_point && needs_base_footprint) {
        point_in_base_footprint =
            base_footprint_transform * tf2::Vector3(*iter_x, *iter_y, *iter_z);
        if (output_in_base_footprint) {
          *iter_x = static_cast<float>(point_in_base_footprint.x());
          *iter_y = static_cast<float>(point_in_base_footprint.y());
          *iter_z = static_cast<float>(point_in_base_footprint.z());
        }
        if (roi) {
          keep_point = pointInBaseFootprintRoi(point_in_base_footprint, *roi);
        }
        if (keep_point && ground_plane) {
          keep_point = !pointOnRemovedGroundPlane(point_in_base_footprint, *ground_plane);
        }
      }

      if (keep_ordered_layout) {
        if (keep_point) {
          valid_count++;
        } else {
          *iter_x = nan;
          *iter_y = nan;
          *iter_z = nan;
        }
        continue;
      }

      if (!keep_point) {
        continue;
      }
      if (write_index != read_index) {
        std::memmove(data + write_index * point_step, data + read_index * point_step, point_step);
      }
      write_index++;
      valid_count++;
    }

    if (!keep_ordered_layout && valid_count != point_count) {
      sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
      modifier.resize(valid_count);
      point_cloud_msg.width = static_cast<uint32_t>(valid_count);
      point_cloud_msg.height = 1;
      point_cloud_msg.row_step = point_cloud_msg.width * point_cloud_msg.point_step;
      point_cloud_msg.data.resize(point_cloud_msg.row_step);
    }
  }

  point_cloud_msg.is_dense = !keep_ordered_layout;
  point_cloud_msg.header.stamp = timestamp;
  point_cloud_msg.header.frame_id =
      output_in_base_footprint
          ? base_footprint_frame_id_
          : (cloud_frame_id_.empty() ? source_frame_id : cloud_frame_id_);
  return true;
}

bool OBCameraNode::createDepthPointCloud(sensor_msgs::msg::PointCloud2::UniquePtr &point_cloud_msg,
                                         bool output_in_base_footprint,
                                         const PointCloudRoi *roi,
                                         bool apply_point_cloud_filters, size_t &valid_count,
                                         std::string &message,
                                         const PointCloudGroundPlane *ground_plane) {
  message.clear();
  valid_count = 0;
  point_cloud_msg.reset();

  std::lock_guard<decltype(point_cloud_mutex_)> point_cloud_msg_lock(point_cloud_mutex_);
  if (!depth_frame_) {
    message = "Depth frame is not available";
    return false;
  }
  auto depth_frame = depth_frame_->as<ob::DepthFrame>();
  if (!depth_frame) {
    message = "Depth frame is not valid";
    return false;
  }
  if (!pipeline_) {
    message = "Pipeline is not available";
    return false;
  }

  auto camera_params = pipeline_->getCameraParam();
  auto device_info = device_->getDeviceInfo();
  if (!device_info) {
    message = "Device info is not available";
    return false;
  }
  auto pid = device_info->pid();
  if (depth_registration_ || pid == DABAI_MAX_PID) {
    camera_params.depthIntrinsic = camera_params.rgbIntrinsic;
  }
  depth_point_cloud_filter_.setCameraParam(camera_params);
  float depth_scale = depth_frame->getValueScale();
  depth_point_cloud_filter_.setPositionDataScaled(depth_scale);
  depth_point_cloud_filter_.setCreatePointFormat(OB_FORMAT_POINT);
  auto result_frame = depth_point_cloud_filter_.process(depth_frame);
  if (!result_frame) {
    message = "Failed to generate point cloud from depth frame";
    return false;
  }

  auto point_size = result_frame->dataSize() / sizeof(OBPoint);
  auto *points = static_cast<OBPoint *>(result_frame->data());
  auto width = depth_frame->width();
  auto height = depth_frame->height();
  point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  const bool keep_ordered_layout = apply_point_cloud_filters && ordered_pc_;
  size_t stride = apply_point_cloud_filters ? static_cast<size_t>(point_cloud_stride_) : 1;
  if (stride < 1) {
    stride = 1;
  }
  if (keep_ordered_layout && stride > 1) {
    // Ordered point cloud expects width*height layout; subsampling would break this semantic.
    RCLCPP_WARN_THROTTLE(logger_, *(node_->get_clock()), 60000,
                         "point_cloud_stride is ignored when ordered_pc is true");
    stride = 1;
  }

  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  if (keep_ordered_layout) {
    modifier.resize(width * height);
    point_cloud_msg->width = width;
    point_cloud_msg->height = height;
    point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
    point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
  } else {
    // Pre-allocate a smaller buffer when publishing unordered point cloud.
    size_t reserved = stride > 1 ? (point_size + stride - 1) / stride : point_size;
    if (reserved == 0) {
      reserved = 1;
    }
    modifier.resize(reserved);
    point_cloud_msg->width = reserved;
    point_cloud_msg->height = 1;
    point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
    point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
  }

  const static float MIN_DISTANCE_MM = 20.0F;  // 2cm
  const float min_depth = apply_point_cloud_filters
                              ? static_cast<float>(MIN_DISTANCE_MM / depth_scale)
                              : -std::numeric_limits<float>::infinity();
  float max_depth = std::numeric_limits<float>::infinity();
  if (apply_point_cloud_filters && point_cloud_max_distance_ > 0.0) {
    const double max_distance_mm = point_cloud_max_distance_ * 1000.0;
    max_depth = static_cast<float>(max_distance_mm / depth_scale);
  }

  size_t candidate_count = 0;
  // Optional outlier suppression (unordered point cloud only)
  const bool enable_radius_filter = apply_point_cloud_filters && (!keep_ordered_layout) &&
                                    point_cloud_radius_ > 0.0 &&
                                    point_cloud_min_neighbors_ > 1;
  if (enable_radius_filter) {
    struct VoxelKey {
      int ix;
      int iy;
      int iz;
      bool operator==(const VoxelKey &o) const { return ix == o.ix && iy == o.iy && iz == o.iz; }
    };
    struct VoxelKeyHash {
      size_t operator()(const VoxelKey &k) const noexcept {
        size_t h = std::hash<int>()(k.ix);
        h ^= std::hash<int>()(k.iy) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>()(k.iz) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
      }
    };
    struct TmpPoint {
      float x;
      float y;
      float z;
      VoxelKey key;
    };

    const double r = point_cloud_radius_;
    const double r2 = r * r;
    // Use voxel hashing with voxel size = radius for neighbor lookup (27 neighboring voxels)
    const double vs = r;
    std::vector<TmpPoint> tmp;
    tmp.reserve(point_size / stride + 1);
    std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHash> voxel_bins;
    voxel_bins.reserve(point_size / stride + 1);

    for (size_t i = 0; i < point_size; i += stride) {
      bool valid_point = points[i].z >= min_depth && points[i].z <= max_depth;
      if (!valid_point) {
        continue;
      }
      const float x = static_cast<float>(points[i].x / 1000.0);
      const float y = static_cast<float>(points[i].y / 1000.0);
      const float z = static_cast<float>(points[i].z / 1000.0);
      VoxelKey key{static_cast<int>(std::floor(x / vs)), static_cast<int>(std::floor(y / vs)),
                   static_cast<int>(std::floor(z / vs))};
      const int idx = static_cast<int>(tmp.size());
      tmp.push_back(TmpPoint{x, y, z, key});
      voxel_bins[key].push_back(idx);
    }
    candidate_count = tmp.size();
    size_t keep_every = 1;
    if (point_cloud_max_points_ > 0 && candidate_count > static_cast<size_t>(point_cloud_max_points_)) {
      keep_every =
          (candidate_count + static_cast<size_t>(point_cloud_max_points_) - 1) /
          static_cast<size_t>(point_cloud_max_points_);
      if (keep_every < 1) {
        keep_every = 1;
      }
    }

    modifier.resize(tmp.size());
    point_cloud_msg->width = tmp.size();
    point_cloud_msg->height = 1;
    point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
    point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");

    size_t emitted = 0;
    for (const auto &p : tmp) {
      int neighbors = 0;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            VoxelKey nk{p.key.ix + dx, p.key.iy + dy, p.key.iz + dz};
            auto it = voxel_bins.find(nk);
            if (it == voxel_bins.end()) {
              continue;
            }
            for (int j : it->second) {
              const auto &q = tmp[static_cast<size_t>(j)];
              const double ddx = static_cast<double>(q.x) - p.x;
              const double ddy = static_cast<double>(q.y) - p.y;
              const double ddz = static_cast<double>(q.z) - p.z;
              const double d2 = ddx * ddx + ddy * ddy + ddz * ddz;
              if (d2 <= r2) {
                neighbors++;
                if (neighbors >= point_cloud_min_neighbors_) {
                  break;
                }
              }
            }
            if (neighbors >= point_cloud_min_neighbors_) {
              break;
            }
          }
          if (neighbors >= point_cloud_min_neighbors_) {
            break;
          }
        }
        if (neighbors >= point_cloud_min_neighbors_) {
          break;
        }
      }
      if (neighbors < point_cloud_min_neighbors_) {
        continue;
      }
      // Hard cap of output size: keep 1 point every keep_every accepted points
      if (keep_every > 1 && (emitted % keep_every) != 0) {
        emitted++;
        continue;
      }
      *iter_x = p.x;
      *iter_y = p.y;
      *iter_z = p.z;
      ++iter_x, ++iter_y, ++iter_z;
      valid_count++;
      emitted++;
    }
  } else {
    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    candidate_count = stride > 1 ? (point_size + stride - 1) / stride : point_size;
    size_t keep_every = 1;
    if (!keep_ordered_layout && apply_point_cloud_filters && point_cloud_max_points_ > 0 &&
        candidate_count > static_cast<size_t>(point_cloud_max_points_)) {
      keep_every =
          (candidate_count + static_cast<size_t>(point_cloud_max_points_) - 1) /
          static_cast<size_t>(point_cloud_max_points_);
      if (keep_every < 1) {
        keep_every = 1;
      }
    }
    size_t emitted = 0;
    for (size_t i = 0; i < point_size; i += stride) {
      bool valid_point = points[i].z >= min_depth && points[i].z <= max_depth;
      if (!valid_point) {
        if (keep_ordered_layout) {
          *iter_x = nan;
          *iter_y = nan;
          *iter_z = nan;
          ++iter_x, ++iter_y, ++iter_z;
        }
        continue;
      }
      if (!keep_ordered_layout && keep_every > 1 && (emitted % keep_every) != 0) {
        emitted++;
        continue;
      }
      *iter_x = static_cast<float>(points[i].x / 1000.0);
      *iter_y = static_cast<float>(points[i].y / 1000.0);
      *iter_z = static_cast<float>(points[i].z / 1000.0);
      ++iter_x, ++iter_y, ++iter_z;
      valid_count++;
      emitted++;
    }
  }
  if (!keep_ordered_layout) {
    point_cloud_msg->is_dense = true;
    point_cloud_msg->width = valid_count;
    point_cloud_msg->height = 1;
    modifier.resize(valid_count);
    point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
    point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
  }

  auto frame_timestamp = getFrameTimestampUs(depth_frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  const std::string source_frame_id =
      depth_registration_ ? optical_frame_id_[COLOR] : optical_frame_id_[DEPTH];
  return finalizePointCloud(*point_cloud_msg, source_frame_id, timestamp, output_in_base_footprint,
                            roi, keep_ordered_layout, valid_count, &message, ground_plane);
}

}  // namespace orbbec_camera
