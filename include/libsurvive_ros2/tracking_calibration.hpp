// Copyright 2026 Sunghyun Kwon
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <chrono>
#include <map>

#include "tf2/LinearMath/Transform.h"

namespace libsurvive_ros2
{
// Snapshots contain only recently observed, calibrated stations.
class TrackingCalibration
{
public:
  using Clock = std::chrono::steady_clock;
  using Stations = std::map<unsigned, tf2::Transform>;

  bool valid() const {return frozen_ && !baseline_.empty();}

  void invalidate() {baseline_.clear();}

  bool update(const Stations & stations, Clock::time_point now)
  {
    if (frozen_ && !valid()) {return false;}
    bool poses_match = true;
    for (const auto & [id, pose] : stations) {
      const auto previous = baseline_.find(id);
      if (previous == baseline_.end() ||
        pose.getOrigin().distance(previous->second.getOrigin()) > 0.01 ||
        pose.getRotation().angleShortestPath(previous->second.getRotation()) > 0.01745329252)
      {
        poses_match = false;
        break;
      }
    }
    if (frozen_) {
      if (!poses_match) {
        invalidate();
        return false;
      }
      // Visibility gates output, but elapsed occlusion is not evidence of a changed layout.
      return stations.size() == baseline_.size();
    }
    const bool unchanged = poses_match && !stations.empty() && stations.size() == baseline_.size();
    if (!unchanged) {
      baseline_ = stations;
      stable_since_ = now;
    }
    frozen_ = unchanged && now - stable_since_ >= std::chrono::seconds(2);
    return frozen_;
  }

private:
  bool frozen_{false};
  Stations baseline_;
  Clock::time_point stable_since_{};
};
}  // namespace libsurvive_ros2
