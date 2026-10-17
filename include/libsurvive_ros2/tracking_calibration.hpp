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
// Only observed, calibrated stations belong in a snapshot; an incomplete snapshot is empty.
class TrackingCalibration
{
public:
  using Clock = std::chrono::steady_clock;
  using Stations = std::map<unsigned, tf2::Transform>;

  bool update(const Stations & stations, Clock::time_point now)
  {
    bool unchanged = !stations.empty() && stations.size() == baseline_.size();
    for (const auto & [id, pose] : stations) {
      const auto previous = baseline_.find(id);
      if (previous == baseline_.end() ||
        pose.getOrigin().distance(previous->second.getOrigin()) > 0.01 ||
        pose.getRotation().angleShortestPath(previous->second.getRotation()) > 0.01745329252)
      {
        unchanged = false;
      }
    }
    if (frozen_) {
      // A changed fixed frame stays invalid until a new calibration is requested.
      if (!unchanged) {baseline_.clear();}
      return unchanged;
    }
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
