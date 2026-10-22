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

#include <cassert>
#include <chrono>

#include "libsurvive_ros2/tracking_calibration.hpp"

using libsurvive_ros2::TrackingCalibration;
using namespace std::chrono_literals;

int main()
{
  const auto start = TrackingCalibration::Clock::time_point{} + 10s;
  const TrackingCalibration::Stations stations{
    {0x1234, tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 2))},
    {0x5678, tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(2, 0, 2))}};
  const auto ready_calibration = [&]() {
      TrackingCalibration calibration;
      assert(!calibration.update(stations, start));
      assert(!calibration.update(stations, start + 1999ms));
      assert(!calibration.valid());
      assert(calibration.update(stations, start + 2s));
      assert(calibration.valid());
      return calibration;
    };

  TrackingCalibration empty;
  assert(!empty.update({}, start));
  assert(!empty.update({}, start + 10s));
  assert(!empty.valid());

  auto transient = ready_calibration();
  auto missing = stations;
  missing.erase(0x5678);
  assert(!transient.update(missing, start + 3s));
  assert(transient.valid());
  assert(transient.update(stations, start + 7999ms));
  assert(transient.valid());
  assert(!transient.update({}, start + 9s));
  assert(!transient.update({}, start + 14s));
  assert(transient.valid());
  assert(!transient.update(missing, start + 1h));
  assert(transient.valid());
  assert(transient.update(stations, start + 1h + 1s));
  assert(transient.valid());

  for (int change = 0; change != 3; ++change) {
    auto calibration = ready_calibration();
    auto changed = stations;
    if (change == 0) {
      auto entry = changed.extract(0x5678);
      entry.key() = 0x9abc;
      changed.insert(std::move(entry));
    } else if (change == 1) {
      changed.at(0x5678).getOrigin().setX(2.011);
    } else {
      changed.at(0x5678).setRotation(tf2::Quaternion(tf2::Vector3(1, 0, 0), 0.018));
    }
    assert(!calibration.update({}, start + 3s));
    assert(!calibration.update({}, start + 1h));
    assert(calibration.valid());
    assert(!calibration.update(changed, start + 1h + 1s));
    assert(!calibration.valid());
    assert(!calibration.update(stations, start + 1h + 4s));
  }

  auto explicitly_invalidated = ready_calibration();
  explicitly_invalidated.invalidate();
  assert(!explicitly_invalidated.valid());
  assert(!explicitly_invalidated.update(stations, start + 4s));
}
