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
#include <cstring>
#include <limits>
#include "libsurvive_ros2/optical_diagnostic.hpp"

int main()
{
  using libsurvive_ros2::OpticalDiagnosticMonitor;
  OpticalDiagnosticMonitor monitor;
  const auto start = OpticalDiagnosticMonitor::Clock::now();
  SurviveOpticalDiagnostic sample{};
  assert(std::strcmp(monitor.quality(sample, start), "insufficient") == 0);
  sample.sequence = 1;
  sample.residual_count = sample.peak_residual_count = 16;
  sample.peak_lighthouse_count = 2;
  sample.peak_rms_rad = 0.001;
  assert(std::strcmp(monitor.quality(sample, start), "normal") == 0);
  sample.sequence++;
  sample.peak_rms_rad = 0.02;
  sample.rms_rad = 0.001;
  assert(std::strcmp(monitor.quality(sample, start), "suspect") == 0);
  sample.peak_lighthouse_count = 1;
  assert(std::strcmp(monitor.quality(sample, start), "insufficient") == 0);
  sample.peak_lighthouse_count = 2;
  sample.peak_residual_count = 4;
  assert(std::strcmp(monitor.quality(sample, start), "insufficient") == 0);
  sample.invalid_fits = 1;
  sample.rms_rad = std::numeric_limits<double>::quiet_NaN();
  assert(std::strcmp(monitor.quality(sample, start), "suspect") == 0);
  sample.invalid_fits = 0;
  sample.failed_fits = 1;
  assert(std::strcmp(monitor.quality(sample, start), "suspect") == 0);
  assert(std::strcmp(monitor.quality(sample, start + std::chrono::seconds(1)), "stale") == 0);
  sample.sequence++;
  assert(std::strcmp(monitor.quality(sample, start + std::chrono::seconds(2)), "suspect") == 0);
  sample.failed_fits = 0;
  sample.fitting_lighthouses = true;
  assert(std::strcmp(monitor.quality(sample, start + std::chrono::seconds(2)),
    "insufficient") == 0);
  sample.fitting_lighthouses = false;
  sample.max_age_seconds = -1;
  assert(std::strcmp(monitor.quality(sample, start + std::chrono::seconds(2)),
    "insufficient") == 0);
  sample.max_age_seconds = std::numeric_limits<double>::quiet_NaN();
  assert(std::strcmp(monitor.quality(sample, start + std::chrono::seconds(2)),
    "insufficient") == 0);
  sample.max_age_seconds = .2;
  sample.window_seconds = .1;
  assert(std::strcmp(monitor.quality(sample, start + std::chrono::seconds(2)),
    "insufficient") == 0);
  monitor = {};
  sample = {};
  assert(std::strcmp(monitor.quality(sample, start), "insufficient") == 0);

  sample = {};
  sample.connected = true;
  sample.residual_count = sample.peak_residual_count = 16;
  sample.peak_lighthouse_count = 2;
  sample.axis_count[0] = sample.axis_count[1] = 4;
  sample.axis_count[2] = sample.axis_count[3] = 4;
  sample.solver_status = 1;
  sample.window_seconds = .1;
  sample.max_age_seconds = .01;
  sample.rms_rad = .0001;
  sample.peak_rms_rad = .0001;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const struct
  {
    double age;
    double window;
    bool eligible;
  } window_cases[] = {
    {0, 0, true}, {.1, .1, true}, {.2, .1, false},
    {-1, .1, false}, {nan, .1, false}, {infinity, .1, false},
    {.01, -1, false}, {.01, nan, false}, {.01, infinity, false}
  };
  for (const auto & test : window_cases) {
    OpticalDiagnosticMonitor window_monitor;
    auto window_sample = sample;
    window_sample.sequence = 1;
    window_sample.max_age_seconds = test.age;
    window_sample.window_seconds = test.window;
    const auto observation = window_monitor.observe(window_sample, start);
    assert(observation.eligible == test.eligible);
    assert(std::strcmp(observation.state, test.eligible ? "normal" : "insufficient") == 0);
  }
  monitor = {};
  auto observe = [&](double seconds) {
      ++sample.sequence;
      return monitor.observe(sample, start + std::chrono::milliseconds(
          static_cast<int>(seconds * 1000)));
    };
  for (int i = 0; i < 4; ++i) {
    assert(std::strcmp(observe(i).trend, "learning") == 0);
  }
  auto result = observe(4);
  assert(std::strcmp(result.trend, "normal") == 0);
  assert(std::abs(result.baseline_rad - .0001) < 1e-10);
  sample.peak_rms_rad = .02;
  assert(std::strcmp(observe(5).trend, "normal") == 0);
  sample.rms_rad = .001;
  assert(std::strcmp(observe(6).trend, "rising") == 0);
  sample.rms_rad = .0001;
  assert(std::strcmp(observe(7).trend, "normal") == 0);
  sample.rms_rad = .001;
  assert(!observe(8).sustained);
  assert(!observe(9).sustained);
  assert(!observe(10).sustained);
  result = observe(11);
  assert(result.sustained && result.rise_seconds == 3);
  assert(std::abs(result.baseline_rad - .0001) < 1e-10);

  libsurvive_ros2::OpticalTrendSummary summary;
  summary.add(result);
  assert(std::strcmp(summary.state(), "unavailable") == 0);
  OpticalDiagnosticMonitor second;
  for (int i = 0; i <= 7; ++i) {
    sample.rms_rad = i < 4 ? .0001 : .001;
    ++sample.sequence;
    result = second.observe(sample, start + std::chrono::seconds(i));
  }
  assert(result.sustained);
  summary.add(result);
  assert(std::strcmp(summary.state(), "simultaneous_sustained") == 0);
  result = second.observe(sample, start + std::chrono::seconds(8));
  assert(!result.eligible && !result.sustained);
  summary = {};
  summary.add(result);
  summary.add(observe(12));
  assert(std::strcmp(summary.state(), "unavailable") == 0);

  // Latest coverage, not a well-supported interval peak, permits trend evaluation.
  sample.axis_count[3] = 0;
  assert(!observe(13).eligible);
  sample.axis_count[3] = 4;
  assert(!observe(14).sustained);
  assert(!observe(15).sustained);
  assert(!observe(16).sustained);
  assert(observe(17).sustained);
  assert(!observe(19).sustained);
  result = monitor.observe(sample, start + std::chrono::seconds(20));
  assert(!result.eligible && std::strcmp(result.state, "stale") == 0);
  assert(!observe(21).sustained);
  sample.solver_status = -1;
  assert(!observe(22).eligible);
  sample.solver_status = 1;
  assert(!observe(23).sustained);
  sample.rms_rad = .0001;
  assert(std::strcmp(observe(24).trend, "normal") == 0);
  ++sample.sequence;
  result = monitor.observe(sample, start + std::chrono::seconds(25), false);
  assert(!result.eligible && !std::isfinite(result.baseline_rad));
  for (int i = 26; i < 30; ++i) {
    assert(std::strcmp(observe(i).trend, "learning") == 0);
  }
  assert(std::strcmp(observe(30).trend, "normal") == 0);

  monitor = {};
  assert(std::strcmp(observe(31).trend, "learning") == 0);
  assert(std::strcmp(observe(33).trend, "learning") == 0);
  for (int i = 34; i < 37; ++i) {
    assert(std::strcmp(observe(i).trend, "learning") == 0);
  }
  assert(std::strcmp(observe(37).trend, "normal") == 0);
  sample.rms_rad = std::numeric_limits<double>::quiet_NaN();
  assert(!observe(38).eligible);
  sample.rms_rad = .0001;
  sample.connected = false;
  assert(!observe(39).eligible);
  sample.connected = true;
  assert(std::strcmp(observe(40).trend, "learning") == 0);
  for (int i = 41; i <= 44; ++i) {
    observe(i);
  }
  sample.sequence = 0;
  result = observe(45);
  assert(std::strcmp(result.trend, "learning") == 0);
  assert(!std::isfinite(result.baseline_rad));

  monitor = {};
  for (int i = 0; i < 5; ++i) {
    sample.rms_rad = i == 1 || i == 3 ? .01 : .0001;
    result = observe(50 + i);
  }
  assert(std::abs(result.baseline_rad - .0001) < 1e-10);
  sample.rms_rad = .00039;
  assert(std::strcmp(observe(55).trend, "normal") == 0);
  sample.rms_rad = .00041;
  assert(std::strcmp(observe(56).trend, "rising") == 0);
  sample.max_age_seconds = .2;
  assert(!observe(57).eligible);
  sample.max_age_seconds = .01;
  sample.residual_count = 7;
  assert(!observe(58).eligible);
  sample.residual_count = 16;
  sample.fitting_lighthouses = true;
  assert(!observe(59).eligible);
  sample.fitting_lighthouses = false;
  assert(!observe(60).sustained);

  summary = {};
  result = {};
  result.eligible = true;
  summary.add(result);
  summary.add(result);
  assert(summary.comparable == 0);
  assert(std::strcmp(summary.state(), "unavailable") == 0);
  result.baseline_rad = .0001;
  result.sustained = result.rising = true;
  summary.add(result);
  result.sustained = result.rising = false;
  summary.add(result);
  assert(std::strcmp(summary.state(), "single_sustained") == 0);
}
