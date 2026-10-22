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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include "libsurvive/survive.h"

namespace libsurvive_ros2
{

struct OpticalDiagnosticObservation
{
  const char * state{"insufficient"};
  const char * trend{"unavailable"};
  double baseline_rad{std::numeric_limits<double>::quiet_NaN()};
  double rise_seconds{0};
  bool eligible{false};
  bool sustained{false};
  bool rising{false};
};

struct OpticalTrendSummary
{
  unsigned comparable{0};
  unsigned rising{0};
  unsigned sustained{0};

  void add(const OpticalDiagnosticObservation & observation)
  {
    if (!observation.eligible || !std::isfinite(observation.baseline_rad)) {return;}
    ++comparable;
    rising += observation.rising;
    sustained += observation.sustained;
  }

  const char * state() const
  {
    if (comparable < 2) {return "unavailable";}
    if (sustained >= 2) {return "simultaneous_sustained";}
    return sustained ? "single_sustained" : "normal";
  }
};

struct OpticalDiagnosticMonitor
{
  using Clock = std::chrono::steady_clock;
  std::uint64_t sequence{0};
  Clock::time_point last_update{};

  std::array<double, 5> baseline_samples{};
  std::size_t baseline_count{0};
  double baseline_rad{std::numeric_limits<double>::quiet_NaN()};
  Clock::time_point baseline_start{};
  Clock::time_point rise_start{};
  bool rising{false};

  OpticalDiagnosticObservation observe(
    const SurviveOpticalDiagnostic & sample, Clock::time_point now, bool calibrated = true)
  {
    if (!calibrated || !sample.connected || sample.sequence < sequence) {*this = {};}
    const bool fresh = sample.sequence != 0 && sample.sequence != sequence;
    const bool gap = sequence != 0 && now - last_update > std::chrono::milliseconds(1500);
    OpticalDiagnosticObservation result;
    result.state = quality(sample, now);
    result.baseline_rad = baseline_rad;
    unsigned complete_stations = 0;
    for (int lh = 0; lh < NUM_GEN2_LIGHTHOUSES; ++lh) {
      complete_stations += sample.axis_count[2 * lh] > 0 && sample.axis_count[2 * lh + 1] > 0;
    }
    result.eligible = calibrated && sample.connected && fresh && !sample.fitting_lighthouses &&
      sample.solver_status > 0 && sample.residual_count >= 8 && complete_stations >= 2 &&
      std::isfinite(sample.rms_rad) && sample.rms_rad >= 0 &&
      withinWindow(sample);
    if (!result.eligible || gap) {
      rising = false;
      if (!std::isfinite(baseline_rad)) {baseline_count = 0;}
      if (!result.eligible) {return result;}
    }
    if (!std::isfinite(baseline_rad)) {
      if (baseline_count == 0) {baseline_start = now;}
      // Log cadence is one second; retain the first four samples until the span is sufficient.
      if (baseline_count < baseline_samples.size() - 1 ||
        now - baseline_start >= std::chrono::seconds(4))
      {
        baseline_samples[baseline_count++] = sample.rms_rad;
      }
      if (baseline_count < baseline_samples.size()) {
        result.trend = "learning";
        return result;
      }
      std::sort(baseline_samples.begin(), baseline_samples.end());
      baseline_rad = baseline_samples[baseline_samples.size() / 2];
      result.baseline_rad = baseline_rad;
    }
    // Provisional log-only thresholds require validation against varied normal motion.
    if (sample.rms_rad >= std::max(4 * baseline_rad, baseline_rad + 0.0003)) {
      if (!rising) {rise_start = now;}
      rising = true;
      result.rising = true;
      result.rise_seconds = std::chrono::duration<double>(now - rise_start).count();
      result.sustained = now - rise_start >= std::chrono::seconds(3);
      result.trend = result.sustained ? "sustained" : "rising";
    } else {
      rising = false;
      result.trend = "normal";
    }
    return result;
  }

  const char * quality(const SurviveOpticalDiagnostic & sample, Clock::time_point now)
  {
    if (sample.sequence != sequence) {
      sequence = sample.sequence;
      last_update = now;
    }
    if (sequence == 0) {return "insufficient";}
    if (now - last_update >= std::chrono::seconds(1)) {return "stale";}
    if (sample.residual_count == 0 || sample.fitting_lighthouses ||
      !withinWindow(sample))
    {
      return "insufficient";
    }
    if (sample.failed_fits || sample.invalid_fits) {return "suspect";}
    if (sample.peak_residual_count < 8 || sample.peak_lighthouse_count < 2) {
      return "insufficient";
    }
    // Provisional logging threshold; this is not a station-motion detector.
    return !std::isfinite(sample.peak_rms_rad) || sample.peak_rms_rad > 0.01 ?
           "suspect" : "normal";
  }

private:
  static bool withinWindow(const SurviveOpticalDiagnostic & sample)
  {
    return std::isfinite(sample.max_age_seconds) && std::isfinite(sample.window_seconds) &&
           sample.max_age_seconds >= 0 && sample.max_age_seconds <= sample.window_seconds;
  }
};

}  // namespace libsurvive_ros2
