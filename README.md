# libsurvive_ros2
A ROS 2 driver for Lighthouse 1.0 and 2.0 trackers using
[libsurvive](https://github.com/cntools/libsurvive), without SteamVR.
It publishes tracker poses, sensor data, and tracking readiness.

This fork of [asymingt/libsurvive_ros2](https://github.com/asymingt/libsurvive_ros2)
adds fixed-frame calibration and tracking diagnostics. CMake fetches a pinned
libsurvive revision and applies the patches in `patches/` during the build.

## Installation instructions

The supplied Docker image targets ROS 2 Jazzy. The same package can be built in
a native ROS 2 workspace.

Before you do anything, you will need to install these udev rules and reload the subsystem.

```
sudo curl -fsSL https://raw.githubusercontent.com/cntools/libsurvive/master/useful_files/81-vive.rules \
    -o /etc/udev/rules.d/81-vive.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

You can now choose to build the driver natively or in a container. The benefit of launching it within a container is that it won't interfere with any pre-existing ROS installation on your machine. However, you will need docker-ce and the compose plugin for things to work.

### Devcontainer build and test

You can open this project as a [Visual Studio Code Devcontainer](https://code.visualstudio.com/docs/devcontainers/containers). It will then prompt you for a target ROS distribution in the drop-down menu at the top of the IDE. When you select your desired distribution, the devcontainer engine will build a container for this ROS distribution, install our project dependencies, and compile the code. You can just open up a terminal and run:

```sh
ros2 launch libsurvive_ros2 libsurvive_ros2.launch.py
```

Volume binding of the source tree happens with the correct permission. So, if you edit the source code in the container folder `/home/ros/ros2_ws/src/libsurvive_ros2` your changes will propagate to your host folder, where you can eventually push them upstream. It is also possible to use git within the container, but this is more complicated and not recommended.

### Container build and test

Install docker and docker-compose: https://docs.docker.com/engine/install/ubuntu/

```sh
$ docker compose build
$ docker compose run libsurvive_ros2 colcon test
```

The image builds this checkout in a ROS 2 Jazzy workspace. To build for arm64:

```sh
$ docker compose build --build-arg ARCH=arm64 --build-arg ROS_DISTRO=jazzy
$ docker compose run libsurvive_ros2 colcon test  # optional, to test the package
```

Note that if you are building on a different architecture than the host you must follow the docker/QEMU installation instructions here before running the command above. Here is a link to a document outlining how this is done: https://docs.nvidia.com/datacenter/cloud-native/playground/x-arch.html

### Native build and test

Install ROS 2 Jazzy before building this package.

Install the native dependencies:

```sh
sudo apt-get install build-essential \
    cmake \
    freeglut3-dev \
    liblapacke-dev \
    libopenblas-dev \
    libpcap-dev \
    libusb-1.0-0-dev \
    libx11-dev \
    zlib1g-dev
```

Finally source ROS2, create a workspace, checkout, compile and test:

```sh
$ source /opt/ros/jazzy/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd  ~/ros2_ws/src
$ git clone https://github.com/Tommoro-AI/libsurvive_ros2.git
$ cd ..
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build
$ colcon test  # optional, to test the package
$ source install/setup.bash
```

## Run the driver

To run the driver on containerized installations do the following:

```sh
$ docker compose up
```

Alternatively, to run the driver on native installations run the following:

```sh
$ ros2 launch libsurvive_ros2 libsurvive_ros2.launch.py
```

There are launch arguments to `libsurvive_ros2.launch.py` to help get up and running:

- `namespace = string (default: 'libsurvive')` : This is the namespace on which to add the extra topics for sensor data.
- `tracking_frame = string (default: 'libsurvive_frame')`: Parent frame for tracker and Lighthouse transforms.
- `force_recalibrate = boolean (default: true)`: Recompute Lighthouse poses at startup. Keep trackers stationary and visible during initialization; use false only for an unchanged installation.

Occlusion monitor behavior:

- `/<namespace>/<occlusion_topic>` publishes `libsurvive_ros2/msg/OcclusionStatus`.
- `header.frame_id` contains the device serial (same style as `/imu`), and `occluded` is the per-device occlusion flag.
- `occluded=true` means the corresponding tracked non-lighthouse device is considered occluded.
- The driver estimates occlusion from libsurvive internal timing by tracking age of each object's latest optical update (`last_light`) against current libsurvive runtime.
- Occlusion thresholds and debounce counts are intentionally fixed in code for stable behavior.

Velocity and battery behavior:

- `/<namespace>/<velocity_topic>` publishes `geometry_msgs/TwistStamped`.
- `header.frame_id` is the device serial; `twist.linear` and `twist.angular` are expressed in the device local frame.
- `/<namespace>/<battery_topic>` publishes `sensor_msgs/BatteryState`.
- `header.frame_id` is the device serial; `percentage` and charging/discharging status are populated from libsurvive object state.

## Calibration and troubleshooting

- **I don't see any data streaming** Examine the console log. If you see a LIBUSB error, chances are high that you either haven't installed the udev rules, or you haven't mounted the /dev/bus/usb volume correctly into the docker container.

- **How do I configure this for my specific tracker ID?** There's no need -- the libsurvive driver will enumerate all devices, query their ID and publish this ID as the transform name using the TF2 standard topic `/tf`. Base station positions change less frequently, and so they are published at a lower rate on `/tf_static`.

- **The base stations locations are not where I'd expect them to be** -- The calibration phase of libsurvive works out the relative location of the base stations automatically, but the configured `tracking_frame` is not map-aligned by default. Its default is `libsurvive_frame`; mocap bringup uses `libsurvive_world`. Applications that require `map` must provide the corresponding alignment transform.

- Startup automatically selects stations with recent optical observations and fixes their calibrated poses after 2 seconds of stability. Each new native context requires freshly received OOTX from each observed station; cached OOTX alone is insufficient. Calibration then freezes in the same context: the global solver worker stops, pending calibration results are discarded, and device connections, tracker filters, OOTX information and the validated station baseline remain intact. The retained layout is checked again before tracker output starts. No station count is configured. `force_recalibrate:=false` reuses the saved configuration for an unchanged installation, but still refreshes OOTX. An explicit recalibration creates a new context and repeats these checks.
- `~/ready` reports usable optical tracking; `~/calibration_valid` reports whether the station layout is retained. Both use `std_msgs/Bool`, reliable, transient-local, with a 200 ms heartbeat. `~/recalibrate` (`std_srvs/Trigger`) requests calibration only when the layout is invalid, including during startup. Consumers must stop motion before requesting recalibration and explicitly resume after readiness returns.
- A newly observed station, an invalidated/changed calibrated pose, or a changed OOTX up direction invalidates the fixed frame until recalibration. OOTX changes invalidate calibration even if optical input continues and the SDK retains the reference station's pose. Cached SDK station entries alone do not establish visibility. Missing optical observations set `ready=false` and withhold tracker poses, but preserve `calibration_valid=true` regardless of the occlusion duration. Returning stations must match the retained layout before output resumes; teleoperation consumers must also require stable input and a new RUN request. A power interruption cannot be distinguished from occlusion when the same station information returns.
- Tracker poses are published without smoothing. Consumers handle filtering and stale or discontinuous input. Station loss cannot be distinguished from prolonged occlusion of every tracker. Physical movement that leaves the SDK's calibration unchanged, including translation with a single station, is not guaranteed to be detected. Recalibration can change the world frame and requires updating external alignment.

### Optical consistency logging

Native SDK messages, including decoder warnings, are saved in the launch
session's `launch.log` as well as shown on screen. USB statistics are disabled
by default. For a transport investigation, append `--usb-hz-output 1` to the
node's `driver_args` parameter. Match `Iface` names with the optical diagnostic's
`device` field and use `serial` to identify the physical tracker. Packet counts
measure USB reports. `Max CB Latency` measures submission-to-callback time,
including host event handling, and does not isolate radio latency.

The driver emits `Optical diagnostic only` once per device per second. This is
observation only. It does not change tracking poses, TF publication, readiness,
calibration, timeout handling, or robot safety checks.

The patched SDK copies the existing MPFIT optical residual statistics after a
solve, including failed solves. `rms_rad` is the unweighted angular RMS from the
last native residual evaluation. It is not a new residual calculation or a
promise that the final published pose has this error. `residuals`, `axes` (SDK
station index followed by horizontal/vertical counts), `solver_status`, device
`device_ticks`, measurement age, and the SDK's adaptive measurement window
provide context. `device` is the immutable SDK object name (for example `WM0`),
and `serial` is the configuration-updated hardware serial. Device ticks are not
wall-clock timestamps.

`peak_rms_rad`, its residual/station support, and fit/failure/invalid counters
cover the interval since the previous log. A brief high residual remains visible
even if the latest fit has recovered. The other fields describe the latest fit,
so their coverage can differ from the interval peak. Logging drains diagnostic
counters under the SDK lock and formats/writes the message after releasing it.
No additional residual evaluation or logging runs in the optimizer. The `fits`
counter includes rejected solver setup attempts, so it does not by itself prove
that an optimizer solve ran.

The provisional labels have no control effect:

- `normal` means the interval peak has at least eight residuals from at least two
  stations with both axes and angular RMS at most 0.01 rad (about 0.57 degrees).
- `suspect` means that supported peak exceeds 0.01 rad, a solve failed, or a solve
  produced unavailable/nonfinite residual statistics.
- `insufficient` means no fit yet, calibration fitting is active, measurement
  timing is invalid/outside the native window, or peak coverage is inadequate.
- `stale` means no new fit sequence was observed for at least one steady-clock
  second. It does not refer to ROS time or convert device ticks to an epoch.

The 0.01 rad threshold is an initial logging aid, not a validated detector of
base-station movement. Occlusion, reflections and poor geometry can also raise
residuals. A coherent coordinate shift or a single usable station can leave low
residuals despite incorrect world coordinates. Native filtering can discard
inconsistent observations before these residuals are reported. Interval
`dropped_measurements` and `dropped_lighthouses` sum the native filter counters
across fits; they are not counts of unique sensors or stations. Compare normal
motion, occlusion and intentional station-change logs before considering any
future blocking policy. Recalibration clears the diagnostic history with the SDK
context; SDK disconnect clears the native snapshot and a device-add event clears
its reader history. Disconnected entries are skipped without reading freed native
objects. `stale` and latest-fit `insufficient` conditions take priority
over the interval `suspect` label. Raw interval peaks and failure counts are still
printed when the latest fit is unavailable or calibration has started.


Each per-device line also reports `trend`, `baseline_rms_rad`, `rise_ratio`,
`rise_seconds` and `trend_eligible`. These are independent of the interval quality
`state` above. Baseline learning starts after calibration is valid and frozen.
It uses the median of five fresh eligible observations spanning at least four
steady-clock seconds. The baseline remains fixed until calibration invalidation,
a new context or device reconnection. Ordinary occlusion retains the baseline.
The median tolerates up to two outlying samples, but movement during learning can
contaminate the baseline; this diagnostic cannot identify that condition.

Trend eligibility requires a new fit sequence, a successful latest solve, at
least eight latest residuals, both axes from at least two stations, finite
nonnegative latest RMS, and valid measurement timing within the native window.
Interval peak coverage does not substitute for latest coverage. `trend` is
`unavailable` without eligible data, `learning` until the baseline is acquired,
and `normal` below the rise threshold. The provisional rise threshold is the
larger of four times baseline and baseline plus 0.0003 rad. `rising` becomes
`sustained` after at least three steady-clock seconds of consecutive eligible
observations. Invalid/repeated observations and gaps longer than 1.5 seconds
break persistence and reset incomplete baseline learning. Acquired baselines
remain intact across these gaps. An isolated interval peak does not count as a
sustained rise.

`Optical trend only` summarizes only eligible observations from the current log
pass whose baseline has been acquired. It prints comparable, rising and
sustained tracker counts. With fewer than two comparable trackers its state is `unavailable`; otherwise it is
`simultaneous_sustained` for at least two sustained rises, `single_sustained` for
one, or `normal`. A disconnected or stale tracker cannot contribute a cached
verdict. Simultaneous residual growth is evidence of optical inconsistency, not
proof of station motion. These thresholds need validation against varied normal
movement and occlusion; no trend label blocks input or changes safety behavior.
