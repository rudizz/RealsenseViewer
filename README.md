# RealsenseViewer with 2D Object detection

C++17 RealSense D455 viewer using librealsense for capture and OpenCV for display.
It has a 2D Object detection section where you can load a calibration image and it looks for it in the live Color image.

The Object detector supports the following options:
 - Feature Detector: SIFT, SURF, ORB.
 - Feature Matcher: FLANN, BruteForce.
 - Calib Image Downsample: x0.125 -> x5.0

The app opens an OpenCV dashboard with stream visibility checkboxes:
- Depth, color, infrared 1, infrared 2
- PCL-computed point cloud from the depth stream in a separate navigable 3D window
- Accel and gyro samples in a motion tile, when requested
- Click a stream name or checkbox to show/hide that stream
- Press number keys 1-9 to toggle the matching stream quickly
- Use the point cloud window mouse controls to rotate, pan, and zoom
- Open a calibration image and tick `Run object detection` to match it against the live Color image

## Requirements

- Windows or MacOS
- CMake 3.22+
- OpenCV
- PCL
- Intel RealSense SDK 2.0 / librealsense2
- A connected Intel RealSense D455

On macOS with Homebrew, the usual setup is:

```sh
brew install cmake opencv pcl librealsense
```

On Windows, install:

- Visual Studio 2022 with the Desktop development with C++ workload
- CMake 3.22+
- OpenCV with CMake config files, expected by the VS Code task at `C:\OpenCV\x64\vc16\lib`
- PCL with `PCLConfig.cmake` available through `PCL_DIR` or `CMAKE_PREFIX_PATH`
- Intel RealSense SDK 2.0

## Build

```sh
cmake -S . -B build
cmake --build build
```

## Tests

The project includes Unity-style C++ tests under `tests/`. CMake enables
testing by default through `BUILD_TESTING`.

There are two CTest targets:

- `realsenseviewer_queue_tests` covers the dependency-free concurrency queue.
- `realsenseviewer_module_tests` covers application flow, command-line parsing,
  frame bundle state, point-cloud settings, and object feature matching. This
  target is built when OpenCV, PCL, and librealsense2 development packages are
  available.

Build and run the tests with the normal viewer build:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

If you only want to run tests without building the viewer application,
configure a test-only build:

```sh
cmake -S . -B build-tests -DRSV_BUILD_VIEWER=OFF
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

Run one target by name with:

```sh
ctest --test-dir build-tests -R realsenseviewer_queue_tests --output-on-failure
ctest --test-dir build-tests -R realsenseviewer_module_tests --output-on-failure
```

To add a test, create or edit a file in `tests/`, include `unity.h`, define
test functions, and register them from `tests/test_module_main.cpp` or the
queue test `main()` with `RUN_TEST(...)`. Add new test source files to
`tests/CMakeLists.txt` when needed.

## Run

```sh
./build/RealsenseViewer
```

Optional arguments:

```sh
./build/RealsenseViewer --serial CAMERA_SERIAL
./build/RealsenseViewer --motion
./build/RealsenseViewer --auto-profiles
./build/RealsenseViewer --list-devices
```


On macOS setup, librealsense may only see the D455 with elevated USB
access. If `rs-enumerate-devices` works only with `sudo`, run:

```sh
sudo ./build/RealsenseViewer --serial 053122251294
```

In VS Code, use `Terminal > Run Task... > Run: viewer with sudo`.

On Windows, use:

```text
Terminal > Run Task... > Run: viewer Windows
```

For debugging, select `(Windows) Launch` in the Run and Debug panel. The
Windows configuration builds into `build/windows/Debug/RealsenseViewer.exe`.

## Debug With Sudo

Because this macOS setup only lets librealsense access the camera with `sudo`,
the debugger must also run elevated.

Use this VS Code debug configuration first:

```text
(sudo lldb-mi) Launch
```

It runs the Microsoft C++ extension's bundled `lldb-mi` through `sudo -A` and
uses a small macOS password dialog from `.vscode/sudo-askpass.sh`.

Another VS Code UI option is the CodeLLDB extension:

1. Install the `CodeLLDB` extension by Vadim Chugunov.
2. Select `(CodeLLDB sudo) Launch` in the Run and Debug panel.
3. Start debugging and enter your password in the integrated terminal.

If the UI debugger still refuses sudo, use the terminal debugger task:

```text
Terminal > Run Task... > Debug: sudo lldb terminal
```

At the LLDB prompt, useful commands are:

```lldb
breakpoint set --file RealsenseViewer.cpp --line 56
run
bt
continue
quit
```

Press `q` or `Esc` in the OpenCV dashboard to quit.


By default the app uses known D455 video profiles instead of asking the SDK to
enumerate every profile first. That path is more reliable on macOS USB stacks.
Use `--auto-profiles` when you want the SDK-driven stream discovery path.

## Troubleshooting

If the app prints `failed to set power state` or `No device connected`, check
the SDK before changing the application code:

```sh
rs-enumerate-devices
sudo rs-enumerate-devices
rs-fw-update -l
```

If macOS sees the camera in USB tools but only `sudo rs-enumerate-devices`
finds it, the viewer also needs sudo unless the underlying macOS/libusb access
issue is resolved. Try a direct USB-C connection, a different USB 3 data cable,
unplug/replug the camera, and rerun the SDK tools from Terminal.

## Architecture

- `IFrameSource` is the camera abstraction.
- `RealSenseFrameSource` implements `IFrameSource` with librealsense and owns the capture/processing worker lifecycle.
- The capture worker waits for RealSense SDK frames and writes raw framesets into a bounded FIFO.
- `RealSenseFrameProcessor` converts raw RealSense framesets into display-ready video, motion, and point-cloud bundles on a separate worker thread.
- `ConcurrentQueue` is the bounded FIFO abstraction; when a queue is full, the newest frame is dropped to keep latency bounded.
- `IFramePresenter` is the display abstraction.
- `OpenCvFramePresenter` implements `IFramePresenter` with an OpenCV dashboard.
- `Application` owns the run loop and only depends on the interfaces.

## Data Flow

The viewer keeps capture, conversion, and presentation on separate steps so a
slow display frame does not block the camera pipeline.

1. `RealSenseFrameSource::captureLoop()` waits for `rs2::frameset` values from
   librealsense and pushes each raw frameset into a small
   `ConcurrentQueue<rs2::frameset>`.
2. `RealSenseFrameSource::processingLoop()` waits on that capture FIFO, passes
   each frameset to `RealSenseFrameProcessor`, and converts it into a
   `FrameBundle` containing display-ready video frames, motion samples, and
   optional point-cloud data.
3. The processed bundle is pushed into a second
   `ConcurrentQueue<FrameBundle>`.
4. The application loop calls `IFrameSource::poll()`. When a processed bundle is
   available, `OpenCvFramePresenter` updates the dashboard and point-cloud
   viewer; otherwise the presenter idles and keeps handling window events.

Both queues are custom bounded concurrent FIFOs. They preserve FIFO ordering for
accepted items, use a mutex and condition variable for safe producer/consumer
handoff, and can be closed to let worker threads exit cleanly. Their bounded
capacity is deliberate: camera frames are time-sensitive, so letting an
unbounded backlog grow would make the UI show old data and increase memory
pressure.

When a FIFO is full, `tryPush()` rejects the newest item and increments the
queue's drop counter. This means the worker that is already behind can continue
draining the older accepted frames without being forced to allocate more memory
or block the RealSense capture thread. In practice, transient slowdowns in
point-cloud conversion, object detection, or OpenCV rendering turn into explicit
frame drops instead of runaway latency. The capture FIFO currently buffers a few
raw framesets, while the presentation FIFO is even smaller so the dashboard
stays close to live camera time.
