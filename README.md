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
- `BoundedQueue` is the FIFO abstraction; when a queue is full, the newest frame is dropped to keep latency bounded.
- `IFramePresenter` is the display abstraction.
- `OpenCvFramePresenter` implements `IFramePresenter` with an OpenCV dashboard.
- `Application` owns the run loop and only depends on the interfaces.
