# RealsenseViewer

C++17 RealSense D455 viewer using librealsense for capture and OpenCV for display.

The app opens one OpenCV window for every D455 video stream it enables:

- Depth, color, infrared 1, infrared 2
- Accel and gyro samples in a small motion panel, when requested

## Requirements

- CMake 3.22+
- OpenCV
- Intel RealSense SDK 2.0 / librealsense2
- A connected Intel RealSense D455

On macOS with Homebrew, the usual setup is:

```sh
brew install cmake opencv librealsense
```

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

Press `q` or `Esc` in any OpenCV window to quit.


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
- `RealSenseFrameSource` implements `IFrameSource` with librealsense.
- `IFramePresenter` is the display abstraction.
- `OpenCvFramePresenter` implements `IFramePresenter` with OpenCV windows.
- `Application` owns the run loop and only depends on the interfaces.
