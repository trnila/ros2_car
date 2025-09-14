# ROS2 car

```sh
$ wget https://github.com/bazelbuild/bazelisk/releases/download/v1.27.0/bazelisk-linux-arm64 -O ~/.local/bin/bazel
$ chmod a+x ~/.local/bin/bazel
$ bazel run @rules_rust//tools/rust_analyzer:gen_rust_project
$ ros2 launch ./car.launch.py
```
