# ROS2 car with bazel

## Build ArchlinuxARM based distribution for Raspberry PI

```
$ cd system
$ ./build.sh
$ ./write_sdcard.sh /dev/mmcblkX
```

## Build and run ROS2 apps
```sh
# build and run
$ bazel run //:car.launch
# generate rust-project.json for rust-analyzer in VSCode
$ bazel run @rules_rust//tools/rust_analyzer:gen_rust_project
```
