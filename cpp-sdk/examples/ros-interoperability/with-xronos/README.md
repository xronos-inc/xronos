# ROS example with Xronos

## Getting started

The following commands build and run the example.

```bash
docker compose build
docker compose up
```

This build involves building gRPC from source in two separate Docker builds, in
parallel. The build completed successfully on a machine with 16 GiB of RAM with
the default settings on Ubuntu for swapping and overcommitting memory, but you
may need to reduce concurrency in the colcon build or build one image at a time
in order to get the build to work on your system.
