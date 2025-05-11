# Xronos/ROS 2 interoperability

This example demonstrates how the Xronos framework may interact with ROS 2.
This allows programs written using the Xronos framework to communicate with
other ROS nodes by delegating interprocess communication to `rclcpp`.

The `without-xronos` directory shows sample ROS 2 code that is closely based on [this
tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).
This example simply demonstrates the publish/subscribe tutorial running in Docker without Xronos.

The `with-xronos` directory shows sample code that lets `xronos` and `rclcpp`
coexist within the same process, delegating interprocess communication to
`rclcpp`. Similar to the `without-xronos` example two ROS nodes are defined.
However, the nodes host a xronos program that implements the application logic.

## License

This example is based on an example from [the ROS 2 documentation](https://github.com/ros2/ros2_documentation/blob/jazzy/LICENSE), which is licensed under the Creative Commons Attribution 4.0 International license.

Changes are licensed under the BSD 3-clause license.
