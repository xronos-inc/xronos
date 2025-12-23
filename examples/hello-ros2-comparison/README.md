# Hello World Demo

This demo compares two simple "Hello, World!" programs written in Xronos and
ROS2. The program consists of 2 nodes/reactors called `Hello` and `Printer`.
`Hello` sends a single message containing "Hello, World!" after startup and then
shuts down. `Printer` prints any message it receives and then terminates. This
simple program is surprisingly difficult to implement in ROS2 as we need to
coordinate the startup order between the two nodes.


## Xronos Implementation

The Xronos program is a straightforward implementation consisting of two
reactors. No additional coordination is required.

Run the demo using the following commands:

```sh
cd xronos
./run_demo.sh
```

## ROS2 Implementation

The ROS2 implementation is a bit more involved as it requires launching separate
processes for the `hello` and `printer` nodes. Moreover, we have to consider the
following:

- Publishing a message directly in the node constructor is not possible. So we
  have to use a timer to invoke a callback later.
- If a small period is used for the timer, the message is not published
  correctly. It is unclear why this is the case. A period of at least 0.1s seems
  to work.
- The order in which the nodes start is important. If `Hello` publishes the message
  before `Printer` has configured its subscription, it does not see the message.
  In this case, we print an error message. This is somewhat mitigated by the
  fact that `Hello` waits 0.1s before publishing, but this is not a guarantee.
  While ROS2 offers several mechanisms for managing startup order (e.g., event
  handlers in the launch file or lifecycle nodes), managing those is tedious,
  and complexity drastically increases for larger programs.
- Setting a QoS profile can help mitigate the startup order issue. When
  setting durability to `TRANSIENT_LOCAL`, the publisher keeps a copy of the
  last message and sends it to new subscribers. However, this copy is only
  available as long as the publisher node is still running. Since we are shutting down
  the node after publishing, `TRANSIENT_LOCAL` has no effect. By keeping the
  node alive for longer (e.g., by using a sleep command), we can increase the
  likelihood that the message will be received correctly, but there is no
  guarantee. Moreover, `TRANSIENT_LOCAL` only preserves the last message. In a
  scenario where we want to send multiple messages in quick succession, it does
  not provide a reliable solution for guaranteeing that messages are received.

Run the demo using the following commands:

```sh
cd ros
./run_demo.sh
```

## Comparison

The demo scripts run the application multiple times (25 by default) and report some
measurements. This includes the average execution time as well as the number
of synchronization errors that occurred. That is, the `Printer` node did not
receive the "Hello, World!" message, likely because it started after `Hello`
already terminated.

Below are results obtained on our developer machine. Note that measurements may
vary depending on your hardware and OS.

| | Xronos | ROS |
| --- | --- | --- |
| number of files | 1 | 7 |
| loc (excluding package files) | 23 | 91 |
| error rate | 0.0% | >10% |
| average execution time | 0.035s | 0.664s

## Conclusion

Coordinating nodes in ROS2 can be surprisingly challenging. A program consisting
of two nodes and a single message gives plenty of room for errors, even when
following best practices (Configuring QoS and writing launch scripts).

With Xronos, you do not need to worry about concurrency bugs. You can focus
solely on the application logic and let the Xronos runtime take care of the
coordination. As the measurements show, Xronos not only makes your application
more reliable, but also faster and more concise.

While in this demo the Xronos program executes as a single binary, the Xronos 
runtime can achieve the same properties also for distributed applications.
If you are interested in learning more, [book a demo](https://app.onecal.io/b/marten/demo).


