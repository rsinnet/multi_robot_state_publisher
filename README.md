# multi_robot_state_publisher

Reduce the compute required to publish frames for multiple robots.

## multi_robot_state_publisher_node

Read robot configs from the Parameter Server and publish frames for them.

### Synopsis

```
multi_robot_state_publisher_node ~publishing_rate:=50
```

### Subscribed Topics

- `/joint_states`: Gets joint data for a robot.

### Published Topics

- `/tf`: Publishes frame data to the dynamic TF tree.

- `/tf_static`: Publishes latched frame data once to the static TF tree.

### Parameters

- `~robots/<robot>` (`struct`, default: {}): Configuration settings for each
  robot. Has the following form:

  - `robot_description` (`str`): The URDF of the robot in
    XML format. Typically this is loaded either from a file or with the `xacro`
    tool to convert a XACRO file into an URDF.

  - `publish_frequency` (`float`, default: 50.0): The rate at which
    this robot will update its joints. If this is higher than the node publish
    rate, it will be capped. If it is lower, it will still publish at the
    higher rate but the data for this robot will only be updated at this rate.

  - `use_tf_static` (`bool`, default: true): If true, publish fixed transforms
    to the `/tf_static` topic instead of `/tf`.

  - `ignore_timestamp` (`bool`, default: true): If true, joint_states messages
    are accepted, no matter their timestamp.

  - `tcp_nodelay` (`bool`, default: false): If true, tell the joint_states
    publisher to set the TCP_NODELAY socket option, which disables the Nagle
    algorithm. Doing so causes segments to be sent as soon as they are ready
    instead of bundling them. This means there will be more frames with smaller
    packets which translates to poorer network utilization but lower latency
    for the joint_states topic, which may be important for real-time control
    applications. See also:
    [tcp(7)](https://man7.org/linux/man-pages/man7/tcp.7.html).

    `tf_prefix` (`str`, default: ""): A prefix to add to the tf frame which
    prevents namespace collisions for multiple robots.
