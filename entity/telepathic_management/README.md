# Telepathic Management

Shared ROS 2 package for telepathic input normalization. Package name is the
noun `telepathic_management`; its executable is the action `telepathic_input`.

The node accepts `std_msgs/msg/String` on `/telepathic/input`. The payload may
be plain text or JSON with a `text` field. It publishes canonical SSS JSON on
`/telepathic/stimulus`, which an assistant adapter can translate into its own
SCS input contract. Topic names and size/repeat limits are ROS parameters.

To build it in either workspace, include this package directory under that
workspace's `src` directory (for example, with a symlink to this directory),
then run `colcon build --packages-select telepathic_management`. Launch with
`ros2 run telepathic_management telepathic_input`.

This package does not yet provide the WebUI, an LLM connection, assistant
adapters, cross-assistant messaging, or radio-signal decoding.
