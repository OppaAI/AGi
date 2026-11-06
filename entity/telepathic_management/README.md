# Telepathic Management

Shared ROS 2 package for telepathic input and the Web Teleops Console. Package
name is the noun `telepathic_management`; executables are actions: `telepathic_input`
and `telepathic_console`.

The input node accepts `std_msgs/msg/String` on `/telepathic/input`. Payloads
may be plain text or JSON with a `text` field. It publishes canonical SSS JSON
on `/telepathic/stimulus`. Topic names and size/repeat limits are ROS parameters.

The console serves the shared frontend from this package and proxies its
WebSocket to local rosbridge at `ws://127.0.0.1:9090` by default. Override the
interface directory with `AGI_WEB_TELEOPS_CONSOLE_INTERFACE_DIR`; set
`AGI_ROSBRIDGE_URL` to select the local bridge.

Both AuRoRA and AIVA include this package in their colcon workspaces through
`src/telepathic_management`. Build with `colcon build --packages-select
telepathic_management` and run either executable with `ros2 run
telepathic_management <executable>`. The package provides shared input and UI
transport; assistant-specific SCS adapters and AIVA LLM/chat behavior remain
separate integrations.
