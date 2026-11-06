# Telepathic Management

Shared ROS 2 package for the Web Teleops Console transport. Its package name is
the noun `telepathic_management`; its executable is `telepathic_console`.

The WebUI backend encodes each message with the shared cryptic codec and
publishes a typed `cryptic_signal_core/msg/TelepathicSignal` through rosbridge
on `/telepathic/stimulus`. Each assistant's TIC subscribes to that topic and
decodes locally before sending the text into its own SCS. The decoded message
does not need another ROS topic.

The console under `telepathic_console/` serves its shared frontend and proxies its
WebSocket to local rosbridge at `ws://127.0.0.1:9090` by default. Override the
interface directory with `AGI_WEB_TELEOPS_CONSOLE_INTERFACE_DIR`; set
`AGI_ROSBRIDGE_URL` to select the local bridge.

Both AuRoRA and AIVA include this package in their colcon workspaces through
`src/telepathic_management`. Build with `colcon build --packages-select
telepathic_management` and run either executable with `ros2 run
telepathic_management <executable>`. The package provides shared input and UI
transport; assistant-specific SCS adapters and AIVA LLM/chat behavior remain
separate integrations.
