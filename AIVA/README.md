# AIVA presence

## Central neural system packages

Packages grouped under `src/central_neural_system/` belong to AIVA's central neural system. For
example, `src/central_neural_system/cryptic_signal_core` is AIVA's cryptic ROS package, containing
its message definitions and encoder class.
Colcon discovers the package beneath the grouping directory; build it with:

```sh
cd ~/AGi/AIVA
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cryptic_signal_core
```

The shared WebUI backend uses AIVA's `CrypticSignalCore` when the AIVA package is
installed. `AGI_TELEPATHIC_SCHEME` selects a registered scheme (default
`ternary_utf8_v1`); its non-secret name travels in the signal so the receiving
TIC can select the matching decoder. Other deployments can retain their own
encoding implementation.

## AIVA chat and simple memory

Chat runs in the separate `aiva_chat` ROS package, not in SCS. It decodes
`/telepathic/stimulus`, sends recent turns to an OpenAI-compatible chat API,
and publishes `start` / `delta` / `done` response events on
`/tms/text_response`, which the WebUI already displays. Recent turns persist
in `~/.agi/aiva/conversation.sqlite3` (8 exchanges by default).

For a local Ollama server, set `AIVA_LLM_BASE_URL` and `AIVA_LLM_MODEL` if your
model or address differs from the defaults. Start Ollama, rosbridge, the AIVA
WebUI server, and the chat node. Build the AIVA packages from this workspace:

```sh
cd ~/AGi/AIVA
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cryptic_signal_core aiva_chat telepathic_management
source install/setup.bash
```

Start Ollama (or another compatible server) and load a model first. Then open
separate terminals, source `AIVA/install/setup.bash` in each, and run:

```sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1
ros2 run aiva_chat aiva_chat
ros2 run telepathic_management telepathic_console
```

Rosbridge must listen at `127.0.0.1:9090`; the console backend publishes the
input signal there, and the browser receives responses through its WebSocket
proxy. The default LLM endpoint is `http://127.0.0.1:11434/v1` and default
model is `llama3.2`. For an authenticated endpoint, set `AIVA_LLM_API_KEY` in
the environment; never put it in the ROS message or source files.

The AIVA ROS workspace can run a small heartbeat process alongside its future
services. It lets the Web Teleops Console on AuRoRA distinguish the PC companion from the
physical robot. When both AIVA's heartbeat and AuRoRA's rosbridge connection
are online, the Web Teleops Console shows Grace's VRM in the center with AuRoRA's original
pink robot beside her. When only AuRoRA is online, the robot returns to center
at regular size; if only AIVA is online, Grace appears by herself.

Install/build the AIVA workspace, configure the AuRoRA Web Teleops Console's Tailscale Serve
URL, and run this as a user service on the AIVA PC:

```sh
cd ~/AGi/AIVA
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scs
source install/setup.bash
export AGI_WEB_TELEOPS_CONSOLE_PRESENCE_URL="https://your-aurora-machine.your-tailnet.ts.net/api/presence/aiva"
ros2 run scs aiva-presence
```

Keep AIVA and AuRoRA on the same tailnet and use its ACL/grant rules to restrict
access to your devices. No second Web Teleops Console account is required. The heartbeat expires
12 seconds after the last successful request by default; the sender posts every
4 seconds. `AGI_PRESENCE_INTERVAL` changes the sender interval, and
`AGI_AIVA_PRESENCE_TIMEOUT` changes the Web Teleops Console expiration period.

To keep presence in sync with the PC power state, install the user service after
building the package:

```sh
mkdir -p ~/.config/agi ~/.config/systemd/user
printf '%s\n' 'AGI_WEB_TELEOPS_CONSOLE_PRESENCE_URL=https://your-aurora-machine.your-tailnet.ts.net/api/presence/aiva' > ~/.config/agi/aiva-presence.env
cp ~/AGi/AIVA/systemd/agi-aiva-presence.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now agi-aiva-presence.service
```

The service starts with your AIVA user session and stops when the PC shuts down.
Replace the URL with the HTTPS host shown by `tailscale serve status` on AuRoRA.
