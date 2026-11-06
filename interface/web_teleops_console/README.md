# Web Teleops Console (Web Teleops Console)

The Web Teleops Console is a browser interface served by AuRoRA, so it remains available when
the AIVA PC is off. The same static interface can be hosted by another AGi
computer; only its backend needs to proxy a WebSocket to that host's rosbridge.

Build and source the ROS 2 workspace, then run `ros2 launch ras ras.launch.py`.
The launch starts Web Teleops Console on `127.0.0.1:8853`; it also starts the existing HTTP API.
Run rosbridge locally and keep its port private to the robot:

```sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1
```

Expose Web Teleops Console to your tailnet with Tailscale Serve:

```sh
tailscale serve --bg 8853
```

Open the HTTPS URL reported by `tailscale serve status`. No Web Teleops Console-specific login
is used: Tailscale identity and the tailnet ACL/grant rules decide which users
can reach the machine. Keep the Web Teleops Console listener and rosbridge listener on loopback;
do not expose port 8853 or 9090 directly on the LAN or public internet. Configure
the tailnet rules to allow only your account/device to reach this machine.

The UI is split by responsibility: `frontend/` contains the chat and settings
pages, while `backend/static/` contains the served CSS and JavaScript, and
`backend/static/assets/` contains the VRM, favicon, and asset notes. The FastAPI/ROS bridge backend lives in
`AuRoRA/src/tms/tms/web_teleops_console.py` and serves this UI.
Open `/settings` to edit HRM parameters marked `[INTRINSIC]`. They are validated
and saved to `~/.agi/ras/aurora.yaml`; restart the AuRoRA ROS launch to apply
them. AGi currently has no dynamic parameter-update callback, so these values
cannot take effect in a running ROS process. Browser theme changes apply
immediately and are kept in browser storage. Environment settings are shown
read-only; edit the service environment and restart its process to change them.

AIVA's `aiva-presence` process posts a heartbeat through the Tailscale Serve
URL every four seconds. AuRoRA's live rosbridge connection supplies its online
state. When both are online, the UI loads `/static/assets/Grace.vrm` for Grace in the
center and moves AuRoRA's original pink robot beside her at a smaller size.
When only AuRoRA is online, its original robot returns to the center at its
usual size. When only AIVA is online, Grace appears alone in a close chest-up
portrait with gentle breathing, head sway, and occasional blinks. The paired
view gives both characters more room. Each Grace message uses a small CSS-scale
copy of the AuRoRA robot.

Typed chat text is sent as `nes/msg/RadioSignal` on `/tms/radio_input`. The
Web Teleops Console posts the original UTF-8 text to its same-origin Python encoder endpoint;
the endpoint returns packed binary signal data, which the browser publishes
through rosbridge as the custom ROS message. TIC decodes it in Python, applies
the existing sensory gates, and forwards normalized SSS to CNC. Each UTF-8
byte becomes six base-3 signal symbols (`-1`, `0`, `+1` mapped to digits
`0`, `1`, `2`), packed five symbols per message byte. This is a reversible
radio-inspired digital encoding, not a physical RF waveform; it is larger
than raw UTF-8 and is for the signal-format experiment, not a speedup. Enabling
Web prepends `/web ` before encoding. The destination `radio_channel` is a
plain routing label, not authentication or encryption. Tailnet ACLs remain the
access boundary. The `/web` directive is preserved through TIC, but CNC does
not yet execute web search from that directive.

The theme control follows Aidon's light and dark palettes and saves your choice
in browser storage.
Three.js, three-vrm, and roslib load from jsDelivr, so the browser needs internet
access for those libraries. The VRM itself is served locally.
