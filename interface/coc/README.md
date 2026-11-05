# Communication Operation Console (COC)

The COC is a browser interface served by AuRoRA, so it remains available when
the AIVA PC is off. The same static interface can be hosted by another AGi
computer; only its backend needs to proxy a WebSocket to that host's rosbridge.

Build and source the ROS 2 workspace, then run `ros2 launch ras ras.launch.py`.
The launch starts COC on `127.0.0.1:8853`; it also starts the existing HTTP API.
Run rosbridge locally and keep its port private to the robot:

```sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1
```

Expose COC to your tailnet with Tailscale Serve:

```sh
tailscale serve --bg 8853
```

Open the HTTPS URL reported by `tailscale serve status`. No COC-specific login
is used: Tailscale identity and the tailnet ACL/grant rules decide which users
can reach the machine. Keep the COC listener and rosbridge listener on loopback;
do not expose port 8853 or 9090 directly on the LAN or public internet. Configure
the tailnet rules to allow only your account/device to reach this machine.

The VRM is loaded from this interface's local `/assets/AGi.vrm` path. Three.js,
three-vrm, and roslib are loaded from jsDelivr, so the browser needs internet
access for those libraries. The small chat portrait is captured from the main
avatar render and reuses that image rather than loading another model.
