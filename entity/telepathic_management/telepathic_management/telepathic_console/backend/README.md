# Telepathic Console assets

`telepathic_management/telepathic_console/server.py` serves these static assets
and the pages in `../frontend/`, and proxies the browser WebSocket to rosbridge.

The companion page pairs a live VRM chamber with a scrolling conversation.
`console.css` owns its responsive layout and dark/light colors; `styles.css`
retains the original robot animation styles. The dark palette is the default
for a new companion page, while saved theme preferences are honored.

`avatar.js` loads Grace when AIVA presence is online. Full body and Portrait
reset the camera, dragging orbits it, and pointer motion gently turns her head.
The chamber background is a local WebP asset. Nineteen bubbles rise in front
of and behind the avatar, with moving light refractions and twinkling glints.
The decorative layers let pointer events pass through to the camera controls.
Bubbles, reflections, glints, and floating motion respect reduced-motion preferences. Chat badges are snapshots of the loaded
VRM; they fall back to a neutral emblem if capture is unavailable.

The empty chamber remains visible while AIVA is offline. Chat submission and
response streaming continue to use the existing telepathic API and ROS bridge.
The UI does not require changes to Grace's LLM or PostgreSQL memory service.

ROS packaging installs the frontend directory and all backend static assets,
including `console.css` and `assets/grace-chamber.webp`.
