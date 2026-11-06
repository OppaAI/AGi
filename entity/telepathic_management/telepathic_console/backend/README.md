# Web Teleops Console backend assets

The ROS-aware FastAPI server is implemented in
`telepathic_management/telepathic_management/telepathic_console.py` serves the
frontend files packaged alongside it and proxies browser WebSockets to
rosbridge.
so it can run as part of the TMS ROS 2 package. This folder contains the static
files served by that backend. Browser pages live in `../frontend/`.
