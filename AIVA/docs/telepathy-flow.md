# Telepathy chat flow

This page records the source-level request/response path for Grace's current
PC runtime. For the complete process map and repository ownership, see
[`../../docs/architecture.md`](../../docs/architecture.md). The checked-in
component changes are merged on the component `dev` branches; this document
does not confirm which code or processes are currently running on a PC.

## Text conversation

```mermaid
sequenceDiagram
    actor User
    participant Browser as Console browser
    participant Console as Console HTTP API / WebSocket proxy
    participant Bridge as rosbridge / ROS 2
    participant Receptor as Telepathy receptor node
    participant Engine as Interactive library
    participant Memory as Memory library / SQLite
    participant Model as OpenAI-compatible model endpoint

    Browser->>Console: Open /rosbridge WebSocket
    Console->>Bridge: Open upstream WebSocket
    Browser->>Console: Subscribe /telepathic/response through ROSLIB
    Console->>Bridge: Forward ROSLIB subscription
    User->>Browser: Submit text
    Browser->>Console: HTTP POST /api/telepathic/send
    Console->>Console: Encode with Cypher
    Console->>Bridge: Publish TelepathicSignal over WebSocket
    Bridge->>Receptor: /telepathic/stimulus
    Receptor->>Receptor: Decode signal and parse request
    Receptor->>Engine: answer(text, request ID)
    Engine->>Memory: Load recent exchanges
    Memory-->>Engine: Up to 8 recent exchanges by default
    Engine->>Model: Streaming /v1/chat/completions
    loop response chunks
        Model-->>Engine: Text delta
        Engine->>Receptor: start / delta event
        Receptor->>Bridge: JSON std_msgs/String on /telepathic/response
        Bridge-->>Console: ROS event over rosbridge WebSocket
        Console-->>Browser: Relay event through /rosbridge proxy
    end
    Engine->>Memory: Persist completed exchange
    Engine->>Receptor: done event
    Receptor->>Bridge: JSON std_msgs/String on /telepathic/response
    Bridge-->>Console: Completion event over rosbridge WebSocket
    Console-->>Browser: Relay completion through /rosbridge proxy
```

The browser subscribes to the ROS response topic through Console's transparent
`/rosbridge` WebSocket proxy; Console relays those ROS messages without parsing
or repackaging the text stream. Console encodes the submitted request before
ROS publication. The current
Cypher scheme is reversible encoding, not encryption. The original prompt
travels from browser to Console over HTTP; use HTTPS if that leg leaves a
trusted local network.

Interactive assembles the persona, current time, recent exchanges, and current
prompt, calls the configured model endpoint, and persists the completed turn
through Memory. Memory is a Python library in the receptor process, not a
separate ROS node or database service. The default store is SQLite. Older turns
remain stored, but this chat path currently passes only the configured recent
window to the model; semantic retrieval is not part of this flow.

## Canvas requests

Canvas Studio sends requests over Console's `/api/canvas/ws` WebSocket. Console
encodes the request and publishes a `TelepathicSignal` on
`/telepathic/stimulus`, then relays matching `/telepathic/response` events to
the page. The receptor handles drawing orchestration, calls the configured
model with the Canvas action schema, and publishes progress, actions, replies,
and completion events. When requested, raster painting uses the optional MOSS
sidecar; Canvas and Imaging packages and the painting sidecar are not ROS
nodes. The page sends review screenshots over the same Console WebSocket, and
Console publishes them on `/aiva/canvas_snapshot` for the pending draw request.

## Processes and topics

| Name | Kind | Role |
|---|---|---|
| `telepathic_console` | Standalone web process | Serves the external Console and exchanges events with rosbridge |
| `/telepathy_receptor` | ROS 2 node | Decodes stimulus signals, runs chat/Canvas handling, publishes response events |
| `/agi_log_collector` | ROS 2 node | Collects ROS logs and publishes log events |
| `/rosbridge_websocket`, `/rosapi` | ROS 2 nodes | Connect the Console process to ROS 2 |
| `agi-interactive`, `agi-memory`, Cypher codecs | In-process libraries | Conversation, storage, and signal decoding |
| llama.cpp or configured compatible server | Separate model process/service | Chat and any configured model endpoints |

The old `aiva_chat` node and `/aiva/chat_input` subscriber are retired from
this flow. `/tms/text_response` is also retired; responses use
`/telepathic/response`. AuRoRA's SCS, HRM, and TMS launch path is separate and
is not started by AIVA.

## Check the local ROS graph

Run these after syncing the component `dev` branches, rebuilding the Shell
workspace, and starting Grace in a fresh terminal:

```sh
ros2 node list
ros2 topic info /telepathic/stimulus -v
ros2 topic info /telepathic/response -v
ros2 topic info /aiva/canvas_snapshot -v
ros2 pkg list | rg '^(telepathy_receptor|telepathic_management|aiva_chat)$'
```

After a clean rebuild, `telepathy_receptor` and `telepathic_management` should
be installed; `aiva_chat` should not. A stale `build/` or `install/` overlay
can make retired packages appear, so verify from a terminal that has sourced
the intended ROS and workspace setup files.
