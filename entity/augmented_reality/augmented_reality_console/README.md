# Grace AR for Android

This Unity project places Grace's complete liquid chamber in an Android ARCore scene. Grace remains inside the transparent tube. The chamber is a world-space object, so it disappears naturally when the phone camera is pointed away and reappears when the user looks back during the same AR session.

## Current milestone

- Android-only Unity project using AR Foundation and Google ARCore.
- Loads the existing `Grace.vrm` model at runtime.
- Procedural transparent chamber, liquid, illuminated rings, and bubbles.
- Tap a detected floor to place or move the chamber.
- Grace gently floats and has initial **Wave** and **Turn** actions.
- Real AR anchors keep the chamber placed during the current tracking session.
- Printable image marker restores its saved room-relative placement after restarting.
- Connect to AIVA's WebUI gateway for streamed chat through ROS, using Grace's shared memory.
- Canvas button opens the hosted Canvas Studio in the phone's browser.

The marker workflow saves the chamber pose relative to a printed physical image, rather than
saving ARCore world coordinates. Rescanning that fixed marker restores the pose in a new session.
Accuracy depends on ARCore tracking, lighting, and keeping the physical marker in the same place.
This implementation still needs a Unity Android build and real-device validation.

## Open and build

1. Install Unity Hub and Unity 6, including **Android Build Support**, **Android SDK & NDK Tools**, and **OpenJDK**.
2. Add this directory as a Unity project:
   `entity/augmented_reality/augmented_reality_console`
3. Let Unity resolve the AR Foundation, ARCore, and UniVRM packages.
4. Run **Grace AR > Configure Android and Create Scene** from Unity's menu.
5. Open **Edit > Project Settings > XR Plug-in Management > Android** and enable **ARCore**.
6. Connect an ARCore-supported Android phone with USB debugging enabled.
7. Use **File > Build Profiles**, select Android, then **Build and Run**.

The first launch needs camera permission. Move the phone slowly until a floor is detected, then tap where Grace's chamber should stand.

## Project layout

- `Assets/StreamingAssets/Grace.vrm`: Grace's runtime model.
- `Assets/GraceAR/Scripts/GraceArApp.cs`: AR session, placement, and controls.
- `Assets/GraceAR/Scripts/GraceChamber.cs`: procedural tube and liquid.
- `Assets/GraceAR/Scripts/GraceVrmLoader.cs`: Android-safe VRM loading.
- `Assets/GraceAR/Scripts/GraceActionController.cs`: floating and actions.
- `Assets/GraceAR/Editor/GraceArProjectSetup.cs`: Android and scene setup.


## Remember a room location

The setup menu generates `Assets/GraceAR/GraceMarker.png` and a reference-image library.
Print the **whole square image at exactly 15 × 15 cm**, without cropping, and mount it flat
in a fixed, well-lit location near the chamber. Scan the marker first, then use **Place / Move**
and tap a horizontal surface. The phone saves the relative pose in its app preferences.
After reopening, scan the same marker: the chamber is restored and attached to a new session
anchor. Once restored, the marker may leave the camera view. Moving the physical marker moves
the reference location; clearing app data removes the saved pose. Placement without a visible
tracked marker is session-only. Marker recognition quality must be checked on the target phone.

## Connect to Grace and Canvas

Start AIVA, then enter a phone-reachable WebUI gateway URL in the AR app and tap **Connect to
AIVA**. Use the HTTPS address from Tailscale Serve, or the PC's LAN address if the console is
explicitly bound to its LAN interface. `127.0.0.1` on a phone refers to the phone, not the PC.
The project allows HTTP for a local gateway; HTTPS uses WSS for the ROS bridge.

AR chat uses `/rosbridge`, publishes JSON prompts on `/aiva/chat_input`, and receives streamed
replies on `/tms/text_response`. Request IDs keep responses separate from other clients. The
phone stores the gateway and room pose locally; LLM credentials and conversation memory stay
on AIVA. AIVA must be online for chat. **Canvas** opens `/canvas` in the system browser; drawing
is not embedded onto a surface in the AR scene. The browser keeps its own drawing document.
Canvas requests use Grace's recent shared conversation context and persist prompts, replies,
and the proposed drawing commands to the same SQLite memory as normal chat.

Rebuild AIVA as well as the Android app for correlated chat events. Check ARCore is enabled in
XR Plug-in Management after running setup. The setup keeps Standard/MToon shaders for runtime
VRM loading and adds a key light. Hardware checks remaining: package import/build, camera
permissions, marker detection/scale, anchor drift, avatar appearance, and gateway streaming.
