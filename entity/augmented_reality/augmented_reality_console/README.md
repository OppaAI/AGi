# Grace AR for Android

This Unity project places Grace's complete liquid chamber in an Android ARCore scene. Grace remains inside the transparent tube. The chamber is a world-space object, so it disappears naturally when the phone camera is pointed away and reappears when the user looks back during the same AR session.

## Current milestone

- Android-only Unity project using AR Foundation and Google ARCore.
- Loads the existing `Grace.vrm` model at runtime.
- Procedural transparent chamber, liquid, illuminated rings, and bubbles.
- Tap a detected floor to place or move the chamber.
- Grace gently floats and has initial **Wave** and **Turn** actions.
- The chamber remains fixed while ARCore's current tracking session is alive.

Persistent recognition of the exact lab corner after fully closing the app will use an image marker mounted near that corner. That is the next milestone; a plain saved coordinate is not reliable because ARCore creates a new origin in each session.

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

