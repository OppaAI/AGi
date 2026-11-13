# Telepathic Console assets

`telepathic_management/telepathic_console/server.py` serves these static assets
and the pages in `../frontend/`, and proxies the browser WebSocket to rosbridge.

The companion page pairs a live VRM chamber with a scrolling conversation.
`console.css` owns its responsive layout and dark/light colors; `styles.css`
retains the original robot animation styles. The dark palette is the default
for a new companion page, while saved theme preferences are honored.

`avatar.js` loads Grace when AIVA presence is online. The camera switch is off for full body and on for a zoomed-in portrait;
changing it resets the camera, dragging orbits it, and pointer motion gently turns her head.
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

## System logs

The AIVA launcher stores service output and ROS logs under `~/.agi/log` by
default. `AIVA_LOG_DIR` can override that location. ROS log records are collected
by the `agi_logging` ROS node, saved as rotating `application.jsonl` and
`errors.jsonl`, and published as JSON strings on `/agi/logs` for other ROS
components. The WebUI **Logs** page reads recent entries from `/api/logs` and
can filter by severity or refresh automatically. The collector keeps five
10 MiB rotations per file by default.

## Canvas page

Open **Canvas** from the chat header, or visit `/canvas`. Brush, eraser, line,
rectangle, ellipse, text, object selection/move/delete, undo/redo, clear and PNG
export work locally. Text is placed by clicking after entering it in the toolbar.
Artwork persists in that browser's local storage (up to 100 objects).

For letters, enter paragraphs in the multiline Text field and choose **Add letter
page**, or ask Grace to write a letter. A portrait page uses serif text, automatic
word wrapping (including long URLs), blank lines between paragraphs, adjustable
margins, line spacing, and left/center/right alignment. New pages use a 16 px font;
select a page with **Select / move** to load its text and formatting, adjust Size
or other controls, then choose **Update letter page**. Changes support undo/redo.
Letters currently fit one page; overflow displays an error so text is not silently
cut off. Shorten the letter or reduce font size, margins, or spacing. PNG export
includes the formatted page. Grace uses the `letter` canvas action for this layout.

The page sends a request and the current drawing objects/selection to AIVA's
`/aiva/chat_input`. AIVA calls the configured OpenAI-compatible model with a
`canvas_draw` tool. This requires an endpoint/model supporting `tools` and
`tool_choice`. AIVA validates the resulting actions and publishes them on
`/tms/text_response`, tagged with the request identifier. The requesting page
replays strokes, shapes, and text visibly; **Stop Grace** interrupts replay and
ignores later results for that request. Completed actions can be undone.

Grace sees object coordinates and text supplied by the page. The page also renders a
downscaled JPEG screenshot of the canvas and sends it with each request, and again when
Grace asks for a fresh look between passes (`canvas_snapshot_request` answered on
`/aiva/canvas_snapshot`), so a vision-capable endpoint can observe what it actually drew
and correct placement before refining. Endpoints that reject image content fall back to
text-only planning automatically. Planning occurs before the animated replay. Browser pages
have independent artwork; the memory service records the requested action plan,
while the actual editable document is saved locally. PNG export saves an image.
Rebuild/source AIVA's packages after updating so the node and packaged pages match.

### Raster painting (MOSS-Image)

In addition to vector actions, Grace can request a raster painting with the canvas
`image` action: `{"kind": "image", "prompt": "...", "x", "y", "width", "height"}`. The
server permits at most two image actions per request and preserves their action order
relative to vector overlays and edits. AIVA generates the bitmap server-side through the MossInfer
`moss-server` sidecar and publishes it as a separate `canvas_image` event — the bitmap
never travels through the model or `canvas_actions`. On failure she publishes
`canvas_error` and keeps layers already received; a failed image is not marked finished. An image action counts as finished
art, so it does not trigger the vector refinement loop. Raster paintings cannot be
recolored in place; move, delete, or repaint them instead. Painting is disabled unless the
moss-server binary and the MOSS-Image weights are installed (see the AIVA README,
MossInfer section). The browser waits for images to decode before taking review screenshots
or exporting PNGs. Later requests send image metadata and a downscaled screenshot rather
than full PNG payloads; old saved pictures without a prompt remain editable. Editable
artwork imports accept up to 32 MiB. If browser storage fills, use **Save artwork**.

### Painting studio

Canvas Studio adds a 16-color artist palette plus arbitrary hex colors, separate
outline and fill colors, ink/soft/marker brushes, transparency, linear/radial
fills, and closed freehand contours. Select an object to load its paint settings,
then use **Apply to selection**; front/back controls reorder objects. Letter
controls are under **Typography & letter layout**. Existing saved art still loads.

Grace's drawing tool accepts `polygon`, `smooth`, `fill_color`, `gradient_color`,
`gradient`, `opacity`, `brush`, and `stroke`, plus `style`, `front`, and `back`
editing actions. Her instructions encourage layered compositions, varied palettes,
consistent lighting, and shadows/highlights. This expands vector illustration
capabilities. On top of vectors, Grace can also request a raster painting (canvas
`image` action), rendered server-side by the MossInfer MOSS-Image engine and published
as a `canvas_image` event. Live drawing quality still depends on the configured model's
tool output.

### Vision feedback

With a vision-capable model (such as Ministral 3 14B Instruct) as `AIVA_LLM_MODEL`, Grace
compares an attached canvas screenshot with the request during drawing and can reposition,
restyle, or delete her own shapes between passes instead of planning blind. Set
`AIVA_SNAPSHOT_TIMEOUT_SECONDS` (default 10) to tune how long a pass waits for the browser's
screenshot; a text-only endpoint is used automatically if images are rejected.
