/* Grace's 3D VRM is shown when the AIVA PC presence heartbeat is online. */
(() => {
  'use strict';

  const canvas = document.getElementById('robotVrmCanvas');
  const container = document.getElementById('avatarContainer');
  if (!canvas || !container) return;

  let avatar = null;
  let frame = 0;
  let loadPromise = null;
  let aivaOnline = false;
  let auroraOnline = window.agiAuroraOnline === true;
  let nextFrameAt = 0;
  let idleTime = 0;
  let blinkWait = 1.5 + Math.random() * 2.5;
  let blinkTime = 0;
  let blinking = false;

  // A small procedural model keeps AuRoRA in the same depth buffer as Grace.
  function createRobot(THREE, height) {
    const group = new THREE.Group();
    const unit = height * 0.13;
    group.scale.setScalar(unit);
    const sphere = new THREE.SphereGeometry(1, 24, 16);
    const pink = new THREE.MeshStandardMaterial({ color: 0xf3b9d5, roughness: 0.38, metalness: 0.08 });
    const dark = new THREE.MeshStandardMaterial({ color: 0x18263f, roughness: 0.32 });
    const pale = new THREE.MeshStandardMaterial({ color: 0xe4f5ff, roughness: 0.35 });
    const accent = new THREE.MeshStandardMaterial({ color: 0xf4479c, roughness: 0.4 });
    function part(material, x, y, z, sx, sy, sz) {
      const mesh = new THREE.Mesh(sphere, material);
      mesh.position.set(x, y, z);
      mesh.scale.set(sx, sy, sz);
      group.add(mesh);
      return mesh;
    }
    part(pink, 0, -0.48, 0, 0.49, 0.42, 0.29);
    part(pink, 0, 0.15, 0, 0.72, 0.58, 0.38);
    part(dark, 0, 0.16, 0.345, 0.54, 0.32, 0.07);
    part(pale, -0.25, 0.22, 0.409, 0.12, 0.095, 0.025);
    part(pale, 0.25, 0.22, 0.409, 0.12, 0.095, 0.025);
    part(accent, 0, -0.04, 0.41, 0.13, 0.06, 0.025);
    part(accent, 0, -0.48, 0.285, 0.12, 0.11, 0.025);
    for (const side of [-1, 1]) {
      part(pink, side * 0.75, -0.22, 0, 0.12, 0.27, 0.13);
      part(pink, side * 0.38, 0.73, 0, 0.055, 0.25, 0.055);
      part(pale, side * 0.38, 0.96, 0, 0.09, 0.09, 0.09);
    }
    group.visible = false;
    return group;
  }

  function frameCamera() {
    if (!avatar) return;
    const { camera, modelHeight } = avatar;
    const soloPortrait = !auroraOnline;
    // In solo mode, crop to a close chest-up portrait. Leave more room when
    // AuRoRA is beside her so both characters remain comfortably in frame.
    const distance = soloPortrait ? 0.68 : 0.92;
    const targetHeight = soloPortrait ? 0.84 : 0.82;
    camera.position.set(0, modelHeight * (targetHeight + 0.05), modelHeight * distance);
    camera.lookAt(0, modelHeight * targetHeight, 0);
    camera.updateProjectionMatrix();
  }

  function rememberRestPose(vrm) {
    const bones = {};
    const humanoid = vrm.humanoid;
    // The Aiko VRM viewer drives raw bones directly. VRM 3.x otherwise
    // copies the normalized T-pose over them on every update.
    if (humanoid) humanoid.autoUpdateHumanBones = false;
    for (const name of ['spine', 'chest', 'neck', 'head', 'leftShoulder', 'rightShoulder', 'leftUpperArm', 'rightUpperArm', 'leftLowerArm', 'rightLowerArm', 'leftHand', 'rightHand']) {
      const bone = humanoid?.getRawBoneNode(name);
      if (bone) bones[name] = { bone, x: bone.rotation.x, y: bone.rotation.y, z: bone.rotation.z };
    }
    // Set the same relaxed arm rest pose used by Aiko's VRM, so a model
    // exported in a T-pose doesn't stay frozen with its arms outstretched.
    if (bones.leftUpperArm) bones.leftUpperArm.z = 1.28;
    if (bones.rightUpperArm) bones.rightUpperArm.z = -1.28;
    if (bones.leftLowerArm) { bones.leftLowerArm.x += 0.12; bones.leftLowerArm.z += 0.08; }
    if (bones.rightLowerArm) { bones.rightLowerArm.x += 0.12; bones.rightLowerArm.z -= 0.08; }
    return bones;
  }

  function applyIdle(delta, animateMotion = true) {
    const t = idleTime;
    const breathe = Math.sin((2 * Math.PI * t) / 3.5);
    const drift = (frequency, phase = 0) => Math.sin(2 * Math.PI * frequency * t + phase);
    const pose = avatar.restPose;
    const apply = (name, x = 0, y = 0, z = 0) => {
      const item = pose[name];
      if (item) item.bone.rotation.set(item.x + x, item.y + y, item.z + z);
    };
    // Match Aiko's gentle, asynchronous breathing and tiny head micro-sway.
    apply('spine', 0.007 * Math.sin((2 * Math.PI * (t - 0.08)) / 3.5));
    apply('chest', 0.014 * breathe);
    apply('head', 0.0045 * drift(0.47, 0.4), 0.016 * drift(0.41) + 0.007 * drift(0.53, 2.1), 0.006 * drift(0.19, 1.1));
    apply('neck', 0.003 * drift(0.67, 1.7), 0.004 * drift(0.59), 0);
    const holdingRobot = auroraOnline;
    const leftUpper = pose.leftUpperArm;
    const rightUpper = pose.rightUpperArm;
    const leftLower = pose.leftLowerArm;
    const rightLower = pose.rightLowerArm;
    apply('leftShoulder', 0, holdingRobot ? -0.08 : 0, 0);
    apply('rightShoulder', 0, holdingRobot ? 0.08 : 0, 0);
    apply('leftUpperArm', 0.008 * drift(0.71, 0.5), holdingRobot ? -0.30 : 0.004 * drift(0.43, 2.2), holdingRobot && leftUpper ? 0.90 - leftUpper.z : 0.007 * drift(0.61, 1.0));
    apply('rightUpperArm', 0.008 * drift(0.73, 2.8), holdingRobot ? 0.30 : 0.004 * drift(0.83, 0.9), holdingRobot && rightUpper ? -0.90 - rightUpper.z : 0.007 * drift(0.79, 1.6));
    apply('leftLowerArm', 0.006 * drift(0.89, 0.2), holdingRobot ? 0.22 : 0.003 * drift(0.97, 1.9), holdingRobot && leftLower ? 2.15 - leftLower.z : 0.005 * drift(0.49, 0.9));
    apply('rightLowerArm', 0.006 * drift(1.02, 1.2), holdingRobot ? -0.22 : 0.003 * drift(1.11, 2.5), holdingRobot && rightLower ? -2.15 - rightLower.z : 0.005 * drift(0.51, 0.4));
    apply('leftHand', 0, 0, holdingRobot ? -0.18 : 0);
    apply('rightHand', 0, 0, holdingRobot ? 0.18 : 0);

    if (!animateMotion) return;
    const expressions = avatar.vrm.expressionManager;
    if (!expressions) return;
    if (!blinking) {
      blinkWait -= delta;
      if (blinkWait <= 0) { blinking = true; blinkTime = 0; }
      return;
    }
    blinkTime += delta;
    const weight = Math.sin(Math.PI * Math.min(1, blinkTime / 0.12));
    try { expressions.setValue('blink', weight); } catch (_) { /* VRM may omit blink. */ }
    if (blinkTime >= 0.12) {
      blinking = false;
      try { expressions.setValue('blink', 0); } catch (_) { /* Optional expression. */ }
      blinkWait = 2 + Math.random() * 4;
    }
  }

  async function loadAvatar() {
    if (avatar) return avatar;
    if (loadPromise) return loadPromise;
    loadPromise = (async () => {
      const THREE = await import('three');
      const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js');
      const { VRMLoaderPlugin, VRMUtils } = await import('@pixiv/three-vrm');

      const renderer = new THREE.WebGLRenderer({
        canvas, alpha: true, antialias: true, powerPreference: 'low-power',
      });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.25));
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.setClearColor(0x000000, 0);

      const scene = new THREE.Scene();
      scene.add(new THREE.HemisphereLight(0xffffff, 0x8c79aa, 2.0));
      const key = new THREE.DirectionalLight(0xffffff, 2.3);
      key.position.set(-1, 2, 3);
      scene.add(key);
      const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 30);

      const loader = new GLTFLoader();
      loader.register((parser) => new VRMLoaderPlugin(parser));
      const gltf = await loader.loadAsync('/assets/AGi.vrm');
      const vrm = gltf.userData.vrm;
      if (!vrm) throw new Error('The avatar file did not contain a VRM model');
      VRMUtils.removeUnnecessaryVertices(gltf.scene);
      VRMUtils.combineSkeletons(gltf.scene);
      scene.add(vrm.scene);
      vrm.scene.rotation.y = Math.PI;

      const bounds = new THREE.Box3().setFromObject(vrm.scene);
      const center = bounds.getCenter(new THREE.Vector3());
      vrm.scene.position.x -= center.x;
      vrm.scene.position.y -= bounds.min.y;
      const modelHeight = Math.max(bounds.max.y - bounds.min.y, 1.2);
      const robot3d = createRobot(THREE, modelHeight);
      scene.add(robot3d);
      const raycaster = new THREE.Raycaster();
      const pointer = new THREE.Vector2();
      const leftHandPosition = new THREE.Vector3();
      const rightHandPosition = new THREE.Vector3();
      canvas.addEventListener('click', (event) => {
        if (!aivaOnline || !auroraOnline || !robot3d.visible) return;
        const rect = canvas.getBoundingClientRect();
        pointer.set((event.clientX - rect.left) / rect.width * 2 - 1, 1 - (event.clientY - rect.top) / rect.height * 2);
        raycaster.setFromCamera(pointer, camera);
        const hit = raycaster.intersectObjects(robot3d.children, false)[0];
        if (!hit) return;
        const point = robot3d.worldToLocal(hit.point.clone());
        const id = point.y > 0.45 ? 'petZone' : (point.x < 0 ? 'pokeZoneLeft' : 'pokeZoneRight');
        document.getElementById(id)?.dispatchEvent(new MouseEvent('click', {
          bubbles: true, clientX: event.clientX, clientY: event.clientY,
        }));
      });
      // Framing is tighter when Grace is alone and wider beside AuRoRA.

      const resize = () => {
        if (!aivaOnline) return;
        const { width, height } = canvas.getBoundingClientRect();
        if (!width || !height) return;
        renderer.setSize(width, height, false);
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
      };
      const resizeObserver = new ResizeObserver(resize);
      resizeObserver.observe(canvas);
      avatar = { renderer, scene, camera, vrm, robot3d, leftHandPosition, rightHandPosition, modelHeight, restPose: rememberRestPose(vrm), clock: new THREE.Clock(), resize };
      frameCamera();
      return avatar;
    })();
    try {
      return await loadPromise;
    } catch (error) {
      loadPromise = null;
      throw error;
    }
  }

  function stopRender() {
    cancelAnimationFrame(frame);
    frame = 0;
    nextFrameAt = 0;
    if (avatar) avatar.clock.getDelta();
  }

  function render(now) {
    if (!aivaOnline || !avatar) return;
    frame = requestAnimationFrame(render);
    if (now < nextFrameAt) return;
    nextFrameAt = now + 33;
    const delta = Math.min(avatar.clock.getDelta(), 0.05);
    const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (!reducedMotion) idleTime += delta;
    applyIdle(delta, !reducedMotion);
    avatar.vrm.update(delta);
    avatar.robot3d.visible = auroraOnline;
    const leftHand = avatar.restPose.leftHand?.bone;
    const rightHand = avatar.restPose.rightHand?.bone;
    if (auroraOnline && leftHand && rightHand) {
      avatar.vrm.scene.updateMatrixWorld(true);
      leftHand.getWorldPosition(avatar.leftHandPosition);
      rightHand.getWorldPosition(avatar.rightHandPosition);
      avatar.robot3d.position.copy(avatar.leftHandPosition).add(avatar.rightHandPosition).multiplyScalar(0.5);
      avatar.robot3d.position.y += avatar.modelHeight * 0.025;
      avatar.robot3d.position.z -= avatar.modelHeight * 0.075;
    } else if (auroraOnline) {
      avatar.robot3d.visible = false;
    }
    avatar.renderer.render(avatar.scene, avatar.camera);
  }

  async function syncPresence() {
    container.classList.toggle('aiva-online', aivaOnline && auroraOnline);
    container.classList.toggle('grace-only', aivaOnline && !auroraOnline);
    if (!aivaOnline) {
      stopRender();
      return;
    }
    try {
      await loadAvatar();
      frameCamera();
      avatar.resize();
      if (!frame) frame = requestAnimationFrame(render);
    } catch (error) {
      console.error('Could not load AGi.vrm:', error);
      container.classList.remove('aiva-online', 'grace-only');
      const presence = document.getElementById('aivaPresence');
      if (presence) presence.textContent = 'AIVA avatar unavailable';
    }
  }

  window.addEventListener('agi-aiva-presence', (event) => {
    aivaOnline = Boolean(event.detail?.online);
    syncPresence();
  });
  window.addEventListener('agi-aurora-presence', (event) => {
    auroraOnline = Boolean(event.detail?.online);
    syncPresence();
  });
})();
