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
  let cameraMode = 'full';
  let lookX = 0;
  let lookY = 0;
  let targetLookX = 0;
  let targetLookY = 0;

  function frameCamera(mode = cameraMode) {
    cameraMode = mode;
    const zoomSwitch = document.getElementById('avatarZoomSwitch');
    if (zoomSwitch) zoomSwitch.checked = mode === 'portrait';
    document.querySelector('.chamber-panel')?.classList.toggle('portrait-zoom', mode === 'portrait');
    if (!avatar) return;
    const { camera, controls, modelHeight } = avatar;
    const distance = mode === 'full' ? 2.3 : 0.72;
    const targetHeight = mode === 'full' ? 0.52 : 0.79;
    camera.position.set(0, modelHeight * targetHeight, modelHeight * distance);
    controls.target.set(0, modelHeight * targetHeight, 0);
    controls.update();
    camera.updateProjectionMatrix();
  }

  function rememberRestPose(vrm) {
    const bones = {};
    const humanoid = vrm.humanoid;
    // The Aiko VRM viewer drives raw bones directly. VRM 3.x otherwise
    // copies the normalized T-pose over them on every update.
    if (humanoid) humanoid.autoUpdateHumanBones = false;
    for (const name of ['spine', 'chest', 'neck', 'head', 'leftUpperArm', 'rightUpperArm', 'leftLowerArm', 'rightLowerArm']) {
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
    apply('head', 0.0045 * drift(0.47, 0.4) - lookY * 0.075, 0.016 * drift(0.41) + 0.007 * drift(0.53, 2.1) + lookX * 0.12, 0.006 * drift(0.19, 1.1));
    apply('neck', 0.003 * drift(0.67, 1.7), 0.004 * drift(0.59), 0);
    apply('leftUpperArm', 0.008 * drift(0.71, 0.5), 0.004 * drift(0.43, 2.2), 0.007 * drift(0.61, 1.0));
    apply('rightUpperArm', 0.008 * drift(0.73, 2.8), 0.004 * drift(0.83, 0.9), 0.007 * drift(0.79, 1.6));
    apply('leftLowerArm', 0.006 * drift(0.89, 0.2), 0.003 * drift(0.97, 1.9), 0.005 * drift(0.49, 0.9));
    apply('rightLowerArm', 0.006 * drift(1.02, 1.2), 0.003 * drift(1.11, 2.5), 0.005 * drift(0.51, 0.4));

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
      const { OrbitControls } = await import('three/addons/controls/OrbitControls.js');

      const renderer = new THREE.WebGLRenderer({
        canvas, alpha: true, antialias: true, powerPreference: 'low-power',
      });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.25));
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.setClearColor(0x000000, 0);

      const scene = new THREE.Scene();
      scene.add(new THREE.HemisphereLight(0xd7faff, 0x164279, 1.6));
      const key = new THREE.DirectionalLight(0xedfaff, 1.8);
      key.position.set(-1, 2, 3);
      scene.add(key);
      const rim = new THREE.DirectionalLight(0x53dfff, 1.5);
      rim.position.set(1, 1, -1);
      scene.add(rim);
      const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 30);
      const controls = new OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.dampingFactor = 0.08;
      controls.enablePan = false;

      const loader = new GLTFLoader();
      loader.register((parser) => new VRMLoaderPlugin(parser));
      const gltf = await loader.loadAsync('/static/assets/Grace.vrm');
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
      controls.minDistance = modelHeight * 0.3;
      controls.maxDistance = modelHeight * 2.5;
      controls.minPolarAngle = 0.08;
      controls.maxPolarAngle = Math.PI - 0.08;
      // frameCamera applies the close chest portrait after avatar state is set.

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
      avatar = { renderer, scene, camera, controls, vrm, modelHeight, restPose: rememberRestPose(vrm), baseY: vrm.scene.position.y, clock: new THREE.Clock(), resize };
      resize();
      // Capture the loaded model itself for the header and conversation badges.
      const selectedMode = cameraMode;
      frameCamera('portrait');
      applyIdle(0, false);
      vrm.update(0);
      renderer.render(scene, camera);
      try {
        document.body.style.setProperty('--grace-portrait', `url("${canvas.toDataURL('image/webp')}")`);
        document.body.classList.add('avatar-portrait-ready');
      } catch (_) { /* Keep the neutral badge if canvas capture is unavailable. */ }
      frameCamera(selectedMode);
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
    const lookEase = Math.min(1, delta * 5);
    lookX += (targetLookX - lookX) * lookEase;
    lookY += (targetLookY - lookY) * lookEase;
    const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (!reducedMotion) idleTime += delta;
    avatar.vrm.scene.position.y = avatar.baseY + (reducedMotion ? 0 : Math.sin(idleTime * 0.8) * avatar.modelHeight * 0.006);
    applyIdle(delta, !reducedMotion);
    avatar.vrm.update(delta);
    avatar.controls.update();
    avatar.renderer.render(avatar.scene, avatar.camera);
  }

  canvas.addEventListener('pointermove', (event) => {
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    targetLookX = Math.max(-1, Math.min(1, ((event.clientX - rect.left) / rect.width - 0.5) * 2));
    targetLookY = Math.max(-1, Math.min(1, ((event.clientY - rect.top) / rect.height - 0.5) * 2));
  });
  canvas.addEventListener('pointerleave', () => { targetLookX = 0; targetLookY = 0; });
  document.getElementById('avatarZoomSwitch')?.addEventListener('change', (event) => {
    frameCamera(event.target.checked ? 'portrait' : 'full');
  });

  async function syncPresence() {
    container.classList.toggle('aurora-online', auroraOnline);
    container.classList.toggle('aiva-online', aivaOnline && auroraOnline);
    container.classList.toggle('grace-only', aivaOnline && !auroraOnline);
    if (!aivaOnline) {
      stopRender();
      return;
    }
    try {
      await loadAvatar();
      avatar.resize();
      if (!frame) frame = requestAnimationFrame(render);
    } catch (error) {
      console.error('Could not load Grace.vrm:', error);
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
